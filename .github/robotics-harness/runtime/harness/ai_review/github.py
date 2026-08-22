"""GitHub Actions adapter for immutable-head Codex PR review.

This module deliberately exposes no approve, merge, branch-write, or source-edit
operation. It only reads pull-request data and publishes review evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from harness.ai_review.diff import (
    DiffBundle,
    DiffCollectionError,
    collect_full_diff,
)
from harness.ai_review.models import (
    AIReview,
    Finding,
    FindingState,
    ReviewContext,
    ReviewResult,
    format_timestamp,
)
from harness.ai_review.provider import (
    ProviderError,
    RecordedReviewProvider,
    ReviewProvider,
)
from harness.ai_review.report import (
    SUMMARY_MARKER,
    decode_review_state,
    render_summary_comment,
    safe_markdown_text,
    write_review_artifacts,
)
from harness.ai_review.service import AIReviewService, PullRequestMetadata
from harness.git import ChangedFile as GitChangedFile
from harness.monitoring import EventStore, EventType, MonitoringEvent
from harness.private_io import write_private_text
from harness.risk.classifier import RiskAssessment, RiskClassifier

_OBJECT_ID = re.compile(r"^[0-9a-fA-F]{40,64}$")
_HUNK = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,\d+)? @@")
_INLINE_SEVERITIES = {"critical", "high", "medium"}
_POLICY_KEYWORDS = (
    "must",
    "never",
    "required",
    "prohibit",
    "protect",
    "safety",
    "approval",
    "review",
    "test",
    "scope",
    "agent",
    "competition",
    "build",
    "gradle",
)
_SELF_REVIEW_CHECK_NAMES = frozenset({"Agentic Review", "Codex PR Review"})


class GitHubApiError(RuntimeError):
    def __init__(self, message: str, *, status_code: int | None = None) -> None:
        super().__init__(message)
        self.status_code = status_code


class _UnavailableProvider:
    def __init__(self, reason: str) -> None:
        self.reason = reason

    @property
    def name(self) -> str:
        return "unavailable"

    def review(self, request: Any) -> Any:
        del request
        raise ProviderError(self.reason)


@dataclass(frozen=True, slots=True)
class PullRequestEvent:
    action: str
    number: int
    repository: str
    base_repository: str
    head_repository: str
    base_sha: str
    head_sha: str
    base_ref: str
    head_ref: str
    title: str
    body: str

    @property
    def from_fork(self) -> bool:
        return self.base_repository.casefold() != self.head_repository.casefold()

    @classmethod
    def from_dict(cls, value: object) -> PullRequestEvent:
        if not isinstance(value, dict):
            raise ValueError("GitHub event must be an object")
        pull_request = value.get("pull_request")
        repository = value.get("repository")
        if not isinstance(pull_request, dict) or not isinstance(repository, dict):
            raise ValueError("event does not contain pull_request and repository objects")
        action = str(value.get("action", ""))
        if action not in {"opened", "synchronize", "reopened", "ready_for_review"}:
            raise ValueError(f"unsupported pull-request action: {action!r}")
        number = value.get("number")
        if not isinstance(number, int) or isinstance(number, bool) or number < 1:
            raise ValueError("pull-request number must be positive")
        base = pull_request.get("base")
        head = pull_request.get("head")
        if not isinstance(base, dict) or not isinstance(head, dict):
            raise ValueError("pull-request base/head metadata is missing")
        base_repo = base.get("repo")
        head_repo = head.get("repo")
        if not isinstance(base_repo, dict) or not isinstance(head_repo, dict):
            raise ValueError("pull-request base/head repository metadata is missing")
        base_sha = str(base.get("sha", ""))
        head_sha = str(head.get("sha", ""))
        if not _OBJECT_ID.fullmatch(base_sha) or not _OBJECT_ID.fullmatch(head_sha):
            raise ValueError("pull-request base and head must be full object IDs")
        repository_name = str(repository.get("full_name", ""))
        if repository_name != str(base_repo.get("full_name", "")):
            raise ValueError("event repository does not match the pull-request base repository")
        return cls(
            action=action,
            number=number,
            repository=repository_name,
            base_repository=str(base_repo.get("full_name", "")),
            head_repository=str(head_repo.get("full_name", "")),
            base_sha=base_sha.lower(),
            head_sha=head_sha.lower(),
            base_ref=str(base.get("ref", "")),
            head_ref=str(head.get("ref", "")),
            title=str(pull_request.get("title", "")),
            body=str(pull_request.get("body") or ""),
        )


class GitHubApi:
    """Narrow API surface for checks and comments; source mutation is impossible."""

    def __init__(
        self,
        *,
        token: str,
        api_url: str = "https://api.github.com",
        actor_login: str = "",
        opener: Any | None = None,
    ) -> None:
        parsed = urllib.parse.urlparse(api_url)
        if parsed.scheme != "https" or not parsed.netloc:
            raise ValueError("GitHub API URL must use HTTPS")
        if not token:
            raise ValueError("GitHub token is required to publish review output")
        self.token = token
        self.api_url = api_url.rstrip("/")
        self._actor_login = actor_login.strip()
        self._opener = opener or urllib.request.urlopen

    def _request(self, method: str, path: str, payload: dict[str, Any] | None = None) -> Any:
        data = None if payload is None else json.dumps(payload, ensure_ascii=False).encode()
        request = urllib.request.Request(
            f"{self.api_url}{path}",
            data=data,
            method=method,
            headers={
                "Authorization": f"Bearer {self.token}",
                "Accept": "application/vnd.github+json",
                "Content-Type": "application/json",
                "X-GitHub-Api-Version": "2022-11-28",
                "User-Agent": "robotics-agentic-development-harness/ai-review",
            },
        )
        try:
            with self._opener(request, timeout=30) as response:
                raw = response.read()
        except urllib.error.HTTPError as error:
            detail = error.read().decode("utf-8", errors="replace")[:500]
            raise GitHubApiError(
                f"GitHub API {method} {path} returned {error.code}: {detail}",
                status_code=error.code,
            ) from error
        except urllib.error.URLError as error:
            raise GitHubApiError(f"GitHub API request failed: {error.reason}") from error
        if not raw:
            return None
        try:
            return json.loads(raw)
        except json.JSONDecodeError as error:
            raise GitHubApiError("GitHub API returned invalid JSON") from error

    def _pages(self, path: str) -> list[dict[str, Any]]:
        items: list[dict[str, Any]] = []
        separator = "&" if "?" in path else "?"
        page = 1
        while True:
            value = self._request("GET", f"{path}{separator}per_page=100&page={page}")
            if not isinstance(value, list):
                raise GitHubApiError("GitHub list endpoint returned a non-list response")
            objects = [item for item in value if isinstance(item, dict)]
            items.extend(objects)
            if len(value) < 100:
                return items
            page += 1

    def _authenticated_login(self) -> str:
        if self._actor_login:
            return self._actor_login
        value = self._request("GET", "/user")
        if not isinstance(value, dict) or not isinstance(value.get("login"), str):
            raise GitHubApiError("GitHub API did not identify the authenticated review actor")
        self._actor_login = str(value["login"])
        return self._actor_login

    def get_summary_comment(self, repository: str, pull_request: int) -> dict[str, Any] | None:
        comments = self._pages(f"/repos/{repository}/issues/{pull_request}/comments")
        actor = self._authenticated_login()
        return next(
            (
                comment
                for comment in comments
                if SUMMARY_MARKER in str(comment.get("body", ""))
                and isinstance(comment.get("user"), dict)
                and comment["user"].get("login") == actor
            ),
            None,
        )

    def get_pull_request(self, repository: str, pull_request: int) -> dict[str, Any]:
        value = self._request("GET", f"/repos/{repository}/pulls/{pull_request}")
        if not isinstance(value, dict):
            raise GitHubApiError("GitHub pull-request endpoint returned a non-object response")
        return value

    def get_associated_pull_requests(
        self, repository: str, commit_sha: str
    ) -> list[dict[str, Any]]:
        value = self._request("GET", f"/repos/{repository}/commits/{commit_sha}/pulls")
        if not isinstance(value, list):
            raise GitHubApiError("GitHub associated-pulls endpoint returned a non-list response")
        return [item for item in value if isinstance(item, dict)]

    def upsert_summary_comment(
        self, repository: str, pull_request: int, body: str
    ) -> dict[str, Any]:
        existing = self.get_summary_comment(repository, pull_request)
        if existing is not None:
            identifier = int(existing["id"])
            value = self._request(
                "PATCH", f"/repos/{repository}/issues/comments/{identifier}", {"body": body}
            )
        else:
            value = self._request(
                "POST",
                f"/repos/{repository}/issues/{pull_request}/comments",
                {"body": body},
            )
        if not isinstance(value, dict):
            raise GitHubApiError("GitHub summary-comment response was invalid")
        return value

    def create_check(self, repository: str, review: AIReview, summary: str) -> dict[str, Any]:
        value = self._request(
            "POST",
            f"/repos/{repository}/check-runs",
            {
                "name": "Agentic Review",
                "head_sha": review.head_sha,
                "status": "completed",
                "conclusion": review.check_conclusion.value,
                "external_id": f"codex-review:pr-{review.pull_request}:{review.head_sha}",
                "output": {
                    "title": f"Codex review: {review.result.value}",
                    "summary": summary[:65_535],
                },
            },
        )
        if not isinstance(value, dict):
            raise GitHubApiError("GitHub check response was invalid")
        return value

    def get_ci_status(self, repository: str, head_sha: str) -> dict[str, str]:
        value = self._request(
            "GET", f"/repos/{repository}/commits/{head_sha}/check-runs?per_page=100"
        )
        if not isinstance(value, dict) or not isinstance(value.get("check_runs"), list):
            return {}
        return {
            str(check.get("name", "")): str(
                check.get("conclusion") or check.get("status") or "unknown"
            )
            for check in value["check_runs"]
            if isinstance(check, dict)
            and str(check.get("name", "")) not in _SELF_REVIEW_CHECK_NAMES
        }

    def post_inline_findings(
        self,
        repository: str,
        pull_request: int,
        head_sha: str,
        findings: tuple[Finding, ...],
        changed_new_lines: dict[str, set[int]],
    ) -> int:
        existing = self._pages(f"/repos/{repository}/pulls/{pull_request}/comments")
        actor = self._authenticated_login()
        existing_ids = {
            match.group(1)
            for comment in existing
            if isinstance(comment.get("user"), dict)
            and comment["user"].get("login") == actor
            and (
                match := re.search(
                    r"<!-- robotics-harness-ai-finding:([0-9a-f]{20}) -->",
                    str(comment.get("body", "")),
                )
            )
        }
        comments: list[dict[str, Any]] = []
        for finding in findings:
            if (
                finding.severity.value not in _INLINE_SEVERITIES
                or finding.start_line is None
                or finding.identity in existing_ids
                or finding.state is FindingState.STILL_PRESENT
            ):
                continue
            line = finding.end_line or finding.start_line
            valid_lines = changed_new_lines.get(finding.path, set())
            if line not in valid_lines:
                continue
            comment: dict[str, Any] = {
                "path": finding.path,
                "line": line,
                "side": "RIGHT",
                "body": (
                    f"<!-- robotics-harness-ai-finding:{finding.identity} -->\n"
                    f"**[{finding.severity.value.title()} · "
                    f"{finding.confidence.value} confidence] "
                    f"{safe_markdown_text(finding.title)}**\n\n"
                    f"{safe_markdown_text(finding.explanation)}\n\n"
                    f"Evidence: {safe_markdown_text(finding.evidence)}\n\n"
                    f"Recommended action: "
                    f"{safe_markdown_text(finding.recommendation)}\n\n"
                    "_AI review is evidence, not approval._"
                ),
            }
            if (
                finding.end_line is not None
                and finding.end_line > finding.start_line
                and all(
                    value in valid_lines
                    for value in range(finding.start_line, finding.end_line + 1)
                )
            ):
                comment["start_line"] = finding.start_line
                comment["start_side"] = "RIGHT"
            comments.append(comment)
        if not comments:
            return 0
        self._request(
            "POST",
            f"/repos/{repository}/pulls/{pull_request}/reviews",
            {
                "commit_id": head_sha,
                "event": "COMMENT",
                "body": "Codex PR review inline evidence. Human review remains required.",
                "comments": comments,
            },
        )
        return len(comments)


def changed_new_lines(diff: DiffBundle) -> dict[str, set[int]]:
    """Identify added right-side lines so GitHub inline comments attach only when valid."""

    result: dict[str, set[int]] = {}
    by_path: dict[str, list[Any]] = {}
    for chunk in diff.chunks:
        if not chunk.binary_metadata_only and not chunk.submodule:
            by_path.setdefault(chunk.path, []).append(chunk)
    for path, chunks in by_path.items():
        patch = "".join(chunk.content for chunk in sorted(chunks, key=lambda item: item.index))
        lines: set[int] = set()
        current: int | None = None
        for line in patch.splitlines():
            match = _HUNK.match(line)
            if match:
                current = int(match.group(1))
                continue
            if current is None or line.startswith("\\"):
                continue
            if line.startswith("+") and not line.startswith("+++"):
                lines.add(current)
                current += 1
            elif line.startswith("-") and not line.startswith("---"):
                continue
            else:
                current += 1
        result[path] = lines
    return result


class AuditTrail:
    """Write workflow-local and shared-schema monitoring evidence."""

    def __init__(
        self,
        *,
        event: PullRequestEvent | None = None,
        event_store: EventStore | None = None,
        task_identifier: str = "",
    ) -> None:
        self.events: list[dict[str, Any]] = []
        self.event = event
        self.event_store = event_store
        resolved_task = task_identifier.strip()
        if resolved_task and (
            len(resolved_task) > 128 or any(ord(character) < 32 for character in resolved_task)
        ):
            raise ValueError("task identifier must be a single line of at most 128 characters")
        self.task_identifier = resolved_task

    def record(self, event_type: str, **metadata: Any) -> None:
        enriched = dict(metadata)
        if self.event is not None:
            # Canonical PR/head values are present on every event so digest
            # aggregation cannot accidentally split one immutable-head review.
            enriched["pull_request"] = self.event.number
            enriched["head_sha"] = self.event.head_sha
        self.events.append(
            {
                "timestamp": format_timestamp(datetime.now(UTC)),
                "event_type": event_type,
                "metadata": enriched,
            }
        )
        if self.event is not None and self.event_store is not None:
            path = enriched.get("path")
            files = (str(path),) if isinstance(path, str) and path else ()
            self.event_store.append(
                MonitoringEvent.create(
                    event_type=EventType(event_type),
                    session_id=f"ai-review-pr-{self.event.number}",
                    repository=self.event.repository,
                    branch=self.event.head_ref,
                    task_identifier=(self.task_identifier or f"pull-request-{self.event.number}"),
                    files_affected=files,
                    result=str(enriched.get("result", "")),
                    commit_or_pr_reference=(f"pr-{self.event.number}@{self.event.head_sha}"),
                    metadata=enriched,
                )
            )

    def write(self, path: Path) -> Path:
        return write_private_text(
            path, "\n".join(json.dumps(event, sort_keys=True) for event in self.events) + "\n"
        )


def _read_optional(path: Path | None, *, maximum: int = 1_000_000) -> str:
    if path is None or not path.is_file() or path.is_symlink():
        return ""
    if path.stat().st_size > maximum:
        return f"[evidence omitted: {path.name} exceeds {maximum} bytes]"
    return path.read_text(encoding="utf-8", errors="replace")


def _policy_excerpt(relative: str, text: str, *, maximum_characters: int) -> str:
    """Create an auditable bounded excerpt without pretending the source was complete."""

    digest = hashlib.sha256(text.encode()).hexdigest()
    marker = (
        f"[source={relative}; sha256={digest}; source_characters={len(text)}; "
        f"excerpt_limit={maximum_characters}]"
    )
    if len(text) <= maximum_characters:
        return f"{marker}\n{text}"
    candidates: list[str] = []
    seen: set[str] = set()
    for index, line in enumerate(text.splitlines()):
        stripped = line.strip()
        lowered = stripped.casefold()
        if not stripped:
            continue
        if (
            index < 12
            or stripped.startswith("#")
            or any(keyword in lowered for keyword in _POLICY_KEYWORDS)
        ) and stripped not in seen:
            seen.add(stripped)
            candidates.append(stripped)
    selected: list[str] = []
    size = 0
    for line in candidates:
        addition = len(line) + 1
        if size + addition > maximum_characters:
            break
        selected.append(line)
        size += addition
    return (
        f"{marker}\n"
        "[deterministic excerpt: the complete source is identified by SHA-256; "
        "omitted text was not sent to the model]\n" + "\n".join(selected)
    )


def _trusted_policy(policy_root: Path) -> str:
    sections: list[str] = []
    for relative in (
        "AGENTS.md",
        "README.md",
        "docs/governance/ENFORCEMENT_MATRIX.md",
        "docs/governance/CODE_OWNERSHIP.md",
        "docs/operations/COMPETITION_HOTFIX_RUNBOOK.md",
    ):
        path = policy_root / relative
        text = _read_optional(path, maximum=250_000)
        if text:
            sections.append(
                f"## {relative}\n{_policy_excerpt(relative, text, maximum_characters=1_200)}"
            )
    return "\n".join(sections)


def _target_policy(repository: Path, base_sha: str) -> str:
    import subprocess

    sections: list[str] = []
    for relative in ("AGENTS.md", "README.md", "build.gradle", "settings.gradle"):
        completed = subprocess.run(
            ("git", "show", f"{base_sha}:{relative}"),
            cwd=repository,
            check=False,
            capture_output=True,
        )
        if completed.returncode == 0:
            text = completed.stdout[:250_000].decode("utf-8", errors="replace")
            sections.append(
                f"## {relative} at base SHA\n"
                f"{_policy_excerpt(relative, text, maximum_characters=1_250)}"
            )
    return "\n".join(sections)


def _load_json(path: Path | None) -> dict[str, Any]:
    text = _read_optional(path)
    if not text:
        return {}
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        return {"status": "invalid JSON evidence"}
    return value if isinstance(value, dict) else {"value": value}


def _safe_harness_evidence(value: dict[str, Any]) -> dict[str, Any]:
    """Reduce caller-supplied evidence to comment-safe status and digest fields."""

    safe: dict[str, Any] = {}
    for key in ("status", "result", "conclusion", "passed", "failed", "head_sha", "base_sha"):
        item = value.get(key)
        if isinstance(item, (str, bool, int, float)):
            safe[key] = item
    for key in ("sha256", "evidence_sha256", "commit"):
        item = value.get(key)
        if isinstance(item, str) and re.fullmatch(r"[0-9a-fA-F]{7,64}", item.strip()):
            safe[key] = item.strip()
    commands = value.get("commands", value.get("test_commands"))
    if isinstance(commands, list):
        safe["command_count"] = len(commands)
    tests = value.get("tests")
    if isinstance(tests, list):
        safe["test_count"] = len(tests)
    return safe or {"status": "supplied_without_publishable_fields"}


def _risk_assessment(diff: DiffBundle, policy_root: Path) -> RiskAssessment:
    chunks_by_path: dict[str, list[Any]] = {}
    for chunk in diff.chunks:
        chunks_by_path.setdefault(chunk.path, []).append(chunk)
    changes = tuple(
        GitChangedFile(
            path=file.path,
            old_path=file.old_path,
            patch="".join(
                chunk.content
                for chunk in sorted(chunks_by_path.get(file.path, []), key=lambda item: item.index)
            ),
        )
        for file in diff.files
    )
    return RiskClassifier(policy_root / "config/risk_rules.yaml").classify(changes)


def _reviewer_group(assessment: RiskAssessment) -> str:
    rule_ids = {match.rule_id for match in assessment.matches}
    if "robot-behaviour" in rule_ids:
        return "eligible safety reviewer under config/review_policy.yaml"
    if assessment.risk.label == "critical":
        return "two distinct current Mentors"
    if assessment.risk.label == "medium":
        return "eligible robot-code reviewer under config/review_policy.yaml"
    return "eligible human reviewer under config/review_policy.yaml"


def _provider(
    event: PullRequestEvent,
    *,
    recorded_responses: Path | None = None,
    target_repository: Path | None = None,
) -> ReviewProvider:
    if recorded_responses is not None:
        source = recorded_responses.expanduser().resolve()
        if target_repository is not None and source.is_relative_to(target_repository.resolve()):
            raise ProviderError(
                "recorded provider responses must be stored outside the target repository"
            )
        return RecordedReviewProvider.from_path(source)
    del event
    return _UnavailableProvider(
        "live GitHub Models and generic HTTP AI review providers are retired; "
        "use the trusted Codex workflow path or explicit recorded responses"
    )


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run trusted GitHub Codex PR review")
    parser.add_argument("--event", type=Path, required=True)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--policy-root", type=Path, required=True)
    parser.add_argument("--artifacts", type=Path, required=True)
    parser.add_argument("--task-specification", type=Path)
    parser.add_argument("--confirmed-plan", type=Path)
    parser.add_argument("--test-evidence", type=Path)
    parser.add_argument("--learning-evidence", type=Path)
    parser.add_argument("--monitoring-store", type=Path)
    parser.add_argument(
        "--task-id",
        default="",
        help="optional harness task identifier for monitoring-event correlation",
    )
    parser.add_argument(
        "--provider-responses",
        type=Path,
        help=(
            "explicit recorded ProviderReview response file outside the target repository; "
            "never discovered implicitly"
        ),
    )
    return parser.parse_args(argv)


def run_workflow(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    event = PullRequestEvent.from_dict(json.loads(args.event.read_text(encoding="utf-8")))
    repository = args.repo.resolve()
    artifacts = args.artifacts.resolve()
    audit = AuditTrail(
        event=event,
        event_store=(
            None if args.monitoring_store is None else EventStore(args.monitoring_store.resolve())
        ),
        task_identifier=args.task_id,
    )
    audit.record("pull_request_detected", pull_request=event.number, action=event.action)
    audit.record("ai_review_started", pull_request=event.number)
    audit.record("ai_base_sha_recorded", sha=event.base_sha, ref=event.base_ref)
    audit.record("ai_head_sha_recorded", sha=event.head_sha, ref=event.head_ref)

    token = os.environ.get("GITHUB_TOKEN", "")
    api: GitHubApi | None = None
    previous: AIReview | None = None
    ci_status: dict[str, str] = {}
    publication_error = ""
    if token:
        api = GitHubApi(
            token=token,
            api_url=os.environ.get("GITHUB_API_URL", "https://api.github.com"),
            actor_login=os.environ.get("AI_REVIEW_COMMENT_ACTOR", ""),
        )
        try:
            existing = api.get_summary_comment(event.repository, event.number)
            if existing is not None:
                previous = decode_review_state(str(existing.get("body", "")))
            ci_status = api.get_ci_status(event.repository, event.head_sha)
        except GitHubApiError as error:
            publication_error = str(error)

    try:
        diff = collect_full_diff(
            repository,
            base_sha=event.base_sha,
            head_sha=event.head_sha,
            maximum_chunk_characters=6_000,
        )
        audit.record(
            "ai_full_diff_collected",
            sha256=diff.full_diff_sha256,
            files=len(diff.files),
            bytes=diff.full_diff_bytes,
        )
    except DiffCollectionError as error:
        audit.record("ai_diff_collection_incomplete", reason=str(error))
        print(f"AI review incomplete: full diff unavailable: {error}", file=sys.stderr)
        diff = DiffBundle(
            repository=str(repository),
            base_sha=event.base_sha,
            head_sha=event.head_sha,
            files=(),
            chunks=(),
            full_diff_sha256=("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"),
            full_diff_bytes=0,
            additions=0,
            deletions=0,
            commit_messages=(),
            complete=False,
            incomplete_reason=f"full diff unavailable: {error}",
        )

    policy_root = args.policy_root.resolve()
    assessment = _risk_assessment(diff, policy_root)
    context = ReviewContext(
        repository_policy=_trusted_policy(policy_root),
        target_policy=_target_policy(repository, event.base_sha),
        task_specification=_read_optional(args.task_specification)
        or "Harness task specification was not supplied.",
        confirmed_plan=_read_optional(args.confirmed_plan)
        or "Confirmed implementation plan was not supplied.",
        risk_classification=assessment.risk.label,
        protected_path_result=json.dumps(assessment.to_dict(), sort_keys=True),
        learning_review_status=(
            str(_load_json(args.learning_evidence).get("status", "not supplied"))
        ),
        test_evidence={
            "github_checks_at_review_start": ci_status,
            "harness": _safe_harness_evidence(_load_json(args.test_evidence)),
        },
    )
    audit.record("ai_review_context_collected", sources="trusted base and harness evidence")
    try:
        provider = _provider(
            event,
            recorded_responses=args.provider_responses,
            target_repository=repository,
        )
    except ProviderError as error:
        provider = _UnavailableProvider(str(error))
    # One bounded chunk per inference keeps small Phase 0 reviews inside the
    # configured input envelope. Every chunk is still reviewed or the result is
    # explicitly incomplete; no tail of a large diff is silently truncated.
    service = AIReviewService(provider, chunks_per_request=1)
    if previous is not None:
        audit.record("ai_review_rerun", previous_head_sha=previous.head_sha)
    review = service.review(
        PullRequestMetadata(
            number=event.number,
            repository=event.repository,
            title=event.title,
            body=event.body,
        ),
        diff,
        context,
        previous=previous,
        required_reviewer_group=_reviewer_group(assessment),
    )
    audit.record("ai_review_completed", result=review.result.value)
    for finding in review.findings:
        audit.record(
            "ai_blocking_finding_created" if finding.blocking else "ai_finding_created",
            finding_id=finding.identity,
            severity=finding.severity.value,
            state=finding.state.value,
            path=finding.path,
        )
        if finding.category.value == "robotics_safety" and finding.blocking:
            audit.record("ai_safety_escalation_created", finding_id=finding.identity)
        if finding.state is FindingState.STILL_PRESENT:
            audit.record("ai_finding_still_present", finding_id=finding.identity)
    for finding in review.resolved_findings:
        audit.record("ai_finding_resolved", finding_id=finding.identity)

    json_path, markdown_path = write_review_artifacts(artifacts, review, diff)
    summary = render_summary_comment(review, diff)
    if api is not None:
        try:
            api.upsert_summary_comment(event.repository, event.number, summary)
            audit.record("ai_summary_posted", head_sha=event.head_sha)
            inline_count = api.post_inline_findings(
                event.repository,
                event.number,
                event.head_sha,
                review.findings,
                changed_new_lines(diff),
            )
            for _ in range(inline_count):
                audit.record("ai_inline_comment_posted", head_sha=event.head_sha)
            api.create_check(event.repository, review, summary)
            audit.record(
                "ai_check_passed"
                if review.check_conclusion.value == "success"
                else "ai_check_failed",
                head_sha=event.head_sha,
            )
        except GitHubApiError as error:
            publication_error = str(error)
    else:
        publication_error = "GITHUB_TOKEN is unavailable; review output could not be published"
    if publication_error:
        audit.record("ai_check_failed", reason=publication_error)
        print(publication_error, file=sys.stderr)
    audit.write(artifacts / f"pr-{event.number}-{event.head_sha}-audit.jsonl")
    print(json_path)
    print(markdown_path)
    if publication_error or review.result not in {
        ReviewResult.PASS,
        ReviewResult.PASS_WITH_SUGGESTIONS,
    }:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(run_workflow())
