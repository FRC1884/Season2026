"""Execution-plane adapters for Codex-agent review orchestration."""

from __future__ import annotations

import base64
import json
import subprocess
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from harness.agent_review.models import (
    STATE_MARKER_PREFIX,
    PullRequestContext,
    PullRequestRef,
    ReviewContext,
    ReviewCycle,
    ReviewFinding,
    ReviewResponse,
    ReviewState,
)
from harness.agent_review.service import (
    build_review_context,
    create_auto_review_request,
    create_review_cycle_with_findings,
    mark_review_stale,
    parse_review_request,
    record_review_response,
    render_review_markdown,
    validate_review_cycle,
)
from harness.ai_review.diff import collect_full_diff
from harness.monitoring import EventType
from harness.private_io import write_private_text

PLATFORM_REVIEW_REQUEST_PREFIX = "<!-- robotics-harness-codex-platform-review-request:"
PLATFORM_REVIEW_REQUEST_SUFFIX = " -->"
PLATFORM_REQUEST_ACK_PREFIX = "<!-- robotics-harness-platform-request-ack:"
PLATFORM_REQUEST_ACK_SUFFIX = " -->"
REVIEW_SUMMARY_MARKER_PREFIX = "<!-- robotics-harness-agent-review-summary:"
REVIEW_SUMMARY_MARKER_SUFFIX = " -->"
GITHUB_REVIEW_COMMENT_LIMIT = 60_000


def _execution() -> Any:
    from harness.execution_plane import runtime

    return runtime


def _require_external(target_repo: Path | str, runtime_root: Path | str) -> tuple[Path, Path]:
    repository, root = _execution()._require_external_runtime_root(target_repo, runtime_root)
    return Path(repository), Path(root)


def _review_root(runtime_root: Path, repository: str, pull_request: int) -> Path:
    return runtime_root / "harness-evidence" / repository.replace("/", "--") / f"pr-{pull_request}"


def _write_json(path: Path, value: object) -> Path:
    return write_private_text(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def _copy_text(source: Path, destination: Path) -> Path:
    return write_private_text(destination, source.read_text(encoding="utf-8"))


def _event(
    *,
    root: Path,
    repo: Path,
    event_type: EventType,
    repository: str,
    pull_request: int,
    base_sha: str,
    head_sha: str,
    task_id: str,
    session_id: str,
    result: str,
    branch: str = "",
    requested_by_session_id: str = "",
    reviewer_session_id: str = "",
    extra: dict[str, Any] | None = None,
    files: tuple[str, ...] = (),
) -> None:
    runtime = _execution()
    metadata = runtime._review_event_metadata(
        target_repo=repo,
        repository=repository,
        pull_request=pull_request,
        base_sha=base_sha,
        head_sha=head_sha,
        requested_by_session_id=requested_by_session_id,
        reviewer_session_id=reviewer_session_id,
        extra=extra,
    )
    runtime._append_review_event(
        event_store=runtime._runtime_event_store(root),
        event_type=event_type,
        session_id=session_id,
        target_repo=repo,
        repository=repository,
        branch=branch,
        task_id=task_id,
        result=result,
        pull_request=pull_request,
        head_sha=head_sha,
        metadata=metadata,
        files_affected=files,
    )


def _fetch_pr_objects(repo: Path, base_sha: str, head_sha: str) -> None:
    completed = subprocess.run(
        ("git", "fetch", "--no-tags", "origin", base_sha, head_sha),
        cwd=repo,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise ValueError(completed.stderr.strip() or "failed to fetch pull-request objects")


def _live_context(repo: Path, repository: str, pull_request: int) -> PullRequestContext:
    runtime = _execution()
    payload = runtime._pull_request_payload(
        repository=repository, pull_request=pull_request, cwd=repo
    )
    state = runtime._pull_request_state(repository=repository, pull_request=pull_request, cwd=repo)
    _fetch_pr_objects(repo, str(state["base_sha"]), str(state["head_sha"]))
    diff = collect_full_diff(
        repo,
        base_sha=str(state["base_sha"]),
        head_sha=str(state["head_sha"]),
        maximum_chunk_characters=12_000,
    )
    diff_text = "\n".join(chunk.content for chunk in diff.chunks).strip()
    if not diff.complete or not diff_text:
        raise ValueError(diff.incomplete_reason or "pull-request diff is incomplete")
    created_at = str(payload.get("created_at", ""))
    opened_at = datetime.fromisoformat(created_at.replace("Z", "+00:00")).astimezone(UTC)
    return PullRequestContext(
        pull_request=PullRequestRef(
            repository=repository,
            number=pull_request,
            base_sha=str(state["base_sha"]),
            head_sha=str(state["head_sha"]),
        ),
        title=str(state["title"]),
        body=str(state["body"]),
        author=str(state["author"]),
        files=tuple(file.path for file in diff.files),
        diff_text=diff_text,
        opened_at=opened_at,
    )


def _request_paths(root: Path, context: ReviewContext) -> tuple[Path, Path]:
    directory = _review_root(root, context.repository, context.pull_request_ref.number) / "requests"
    stem = f"review-request-{context.head_sha}"
    return directory / f"{stem}.json", directory / f"{stem}.md"


def _request_markdown(context: ReviewContext) -> str:
    pr = context.pull_request
    return "\n".join(
        (
            f"# Automated Reviewer Request — PR #{pr.pull_request.number}",
            "",
            "## Objective",
            "",
            context.objective,
            "",
            "## Immutable Metadata",
            "",
            f"- Repository: `{pr.pull_request.repository}`",
            f"- Base SHA: `{pr.pull_request.base_sha}`",
            f"- Head SHA: `{pr.pull_request.head_sha}`",
            f"- Request digest: `{context.request_digest_sha256}`",
            f"- Requested by session: `{context.requested_by_session_id}`",
            "",
            "## Changed Files",
            "",
            *[f"- `{path}`" for path in pr.files],
            "",
            "## Trusted Review Policy",
            "",
            context.trusted_policy or "No trusted policy snapshot was available.",
            "",
            "## CI / Test State",
            "",
            "```json",
            json.dumps(context.ci_status, indent=2, sort_keys=True),
            "```",
            "",
            "## Prior Review State",
            "",
            "```json",
            json.dumps(context.prior_review_state, indent=2, sort_keys=True),
            "```",
            "",
            "## Review Context",
            "",
            "```diff",
            pr.diff_text,
            "```",
            "",
            "Implementation-agent conclusions are intentionally absent.",
            "",
        )
    )


def runtime_prepare_review(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    repository: str,
    pull_request: int,
    task_id: str,
    requested_by_session_id: str,
    automatic: bool = False,
    **_: Any,
) -> dict[str, Any]:
    repo, root = _require_external(target_repo, runtime_root)
    pr_context = _live_context(repo, repository, pull_request)
    request = (
        create_auto_review_request(
            repository=repository,
            pull_request_number=pull_request,
            head_sha=pr_context.pull_request.head_sha,
            requested_by_session_id=requested_by_session_id,
        )
        if automatic
        else parse_review_request(f"review {repository} PR #{pull_request}")
    )
    runtime = _execution()
    check_payload = runtime._run_gh_api(
        path=(
            f"repos/{repository}/commits/{pr_context.pull_request.head_sha}/check-runs?per_page=100"
        ),
        cwd=repo,
    )
    ci_status = {
        str(item.get("name", "")): str(item.get("conclusion") or item.get("status") or "unknown")
        for item in (check_payload.get("check_runs", []) if isinstance(check_payload, dict) else [])
        if isinstance(item, dict) and str(item.get("name", "")) != "Agentic Review"
    }
    previous_path = _cycle_directory(root, repository, pull_request) / "latest-review.json"
    prior_review_state = (
        json.loads(previous_path.read_text(encoding="utf-8")) if previous_path.is_file() else {}
    )
    trusted_policy = "\n\n".join(
        (
            runtime._trusted_policy(runtime._repo_root()),
            runtime._target_policy(repo, pr_context.pull_request.base_sha),
        )
    ).strip()
    context = build_review_context(
        request=request,
        pull_request=pr_context,
        requested_by_session_id=requested_by_session_id,
        trusted_policy=trusted_policy,
        ci_status=ci_status,
        prior_review_state=prior_review_state,
    )
    json_path, markdown_path = _request_paths(root, context)
    _write_json(json_path, context.to_dict())
    write_private_text(markdown_path, _request_markdown(context))
    common: dict[str, Any] = dict(
        root=root,
        repo=repo,
        repository=repository,
        pull_request=pull_request,
        base_sha=context.pull_request_ref.base_sha,
        head_sha=context.head_sha,
        task_id=task_id,
        session_id=f"review-request-{context.request_digest_sha256[:12]}",
        requested_by_session_id=requested_by_session_id,
        branch="",
    )
    _event(
        **common,
        event_type=EventType.REVIEW_AGENT_REQUESTED,
        result="requested",
        extra={"request_digest_sha256": context.request_digest_sha256},
    )
    _event(
        **common,
        event_type=EventType.REVIEW_CONTEXT_LOADED,
        result="loaded",
        extra={"request_path": str(json_path)},
    )
    _event(
        **common,
        event_type=EventType.REVIEW_DIFF_LOADED,
        result="loaded",
        extra={"diff_sha256": pr_context.diff_sha256, "file_count": len(pr_context.files)},
        files=pr_context.files,
    )
    return {
        "status": "prepared",
        "repository": repository,
        "pull_request": pull_request,
        "review_request": f"review {repository} PR #{pull_request}",
        "base_sha": context.pull_request_ref.base_sha,
        "head_sha": context.head_sha,
        "request_digest_sha256": context.request_digest_sha256,
        "objective_bundle_path": str(markdown_path),
        "review_context_path": str(json_path),
        "handoff_path": str(markdown_path),
        "machine_handoff": {
            "kind": "codex_subagent_review_request",
            "command": "spawn_agent",
            "request": f"review {repository} PR #{pull_request}",
            "agent": "automated-reviewer",
            "fresh_context": True,
            "reviewer_role": "Automated Reviewer",
            "objective_bundle_path": str(markdown_path),
            "review_context_path": str(json_path),
            "requested_by_session_id": requested_by_session_id,
            "head_sha": context.head_sha,
        },
    }


def _latest_request(root: Path, repository: str, pull_request: int, head_sha: str) -> Path:
    path = (
        _review_root(root, repository, pull_request)
        / "requests"
        / f"review-request-{head_sha}.json"
    )
    if not path.is_file():
        raise ValueError("no prepared objective review request exists for this head")
    return path


def _finding(value: object, index: int) -> ReviewFinding:
    if not isinstance(value, dict):
        raise ValueError("review findings must be objects")
    return ReviewFinding(
        finding_id=str(
            value.get("finding_id") or value.get("stable_identifier") or f"AR-{index:03d}"
        ),
        severity=str(value.get("severity", "medium")),
        category=str(value.get("category", "correctness")),
        title=str(value.get("title", "Review finding")),
        summary=str(
            value.get("summary") or value.get("explanation") or "Finding requires attention."
        ),
        recommendation=str(
            value.get("recommendation") or "Address this finding before merge readiness."
        ),
        blocking=bool(value.get("blocking", False)),
        path=str(value.get("path", "")),
        start_line=(int(value["start_line"]) if value.get("start_line") is not None else None),
        end_line=int(value["end_line"]) if value.get("end_line") is not None else None,
    )


def _validate_reviewer_result(
    result: str, findings: tuple[ReviewFinding, ...], raw_findings: list[object]
) -> None:
    blocking = any(finding.blocking for finding in findings)
    if result == "pass" and findings:
        raise ValueError("pass reviewer output must not contain findings")
    if result == "pass_with_suggestions" and (not findings or blocking):
        raise ValueError(
            "pass_with_suggestions requires at least one advisory finding and no blocking findings"
        )
    if result == "changes_requested" and not blocking:
        raise ValueError("changes_requested requires at least one blocking finding")
    if result == "escalate_to_safety_reviewer":
        safety_blocking = any(
            finding.blocking
            and (
                "safety" in finding.category.casefold()
                or (isinstance(raw, dict) and bool(raw.get("safety_review", False)))
            )
            for finding, raw in zip(findings, raw_findings, strict=True)
        )
        if not safety_blocking:
            raise ValueError(
                "escalate_to_safety_reviewer requires a blocking robotics-safety finding"
            )


def _cycle_directory(root: Path, repository: str, pull_request: int) -> Path:
    return _review_root(root, repository, pull_request) / "reviews"


def _next_cycle_number(directory: Path) -> int:
    return len(tuple(directory.glob("review-cycle-*.json"))) + 1


def _write_cycle(
    root: Path,
    cycle: ReviewCycle,
    *,
    provider_payload: dict[str, Any] | None = None,
    provider_markdown: str = "",
) -> dict[str, str | int]:
    directory = _cycle_directory(root, cycle.repository, cycle.pull_request)
    number = _next_cycle_number(directory)
    json_path = _write_json(directory / f"review-cycle-{number:03d}.json", cycle.to_dict())
    canonical_markdown = render_review_markdown(cycle)
    complete_markdown = canonical_markdown
    if provider_markdown.strip():
        complete_markdown += (
            "\n\n## Automated Reviewer Source Report\n\n" + provider_markdown.strip() + "\n"
        )
    markdown_path = write_private_text(
        directory / f"review-cycle-{number:03d}.md", complete_markdown
    )
    provider_json = ""
    if provider_payload is not None:
        provider_json = str(
            _write_json(directory / f"review-cycle-{number:03d}-provider.json", provider_payload)
        )
    latest_json = _copy_text(json_path, directory / "latest-review.json")
    latest_markdown = _copy_text(markdown_path, directory / "latest-review.md")
    state_path = _copy_text(json_path, directory / "review-state.json")
    return {
        "cycle": number,
        "json": str(json_path),
        "markdown": str(markdown_path),
        "latest_json": str(latest_json),
        "latest_markdown": str(latest_markdown),
        "state": str(state_path),
        "provider_json": provider_json,
    }


def runtime_record_review(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    repository: str,
    pull_request: int,
    bundle: Path | str,
    requested_by_session_id: str = "",
    reviewer_session_id: str = "",
) -> dict[str, Any]:
    repo, root = _require_external(target_repo, runtime_root)
    source = Path(bundle).expanduser().resolve()
    if source.is_relative_to(repo):
        raise ValueError("reviewer output must be outside the target repository")
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("reviewer output must be a JSON object")
    from harness.ai_review.codex import _read_schema, _validate_schema

    schema_path = (
        _execution()._repo_root()
        / ".github"
        / "codex"
        / "schemas"
        / "provider-review-output-v1.json"
    )
    _validate_schema(_read_schema(schema_path), payload)
    head_sha = str(payload.get("head_sha", ""))
    context = ReviewContext.from_dict(
        json.loads(_latest_request(root, repository, pull_request, head_sha).read_text())
    )
    if str(payload.get("repository", "")) != repository:
        raise ValueError("reviewer output repository does not match the request")
    if int(payload.get("pull_request", 0)) != pull_request:
        raise ValueError("reviewer output pull request does not match the request")
    if str(payload.get("base_sha", "")) != context.pull_request_ref.base_sha:
        raise ValueError("reviewer output base_sha does not match the request")
    if str(payload.get("request_digest_sha256", "")) != context.request_digest_sha256:
        raise ValueError("reviewer output request digest does not match the objective bundle")
    reviewer_role = str(payload.get("reviewer_role", ""))
    session = reviewer_session_id or str(payload.get("reviewer_session_id", ""))
    requester = requested_by_session_id or context.requested_by_session_id
    if requester != context.requested_by_session_id:
        raise ValueError("requested_by_session_id does not match the prepared request")
    started = any(
        item.event_type is EventType.REVIEW_AGENT_STARTED
        and item.session_id == f"review-agent-{session}"
        and str(item.metadata.get("head_sha", "")) == head_sha
        for item in _execution()._runtime_event_store(root).read_all()
    )
    if not started:
        raise ValueError("review output is missing a matching reviewer-agent start event")
    raw_findings = payload.get("findings", [])
    if not isinstance(raw_findings, list):
        raise ValueError("reviewer findings must be a list")
    findings = tuple(_finding(item, index) for index, item in enumerate(raw_findings, 1))
    result = str(payload.get("result", "")).casefold()
    _validate_reviewer_result(result, findings, raw_findings)
    previous_path = _cycle_directory(root, repository, pull_request) / "latest-review.json"
    previous_cycle = _load_cycle(previous_path) if previous_path.is_file() else None
    cycle = create_review_cycle_with_findings(
        context=context,
        reviewer_role=reviewer_role,
        reviewer_session_id=session,
        findings=findings,
        previous_cycle=previous_cycle,
    )
    if result in {"review_incomplete", "failed"}:
        cycle = cycle.with_state(ReviewState.FAILED)
    elif result == "pass" and not findings:
        cycle = cycle.with_state(ReviewState.PASS)
    validate_review_cycle(cycle, current_head_sha=head_sha)
    paths = _write_cycle(
        root,
        cycle,
        provider_payload=payload,
        provider_markdown=str(payload.get("markdown_report", "")),
    )
    session_id = f"review-agent-{session}"
    common: dict[str, Any] = dict(
        root=root,
        repo=repo,
        repository=repository,
        pull_request=pull_request,
        base_sha=cycle.base_sha,
        head_sha=head_sha,
        task_id=task_id,
        session_id=session_id,
        requested_by_session_id=requester,
        reviewer_session_id=session,
        files=context.pull_request.files,
    )
    _event(
        **common,
        event_type=EventType.REVIEW_COMPLETED,
        result=cycle.state.value,
        extra={"integrity_hash": cycle.integrity_hash, "cycle": paths["cycle"]},
    )
    for finding in cycle.findings:
        _event(
            **common,
            event_type=EventType.REVIEW_FINDING_CREATED,
            result=finding.severity,
            extra={
                "finding_id": finding.finding_id,
                "category": finding.category,
                "blocking": finding.blocking,
            },
        )
    _event(
        **common,
        event_type=EventType.REVIEW_STATE_RECORDED,
        result=cycle.state.value,
        extra={"integrity_hash": cycle.integrity_hash, "state_path": paths["state"]},
    )
    if int(paths["cycle"]) > 1:
        _event(
            **common,
            event_type=EventType.REVIEW_RERUN_COMPLETED,
            result=cycle.state.value,
            extra={"cycle": paths["cycle"]},
        )
    return {
        "status": "recorded",
        "repository": repository,
        "pull_request": pull_request,
        "head_sha": head_sha,
        "review_state": cycle.state.value,
        "finding_count": len(cycle.findings),
        "integrity_hash": cycle.integrity_hash,
        "artifacts": paths,
    }


def _load_cycle(path: Path | str) -> ReviewCycle:
    return ReviewCycle.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def _latest_cycle(root: Path, repository: str, pull_request: int) -> ReviewCycle:
    path = _cycle_directory(root, repository, pull_request) / "latest-review.json"
    if not path.is_file():
        raise ValueError("no recorded review cycle exists")
    return _load_cycle(path)


def _github_body(cycle: ReviewCycle, *, markdown_source: str = "") -> str:
    source = markdown_source.strip() or render_review_markdown(cycle)
    source = "\n".join(
        line for line in source.splitlines() if not line.startswith(STATE_MARKER_PREFIX)
    ).strip()
    footer = (
        "\n\n_Logical actor: Automated Reviewer. This hosted report is bounded; the full "
        "validated Markdown/JSON remains in the external Harness evidence directory. "
        "Local evidence is not publisher authentication. The required check independently "
        "requires a current-head review from the configured Codex GitHub identity. "
        "Human review remains required._\n"
    )
    marker = _review_summary_marker(repository=cycle.repository, pull_request=cycle.pull_request)
    prefix = marker + "\n"
    if len(prefix) + len(source) + len(footer) > GITHUB_REVIEW_COMMENT_LIMIT:
        notice = "\n\n> Hosted report truncated at the GitHub comment boundary."
        source = source[
            : GITHUB_REVIEW_COMMENT_LIMIT - len(prefix) - len(footer) - len(notice)
        ].rstrip()
        source += notice
    return prefix + source + footer


def _review_summary_marker(*, repository: str, pull_request: int) -> str:
    payload = json.dumps(
        {"repository": repository, "pull_request": pull_request},
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    encoded = base64.urlsafe_b64encode(payload).decode("ascii").rstrip("=")
    return f"{REVIEW_SUMMARY_MARKER_PREFIX}{encoded}{REVIEW_SUMMARY_MARKER_SUFFIX}"


def _decode_review_summary_marker(text: str) -> dict[str, object] | None:
    start = text.find(REVIEW_SUMMARY_MARKER_PREFIX)
    if start < 0:
        return None
    start += len(REVIEW_SUMMARY_MARKER_PREFIX)
    end = text.find(REVIEW_SUMMARY_MARKER_SUFFIX, start)
    if end < 0:
        return None
    encoded = text[start:end].strip()
    try:
        padded = encoded + "=" * (-len(encoded) % 4)
        value = json.loads(base64.urlsafe_b64decode(padded).decode("utf-8"))
    except (ValueError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _matches_review_summary(value: object, *, repository: str, pull_request: int) -> bool:
    return (
        isinstance(value, dict)
        and str(value.get("repository", "")).casefold() == repository.casefold()
        and value.get("pull_request") == pull_request
    )


def _platform_request_marker(*, repository: str, pull_request: int, head_sha: str) -> str:
    payload = json.dumps(
        {
            "repository": repository,
            "pull_request": pull_request,
            "head_sha": head_sha,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    encoded = base64.urlsafe_b64encode(payload).decode("ascii").rstrip("=")
    return f"{PLATFORM_REVIEW_REQUEST_PREFIX}{encoded}{PLATFORM_REVIEW_REQUEST_SUFFIX}"


def _decode_platform_request_marker(text: str) -> dict[str, object] | None:
    start = text.find(PLATFORM_REVIEW_REQUEST_PREFIX)
    if start < 0:
        return None
    start += len(PLATFORM_REVIEW_REQUEST_PREFIX)
    end = text.find(PLATFORM_REVIEW_REQUEST_SUFFIX, start)
    if end < 0:
        return None
    encoded = text[start:end].strip()
    try:
        padded = encoded + "=" * (-len(encoded) % 4)
        value = json.loads(base64.urlsafe_b64decode(padded).decode("utf-8"))
    except (ValueError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _matches_platform_request(
    value: object, *, repository: str, pull_request: int, head_sha: str
) -> bool:
    if not isinstance(value, dict):
        return False
    return (
        str(value.get("repository", "")).casefold() == repository.casefold()
        and value.get("pull_request") == pull_request
        and str(value.get("head_sha", "")).casefold() == head_sha.casefold()
    )


def _platform_request_ack_marker(
    *, repository: str, pull_request: int, head_sha: str, request_comment_id: int
) -> str:
    payload = json.dumps(
        {
            "repository": repository,
            "pull_request": pull_request,
            "head_sha": head_sha,
            "request_comment_id": request_comment_id,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    encoded = base64.urlsafe_b64encode(payload).decode("ascii").rstrip("=")
    return f"{PLATFORM_REQUEST_ACK_PREFIX}{encoded}{PLATFORM_REQUEST_ACK_SUFFIX}"


def _decode_platform_request_ack(text: str) -> dict[str, object] | None:
    start = text.find(PLATFORM_REQUEST_ACK_PREFIX)
    if start < 0:
        return None
    start += len(PLATFORM_REQUEST_ACK_PREFIX)
    end = text.find(PLATFORM_REQUEST_ACK_SUFFIX, start)
    if end < 0:
        return None
    encoded = text[start:end].strip()
    try:
        padded = encoded + "=" * (-len(encoded) % 4)
        value = json.loads(base64.urlsafe_b64decode(padded).decode("utf-8"))
    except (ValueError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _matches_platform_request_ack(
    value: object, *, repository: str, pull_request: int, head_sha: str
) -> bool:
    return (
        isinstance(value, dict)
        and str(value.get("repository", "")).casefold() == repository.casefold()
        and value.get("pull_request") == pull_request
        and str(value.get("head_sha", "")).casefold() == head_sha.casefold()
        and isinstance(value.get("request_comment_id"), int)
        and int(value["request_comment_id"]) > 0
    )


def _paged_values(runtime: Any, *, path: str, cwd: Path) -> list[dict[str, Any]]:
    values: list[dict[str, Any]] = []
    page = 1
    while True:
        page_path = path if page == 1 else f"{path}&page={page}"
        fetched = runtime._run_gh_api(path=page_path, cwd=cwd)
        if not isinstance(fetched, list):
            return values
        batch = [item for item in fetched if isinstance(item, dict)]
        values.extend(batch)
        if len(fetched) < 100:
            return values
        page += 1


def _github_identity_matches(
    value: object,
    *,
    login: str,
    account_id: int,
    account_type: str,
) -> bool:
    if not isinstance(value, dict):
        return False
    try:
        observed_id = int(value.get("id", 0))
    except (TypeError, ValueError):
        return False
    return (
        str(value.get("login", "")).casefold() == login.casefold()
        and observed_id == account_id
        and str(value.get("type", "")).casefold() == account_type.casefold()
    )


def _github_datetime(value: object) -> datetime | None:
    text = str(value or "").strip()
    if not text:
        return None
    try:
        return datetime.fromisoformat(text.replace("Z", "+00:00")).astimezone(UTC)
    except ValueError:
        return None


def _latest_platform_request_ack(
    runtime: Any,
    *,
    repo: Path,
    repository: str,
    pull_request: int,
    head_sha: str,
    request_publisher_login: str,
    request_publisher_id: int,
    request_publisher_type: str,
    minimum_ack_comment_id: int = 0,
) -> dict[str, Any] | None:
    comments = _paged_values(
        runtime,
        path=f"repos/{repository}/issues/{pull_request}/comments?per_page=100",
        cwd=repo,
    )
    return next(
        (
            comment
            for comment in reversed(comments)
            if int(comment.get("id", 0)) >= minimum_ack_comment_id
            and _github_identity_matches(
                comment.get("user"),
                login=request_publisher_login,
                account_id=request_publisher_id,
                account_type=request_publisher_type,
            )
            and _matches_platform_request_ack(
                _decode_platform_request_ack(str(comment.get("body", ""))),
                repository=repository,
                pull_request=pull_request,
                head_sha=head_sha,
            )
            and bool(str(comment.get("created_at", "")))
            and str(comment.get("created_at", "")) == str(comment.get("updated_at", ""))
        ),
        None,
    )


def _trusted_platform_response(
    runtime: Any,
    *,
    repo: Path,
    repository: str,
    pull_request: int,
    head_sha: str,
    reviewer_login: str,
    reviewer_id: int,
    reviewer_type: str,
    request_publisher_login: str,
    request_publisher_id: int,
    request_publisher_type: str,
    minimum_request_comment_id: int = 0,
    required_request_comment_id: int = 0,
) -> dict[str, Any]:
    comments = _paged_values(
        runtime,
        path=f"repos/{repository}/issues/{pull_request}/comments?per_page=100",
        cwd=repo,
    )
    request_comment = next(
        (
            comment
            for comment in reversed(comments)
            if _github_identity_matches(
                comment.get("user"),
                login=request_publisher_login,
                account_id=request_publisher_id,
                account_type=request_publisher_type,
            )
            and _matches_platform_request(
                _decode_platform_request_marker(str(comment.get("body", ""))),
                repository=repository,
                pull_request=pull_request,
                head_sha=head_sha,
            )
            and bool(str(comment.get("created_at", "")))
            and str(comment.get("created_at", "")) == str(comment.get("updated_at", ""))
            and int(comment.get("id", 0)) >= minimum_request_comment_id
            and (
                required_request_comment_id < 1
                or int(comment.get("id", 0)) == required_request_comment_id
            )
        ),
        None,
    )
    request_comment_id = int(request_comment.get("id", 0)) if request_comment else 0
    request_created_at = str(request_comment.get("created_at", "")) if request_comment else ""
    if request_comment is None and minimum_request_comment_id > 0:
        return {"kind": "pending", "request_comment_id": 0}
    reviews = _paged_values(
        runtime,
        path=f"repos/{repository}/pulls/{pull_request}/reviews?per_page=100",
        cwd=repo,
    )
    matching_reviews = [
        review
        for review in reviews
        if _github_identity_matches(
            review.get("user"),
            login=reviewer_login,
            account_id=reviewer_id,
            account_type=reviewer_type,
        )
        and str(review.get("commit_id", "")).casefold() == head_sha.casefold()
        and (not request_created_at or str(review.get("submitted_at", "")) >= request_created_at)
    ]
    review = matching_reviews[-1] if matching_reviews else None
    review_created_at = str(review.get("submitted_at", "")) if review else ""
    clean: dict[str, Any] | None = None
    if request_comment_id > 0:
        reactions = _paged_values(
            runtime,
            path=(
                f"repos/{repository}/issues/comments/{request_comment_id}/reactions?per_page=100"
            ),
            cwd=repo,
        )
        clean = next(
            (
                reaction
                for reaction in reversed(reactions)
                if str(reaction.get("content", "")) == "+1"
                and _github_identity_matches(
                    reaction.get("user"),
                    login=reviewer_login,
                    account_id=reviewer_id,
                    account_type=reviewer_type,
                )
                and str(reaction.get("created_at", "")) >= request_created_at
            ),
            None,
        )
    clean_created_at = str(clean.get("created_at", "")) if clean else ""
    if review is not None and review_created_at >= clean_created_at:
        return {
            "kind": "review_with_findings",
            "review_id": int(review.get("id", 0)),
            "review_state": str(review.get("state", "")),
            "request_comment_id": request_comment_id,
            "result_created_at": review_created_at,
        }
    if clean is not None:
        return {
            "kind": "clean_reaction",
            "reaction_id": int(clean.get("id", 0)),
            "request_comment_id": request_comment_id,
            "result_created_at": clean_created_at,
        }
    return {"kind": "pending", "request_comment_id": request_comment_id}


def runtime_request_platform_review_github(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    repository: str,
    pull_request: int,
    expected_head: str,
    request_publisher_login: str,
    request_publisher_id: int,
    request_publisher_type: str = "Bot",
    reviewer_login: str = "chatgpt-codex-connector[bot]",
    reviewer_id: int = 199175422,
    reviewer_type: str = "Bot",
    retry_after_seconds: int = 900,
) -> dict[str, Any]:
    runtime = _execution()
    runtime._require_github_actions_token()
    repo, root = _require_external(target_repo, runtime_root)
    live = runtime._pull_request_state(repository=repository, pull_request=pull_request, cwd=repo)
    current_head = str(live["head_sha"])
    if not expected_head or expected_head.casefold() != current_head.casefold():
        raise ValueError("platform review request head does not match the live pull request")
    if not request_publisher_login.strip() or request_publisher_id < 1:
        raise ValueError("platform review request requires a trusted publisher identity")
    if not reviewer_login.strip() or reviewer_id < 1 or retry_after_seconds < 1:
        raise ValueError("platform review request requires reviewer identity and retry window")
    comments = _paged_values(
        runtime,
        path=f"repos/{repository}/issues/{pull_request}/comments?per_page=100",
        cwd=repo,
    )
    previous = next(
        (
            comment
            for comment in reversed(comments)
            if _github_identity_matches(
                comment.get("user"),
                login=request_publisher_login,
                account_id=request_publisher_id,
                account_type=request_publisher_type,
            )
            and _matches_platform_request(
                _decode_platform_request_marker(str(comment.get("body", ""))),
                repository=repository,
                pull_request=pull_request,
                head_sha=current_head,
            )
            and bool(str(comment.get("created_at", "")))
            and str(comment.get("created_at", "")) == str(comment.get("updated_at", ""))
        ),
        None,
    )
    if previous is not None:
        previous_id = int(previous.get("id", 0))
        reactions = _paged_values(
            runtime,
            path=f"repos/{repository}/issues/comments/{previous_id}/reactions?per_page=100",
            cwd=repo,
        )
        pending = any(
            str(reaction.get("content", "")) == "eyes"
            and _github_identity_matches(
                reaction.get("user"),
                login=reviewer_login,
                account_id=reviewer_id,
                account_type=reviewer_type,
            )
            for reaction in reactions
        )
        completed = any(
            str(reaction.get("content", "")) == "+1"
            and _github_identity_matches(
                reaction.get("user"),
                login=reviewer_login,
                account_id=reviewer_id,
                account_type=reviewer_type,
            )
            for reaction in reactions
        )
        previous_created = str(previous.get("created_at", ""))
        if not completed:
            reviews = _paged_values(
                runtime,
                path=f"repos/{repository}/pulls/{pull_request}/reviews?per_page=100",
                cwd=repo,
            )
            completed = any(
                _github_identity_matches(
                    review.get("user"),
                    login=reviewer_login,
                    account_id=reviewer_id,
                    account_type=reviewer_type,
                )
                and str(review.get("commit_id", "")).casefold() == current_head.casefold()
                and str(review.get("submitted_at", "")) >= previous_created
                for review in reviews
            )
        created_at = _github_datetime(previous.get("created_at"))
        within_grace = bool(
            created_at is not None
            and (datetime.now(UTC) - created_at).total_seconds() < retry_after_seconds
        )
        if not completed and (pending or within_grace):
            request = previous
            request_status = "in_progress"
        else:
            request = None
            request_status = "requested"
    else:
        request = None
        request_status = "requested"
    if request is None:
        request = runtime._run_gh_api(
            path=f"repos/{repository}/issues/{pull_request}/comments",
            method="POST",
            payload={
                "body": (
                    "@codex review\n\n"
                    "Harness request: publish a fresh independent current-head review through "
                    "the Codex GitHub integration. Each retry uses a new immutable request.\n\n"
                    + _platform_request_marker(
                        repository=repository,
                        pull_request=pull_request,
                        head_sha=current_head,
                    )
                )
            },
            cwd=repo,
        )
    if not isinstance(request, dict) or not _github_identity_matches(
        request.get("user"),
        login=request_publisher_login,
        account_id=request_publisher_id,
        account_type=request_publisher_type,
    ):
        raise ValueError("platform review request was not published by the trusted bot")
    comment_id = int(request.get("id", 0))
    if comment_id < 1:
        raise ValueError("platform review request comment ID is missing")
    acknowledgement = runtime._run_gh_api(
        path=f"repos/{repository}/issues/{pull_request}/comments",
        method="POST",
        payload={
            "body": (
                "Harness platform review request acknowledged.\n\n"
                + _platform_request_ack_marker(
                    repository=repository,
                    pull_request=pull_request,
                    head_sha=current_head,
                    request_comment_id=comment_id,
                )
            )
        },
        cwd=repo,
    )
    if not isinstance(acknowledgement, dict) or not _github_identity_matches(
        acknowledgement.get("user"),
        login=request_publisher_login,
        account_id=request_publisher_id,
        account_type=request_publisher_type,
    ):
        raise ValueError("platform review acknowledgement was not published by the trusted bot")
    acknowledgement_id = int(acknowledgement.get("id", 0))
    if acknowledgement_id < 1:
        raise ValueError("platform review acknowledgement comment ID is missing")
    _event(
        root=root,
        repo=repo,
        event_type=EventType.REVIEW_AGENT_REQUESTED,
        repository=repository,
        pull_request=pull_request,
        base_sha=str(live["base_sha"]),
        head_sha=current_head,
        task_id=task_id,
        session_id=f"platform-review-request-pr-{pull_request}",
        result=request_status,
        extra={
            "request_comment_id": comment_id,
            "acknowledgement_comment_id": acknowledgement_id,
            "request_publisher_login": request_publisher_login,
            "request_publisher_id": request_publisher_id,
        },
    )
    return {
        "status": request_status,
        "repository": repository,
        "pull_request": pull_request,
        "head_sha": current_head,
        "request_comment_id": comment_id,
        "acknowledgement_comment_id": acknowledgement_id,
        "retry_after_seconds": retry_after_seconds,
    }


def runtime_publish_review(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    repository: str,
    pull_request: int,
    bundle: Path | str,
    trusted_reviewer_login: str,
    trusted_reviewer_id: int,
    trusted_reviewer_type: str = "Bot",
    trusted_request_publisher_login: str = "github-actions[bot]",
    trusted_request_publisher_id: int = 41898282,
    trusted_request_publisher_type: str = "Bot",
    platform_wait_seconds: int = 0,
    platform_poll_seconds: int = 5,
) -> dict[str, Any]:
    runtime = _execution()
    repo, root = _require_external(target_repo, runtime_root)
    cycle = _load_cycle(bundle)
    live = runtime._pull_request_state(repository=repository, pull_request=pull_request, cwd=repo)
    validate_review_cycle(cycle, current_head_sha=str(live["head_sha"]))
    markdown_path = Path(bundle).with_suffix(".md")
    markdown_source = markdown_path.read_text(encoding="utf-8") if markdown_path.is_file() else ""
    body = _github_body(cycle, markdown_source=markdown_source)
    actor = runtime._gh_actor_login(cwd=repo)
    comments = _paged_values(
        runtime,
        path=f"repos/{repository}/issues/{pull_request}/comments?per_page=100",
        cwd=repo,
    )
    existing_id = 0
    for comment in reversed(comments):
        user = comment.get("user") or {}
        if not isinstance(user, dict) or str(user.get("login", "")) != actor:
            continue
        comment_body = str(comment.get("body", ""))
        if not existing_id and _matches_review_summary(
            _decode_review_summary_marker(comment_body),
            repository=repository,
            pull_request=pull_request,
        ):
            existing_id = int(comment["id"])
        if existing_id:
            break
    if existing_id:
        summary = runtime._run_gh_api(
            path=f"repos/{repository}/issues/comments/{existing_id}",
            method="PATCH",
            payload={"body": body},
            cwd=repo,
        )
    else:
        summary = runtime._run_gh_api(
            path=f"repos/{repository}/issues/{pull_request}/comments",
            method="POST",
            payload={"body": body},
            cwd=repo,
        )
    if not trusted_reviewer_login.strip() or trusted_reviewer_id < 1:
        raise ValueError("review publication requires an explicit trusted reviewer identity")
    if not trusted_request_publisher_login.strip() or trusted_request_publisher_id < 1:
        raise ValueError("review publication requires a trusted request publisher identity")
    if platform_wait_seconds < 0 or platform_poll_seconds < 1:
        raise ValueError("platform review wait values must be non-negative with a positive poll")
    prior_acknowledgement = _latest_platform_request_ack(
        runtime,
        repo=repo,
        repository=repository,
        pull_request=pull_request,
        head_sha=cycle.head_sha,
        request_publisher_login=trusted_request_publisher_login,
        request_publisher_id=trusted_request_publisher_id,
        request_publisher_type=trusted_request_publisher_type,
    )
    prior_acknowledgement_id = (
        int(prior_acknowledgement.get("id", 0)) if prior_acknowledgement else 0
    )
    runtime._run_gh_api(
        path=f"repos/{repository}/dispatches",
        method="POST",
        payload={
            "event_type": "agentic_review_requested",
            "client_payload": {"pull_request": pull_request, "head_sha": cycle.head_sha},
        },
        cwd=repo,
    )
    deadline = time.monotonic() + platform_wait_seconds
    acknowledgement = _latest_platform_request_ack(
        runtime,
        repo=repo,
        repository=repository,
        pull_request=pull_request,
        head_sha=cycle.head_sha,
        request_publisher_login=trusted_request_publisher_login,
        request_publisher_id=trusted_request_publisher_id,
        request_publisher_type=trusted_request_publisher_type,
        minimum_ack_comment_id=prior_acknowledgement_id + 1,
    )
    while acknowledgement is None and time.monotonic() < deadline:
        time.sleep(min(platform_poll_seconds, max(0.0, deadline - time.monotonic())))
        acknowledgement = _latest_platform_request_ack(
            runtime,
            repo=repo,
            repository=repository,
            pull_request=pull_request,
            head_sha=cycle.head_sha,
            request_publisher_login=trusted_request_publisher_login,
            request_publisher_id=trusted_request_publisher_id,
            request_publisher_type=trusted_request_publisher_type,
            minimum_ack_comment_id=prior_acknowledgement_id + 1,
        )
    acknowledgement_payload = (
        _decode_platform_request_ack(str(acknowledgement.get("body", "")))
        if acknowledgement is not None
        else None
    )
    raw_acknowledged_request_id = (
        acknowledgement_payload.get("request_comment_id", 0)
        if isinstance(acknowledgement_payload, dict)
        else 0
    )
    acknowledged_request_id = (
        raw_acknowledged_request_id
        if isinstance(raw_acknowledged_request_id, int)
        and not isinstance(raw_acknowledged_request_id, bool)
        else 0
    )
    platform_response: dict[str, Any] = {
        "kind": "pending",
        "request_comment_id": acknowledged_request_id,
    }
    if acknowledged_request_id > 0:
        platform_response = _trusted_platform_response(
            runtime,
            repo=repo,
            repository=repository,
            pull_request=pull_request,
            head_sha=cycle.head_sha,
            reviewer_login=trusted_reviewer_login,
            reviewer_id=trusted_reviewer_id,
            reviewer_type=trusted_reviewer_type,
            request_publisher_login=trusted_request_publisher_login,
            request_publisher_id=trusted_request_publisher_id,
            request_publisher_type=trusted_request_publisher_type,
            required_request_comment_id=acknowledged_request_id,
        )
        while platform_response["kind"] == "pending" and time.monotonic() < deadline:
            time.sleep(min(platform_poll_seconds, max(0.0, deadline - time.monotonic())))
            platform_response = _trusted_platform_response(
                runtime,
                repo=repo,
                repository=repository,
                pull_request=pull_request,
                head_sha=cycle.head_sha,
                reviewer_login=trusted_reviewer_login,
                reviewer_id=trusted_reviewer_id,
                reviewer_type=trusted_reviewer_type,
                request_publisher_login=trusted_request_publisher_login,
                request_publisher_id=trusted_request_publisher_id,
                request_publisher_type=trusted_request_publisher_type,
                required_request_comment_id=acknowledged_request_id,
            )
    dispatch_event = ""
    if platform_response["kind"] != "pending":
        runtime._run_gh_api(
            path=f"repos/{repository}/dispatches",
            method="POST",
            payload={
                "event_type": "agentic_review_recorded",
                "client_payload": {"pull_request": pull_request, "head_sha": cycle.head_sha},
            },
            cwd=repo,
        )
        dispatch_event = "agentic_review_recorded"
    _event(
        root=root,
        repo=repo,
        event_type=EventType.REVIEW_PUBLISHED,
        repository=repository,
        pull_request=pull_request,
        base_sha=cycle.base_sha,
        head_sha=cycle.head_sha,
        task_id=task_id,
        session_id=f"review-publish-{cycle.reviewer_session_id}",
        result=(
            "published" if platform_response["kind"] != "pending" else "awaiting_platform_review"
        ),
        requested_by_session_id=cycle.context.requested_by_session_id,
        reviewer_session_id=cycle.reviewer_session_id,
        extra={
            "summary_comment_id": int(summary.get("id", 0)) if isinstance(summary, dict) else 0,
            "platform_review_request_comment_id": int(
                platform_response.get("request_comment_id", 0)
            ),
            "platform_request_acknowledgement_comment_id": (
                int(acknowledgement.get("id", 0)) if acknowledgement is not None else 0
            ),
            "integrity_hash": cycle.integrity_hash,
            "publisher_authentication": "pending_codex_github_identity",
            "platform_response": platform_response,
        },
    )
    return {
        "status": (
            "published" if platform_response["kind"] != "pending" else "awaiting_platform_review"
        ),
        "repository": repository,
        "pull_request": pull_request,
        "head_sha": cycle.head_sha,
        "summary_comment_id": int(summary.get("id", 0)) if isinstance(summary, dict) else 0,
        "review_id": 0,
        "platform_review_request_comment_id": int(platform_response.get("request_comment_id", 0)),
        "platform_request_acknowledgement_comment_id": (
            int(acknowledgement.get("id", 0)) if acknowledgement is not None else 0
        ),
        "request_dispatch_event": "agentic_review_requested",
        "dispatch_event": dispatch_event,
        "platform_response": platform_response,
    }


def runtime_validate_review_github(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    repository: str,
    pull_request: int,
    expected_head: str = "",
    trusted_reviewer_login: str = "",
    trusted_reviewer_id: int = 0,
    trusted_reviewer_type: str = "Bot",
    trusted_request_publisher_login: str = "github-actions[bot]",
    trusted_request_publisher_id: int = 41898282,
    trusted_request_publisher_type: str = "Bot",
) -> dict[str, Any]:
    runtime = _execution()
    runtime._require_github_actions_token()
    repo, root = _require_external(target_repo, runtime_root)
    live = runtime._pull_request_state(repository=repository, pull_request=pull_request, cwd=repo)
    current_head = str(live["head_sha"])
    reviewer_login = trusted_reviewer_login.strip()
    reviewer_type = trusted_reviewer_type.strip()
    if not reviewer_login or trusted_reviewer_id < 1 or not reviewer_type:
        raise ValueError("review validation requires an explicit trusted reviewer identity")
    request_publisher_login = trusted_request_publisher_login.strip()
    request_publisher_type = trusted_request_publisher_type.strip()
    if (
        not request_publisher_login
        or trusted_request_publisher_id < 1
        or not request_publisher_type
    ):
        raise ValueError("review validation requires a trusted request publisher identity")
    platform_response = _trusted_platform_response(
        runtime,
        repo=repo,
        repository=repository,
        pull_request=pull_request,
        head_sha=current_head,
        reviewer_login=reviewer_login,
        reviewer_id=trusted_reviewer_id,
        reviewer_type=reviewer_type,
        request_publisher_login=request_publisher_login,
        request_publisher_id=trusted_request_publisher_id,
        request_publisher_type=request_publisher_type,
    )
    blockers: list[str] = []
    if expected_head and expected_head.casefold() != current_head.casefold():
        blockers.append("expected_head_mismatch")
    if platform_response["kind"] == "pending":
        blockers.append("missing_trusted_reviewer_attestation")
    elif platform_response["kind"] == "review_with_findings":
        blockers.append("trusted_reviewer_blocking_findings")
    conclusion = "success" if not blockers else "failure"
    check = runtime._run_gh_api(
        path=f"repos/{repository}/check-runs",
        method="POST",
        payload={
            "name": "Agentic Review",
            "head_sha": current_head,
            "status": "completed",
            "conclusion": conclusion,
            "external_id": f"codex-agent-review:pr-{pull_request}:{current_head}",
            "output": {
                "title": "Trusted Codex review current"
                if not blockers
                else "Trusted Codex review blocked",
                "summary": "The pinned Codex GitHub identity recorded a clean exact-head result."
                if not blockers
                else "\n".join(f"- {item}" for item in blockers),
            },
        },
        cwd=repo,
    )
    _event(
        root=root,
        repo=repo,
        event_type=EventType.REVIEW_VALIDATED,
        repository=repository,
        pull_request=pull_request,
        base_sha=str(live["base_sha"]),
        head_sha=current_head,
        task_id=task_id,
        session_id=f"review-validator-pr-{pull_request}",
        result="ok" if not blockers else "blocked",
        extra={
            "blockers": blockers,
            "trusted_reviewer_login": reviewer_login,
            "trusted_reviewer_id": trusted_reviewer_id,
            "trusted_reviewer_type": reviewer_type,
            "trusted_request_publisher_login": request_publisher_login,
            "trusted_request_publisher_id": trusted_request_publisher_id,
            "trusted_request_publisher_type": request_publisher_type,
            "platform_response": platform_response,
            "local_cycle_security_authority": False,
            "check_id": int(check.get("id", 0)) if isinstance(check, dict) else 0,
            "check_conclusion": conclusion,
        },
    )
    return {
        "status": "ok" if not blockers else "blocked",
        "repository": repository,
        "pull_request": pull_request,
        "head_sha": current_head,
        "blockers": blockers,
        "check_id": int(check.get("id", 0)) if isinstance(check, dict) else 0,
        "trusted_review_id": int(platform_response.get("review_id", 0)),
        "trusted_clean_reaction_id": int(platform_response.get("reaction_id", 0)),
    }


def _response_markdown(response: ReviewResponse, cycle: ReviewCycle) -> str:
    return "\n".join(
        (
            "# Software Team Member Response",
            "",
            f"## {response.finding_id}",
            "",
            f"Decision: {response.decision.upper()}",
            "",
            "Reason:",
            response.rationale or "Accepted for correction.",
            "",
            "Evidence:",
            *([f"- {item}" for item in response.evidence] or ["- Pending fix validation."]),
            "",
            f"Reviewed head: `{cycle.head_sha}`",
            "",
        )
    )


def runtime_record_review_response(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    repository: str,
    pull_request: int,
    head_sha: str,
    finding_id: str,
    decision: str,
    rationale: str,
    evidence: tuple[str, ...],
    responder_role: str,
) -> dict[str, Any]:
    repo, root = _require_external(target_repo, runtime_root)
    cycle = _latest_cycle(root, repository, pull_request)
    validate_review_cycle(cycle, current_head_sha=head_sha)
    response = ReviewResponse(
        finding_id=finding_id,
        decision=decision,
        rationale=rationale,
        evidence=evidence,
        responder_role=responder_role,
    )
    _event(
        root=root,
        repo=repo,
        event_type=EventType.REVIEW_RESPONSE_STARTED,
        repository=repository,
        pull_request=pull_request,
        base_sha=cycle.base_sha,
        head_sha=head_sha,
        task_id=task_id,
        session_id=f"review-response-{finding_id}",
        result="started",
        extra={"finding_id": finding_id},
    )
    updated = record_review_response(cycle, response)
    directory = _cycle_directory(root, repository, pull_request)
    number = len(tuple(directory.glob("review-response-cycle-*.json"))) + 1
    json_path = _write_json(
        directory / f"review-response-cycle-{number:03d}.json", response.to_dict()
    )
    markdown_path = write_private_text(
        directory / f"review-response-cycle-{number:03d}.md",
        _response_markdown(response, updated),
    )
    _write_json(directory / "latest-review.json", updated.to_dict())
    _write_json(directory / "review-state.json", updated.to_dict())
    event_type = (
        EventType.REVIEW_FINDING_ACCEPTED
        if response.decision == "accept"
        else EventType.REVIEW_FINDING_REJECTED
    )
    _event(
        root=root,
        repo=repo,
        event_type=event_type,
        repository=repository,
        pull_request=pull_request,
        base_sha=cycle.base_sha,
        head_sha=head_sha,
        task_id=task_id,
        session_id=f"review-response-{finding_id}",
        result=response.decision,
        extra={"finding_id": finding_id, "response_path": str(json_path)},
    )
    return {
        "status": "recorded",
        "finding_id": finding_id,
        "decision": response.decision,
        "json": str(json_path),
        "markdown": str(markdown_path),
    }


def runtime_mark_review_stale(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    repository: str,
    pull_request: int,
    previous_head_sha: str,
    new_head_sha: str,
) -> dict[str, Any]:
    repo, root = _require_external(target_repo, runtime_root)
    cycle = _latest_cycle(root, repository, pull_request)
    if cycle.head_sha != previous_head_sha:
        raise ValueError("previous_head_sha does not match the latest review")
    stale = mark_review_stale(cycle, new_head_sha=new_head_sha)
    directory = _cycle_directory(root, repository, pull_request)
    _write_json(directory / "latest-review.json", stale.to_dict())
    _write_json(directory / "review-state.json", stale.to_dict())
    for event_type, result in (
        (EventType.REVIEW_MARKED_STALE, "stale"),
        (EventType.REVIEW_RERUN_REQUESTED, "requested"),
    ):
        _event(
            root=root,
            repo=repo,
            event_type=event_type,
            repository=repository,
            pull_request=pull_request,
            base_sha=cycle.base_sha,
            head_sha=new_head_sha,
            task_id=task_id,
            session_id=f"review-stale-pr-{pull_request}",
            result=result,
            extra={"previous_head_sha": previous_head_sha},
        )
    return {
        "status": "stale",
        "repository": repository,
        "pull_request": pull_request,
        "previous_head_sha": previous_head_sha,
        "new_head_sha": new_head_sha,
        "rerun_request": f"review {repository} PR #{pull_request}",
    }
