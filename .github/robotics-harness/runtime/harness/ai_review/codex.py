"""Trusted Codex review preparation, local audits, and publication helpers."""

from __future__ import annotations

import argparse
import base64
import fnmatch
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import replace
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, cast

from harness.ai_review.diff import DiffBundle, DiffCollectionError, collect_full_diff
from harness.ai_review.github import (
    GitHubApi,
    GitHubApiError,
    PullRequestEvent,
    _load_json,
    _read_optional,
    _reviewer_group,
    _risk_assessment,
    _target_policy,
    _trusted_policy,
    changed_new_lines,
)
from harness.ai_review.models import (
    AIReview,
    Finding,
    ProviderReview,
    ReviewContext,
    ReviewRequest,
    ReviewResult,
)
from harness.ai_review.prompt import build_provider_payload
from harness.ai_review.report import SUMMARY_MARKER
from harness.ai_review.service import AIReviewService, PullRequestMetadata
from harness.monitoring.emitter import emit_service_event
from harness.monitoring.models import EventType
from harness.private_io import write_private_text

_REPO_ROOT = Path(__file__).resolve().parents[2]
_PR_PROMPT_PATH = _REPO_ROOT / ".github/codex/prompts/full-diff-review-v1.md"
_REPOSITORY_PROMPT_PATH = _REPO_ROOT / ".github/codex/prompts/repository-review.md"
_PROVIDER_SCHEMA_PATH = _REPO_ROOT / ".github/codex/schemas/provider-review-output-v1.json"
_FINAL_SCHEMA_PATH = _REPO_ROOT / ".github/codex/review.schema.json"
_OUTBOUND_POLICY_PATH = _REPO_ROOT / ".github/codex/outbound-data-policy.json"
_REQUEST_FILE = "review-request.json"
_OUTPUT_FILE = "provider-review.json"
_PUBLISH_FILE = "publication-bundle.json"
_EMPTY_SHA256 = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
_REPO_URL = re.compile(r"github\.com[:/](?P<owner>[^/]+)/(?P<repo>[^/.]+)(?:\.git)?$")
_PR_PROMPT_VERSION = "full-diff-review-v1"
_REPOSITORY_PROMPT_VERSION = "repository-review-v1"
_POLICY_VERSION = "2026-08-09"
_FINAL_STATE_PREFIX = "<!-- robotics-harness-codex-review-state:"
_FINAL_STATE_SUFFIX = " -->"
_SENSITIVE_PATTERNS: tuple[tuple[str, re.Pattern[str]], ...] = (
    ("absolute_path", re.compile(r"/Users/[^/\s]+/|/home/[^/\s]+/|[A-Z]:\\\\Users\\\\", re.I)),
    ("private_key", re.compile(r"-----BEGIN [A-Z ]*PRIVATE KEY-----")),
    ("bearer_token", re.compile(r"bearer\s+[A-Za-z0-9._-]{16,}", re.I)),
    ("openai_key", re.compile(r"\bsk-[A-Za-z0-9_-]{12,}\b")),
    ("github_pat", re.compile(r"\bgithub_pat_[A-Za-z0-9_]{16,}\b")),
    ("github_token", re.compile(r"\bgh[pousr]_[A-Za-z0-9_]{12,}\b")),
    ("aws_access_key", re.compile(r"\bAKIA[0-9A-Z]{12,}\b")),
    ("slack_token", re.compile(r"\bxox[baprs]-[A-Za-z0-9-]{10,}\b")),
    ("azure_secret", re.compile(r"azure[_-]openai[_-]api[_-]key\s*[:=]\s*\\S+", re.I)),
    (
        "credential_assignment",
        re.compile(
            r"""
            (?:
                ^|[\r\n]
            )
            \s*
            (?:export\s+)?
            (?:
                FRC1884_GOVERNANCE_READ_TOKEN
                |
                [A-Za-z_][A-Za-z0-9_]*?(?:key|token|password|passwd|secret)[A-Za-z0-9_]*
            )
            \s*[:=]\s*
            \S+
            """,
            re.I | re.VERBOSE,
        ),
    ),
    ("password", re.compile(r"(password|passwd|smtp_password)\s*[:=]\s*\\S{8,}", re.I)),
    ("basic_auth", re.compile(r"authorization\s*:\s*basic\s+[A-Za-z0-9+/=]{12,}", re.I)),
    ("codex_auth_path", re.compile(r"(^|[\s`'\"/])\.codex/auth([./][^\s`'\"]*)?", re.I)),
    (
        "secret_file",
        re.compile(r"(^|/)(?:credentials?|secrets?)\.(?:json|ya?ml|txt|properties)\b", re.I),
    ),
    ("monitoring_store_path", re.compile(r"(^|[\s`'\"/])monitoring\.jsonl($|[\s`'\"])", re.I)),
    (
        "private_store_path",
        re.compile(
            r"(^|[\s`'\"/])\.omx/|(^|[\s`'\"])notepad\.md($|[\s`'\"])",
            re.I,
        ),
    ),
    ("personal_record", re.compile(r"(student_id|birthdate)\s*[:=]\s*\S+", re.I)),
)
_PLACEHOLDER_CHUNK_ID = "sanitized-incomplete"
_DEFAULT_MAX_INPUT_BYTES = 2_000_000
_DEFAULT_MAX_OUTPUT_BYTES = 256_000
_DEFAULT_TIMEOUT_SECONDS = 180.0


def _outbound_data_policy() -> dict[str, Any]:
    value = json.loads(_OUTBOUND_POLICY_PATH.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or value.get("schema_version") != 1:
        raise ValueError("outbound-data policy must use schema_version 1")
    patterns = value.get("sensitive_path_patterns")
    if not isinstance(patterns, list) or not all(
        isinstance(pattern, str) and pattern for pattern in patterns
    ):
        raise ValueError("outbound-data policy sensitive_path_patterns are invalid")
    if value.get("maximum_provider_requests_per_review") != 1:
        raise ValueError("Codex review permits exactly one provider request")
    for key in ("max_input_bytes", "max_output_bytes", "timeout_seconds"):
        if not isinstance(value.get(key), int) or int(value[key]) <= 0:
            raise ValueError(f"outbound-data policy {key} must be a positive integer")
    return value


def _json_dump(value: object) -> str:
    return json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n"


def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _runtime_paths(root: Path) -> tuple[Path, Path]:
    configured = os.environ.get("HARNESS_REVIEW_RUNTIME_DIR", "").strip()
    runtime = (
        Path(configured).expanduser().resolve()
        if configured
        else (
            Path(tempfile.gettempdir())
            / "robotics-harness-review"
            / _sha256_text(str(root.resolve()))[:16]
        ).resolve()
    )
    if runtime.is_relative_to(root.resolve()):
        raise ValueError("HARNESS_REVIEW_RUNTIME_DIR must be outside the target repository")
    return runtime / _REQUEST_FILE, runtime / _OUTPUT_FILE


def _positive_budget(name: str, default: int) -> int:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        value = int(raw)
    except ValueError as error:
        raise ValueError(f"{name} must be a positive integer") from error
    if value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _provider_request_document(request: ReviewRequest, *, prompt_version: str) -> str:
    return _json_dump(
        {
            "prompt_version": prompt_version,
            "output_schema_version": "provider-review-output-v1",
            "review": build_provider_payload(request),
        }
    )


def _github_token() -> str:
    for name in ("GITHUB_TOKEN", "GH_TOKEN"):
        value = os.environ.get(name, "").strip()
        if value:
            return value
    raise ValueError("GitHub authentication is unavailable; export GITHUB_TOKEN or GH_TOKEN")


def _git_output(repository: Path, *args: str) -> str:
    completed = subprocess.run(
        ("git", *args),
        cwd=repository,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        stderr = completed.stderr.strip() or f"git {' '.join(args)} failed"
        raise ValueError(stderr)
    return completed.stdout.strip()


def _git_root(start: Path) -> Path:
    return Path(_git_output(start, "rev-parse", "--show-toplevel")).resolve()


def _origin_repository_identity(repository: Path) -> str:
    remote = _git_output(repository, "remote", "get-url", "origin")
    match = _REPO_URL.search(remote)
    if match is None:
        raise ValueError("origin remote does not resolve to a GitHub owner/repository identity")
    return f"{match.group('owner')}/{match.group('repo')}"


def _head_sha(repository: Path) -> str:
    return _git_output(repository, "rev-parse", "HEAD")


def _merge_base_sha(repository: Path, base_ref: str) -> str:
    try:
        return _git_output(repository, "merge-base", "HEAD", base_ref)
    except ValueError:
        fallback = f"origin/{base_ref}"
        return _git_output(repository, "merge-base", "HEAD", fallback)


def _empty_tree_sha(repository: Path) -> str:
    completed = subprocess.run(
        ("git", "hash-object", "-t", "tree", "--stdin"),
        cwd=repository,
        input="",
        text=True,
        check=False,
        capture_output=True,
    )
    value = completed.stdout.strip()
    if completed.returncode != 0 or not re.fullmatch(r"[0-9a-fA-F]{40,64}", value):
        raise ValueError("could not resolve the repository's empty-tree object")
    return value


def _current_branch_name(repository: Path) -> str:
    return _git_output(repository, "rev-parse", "--abbrev-ref", "HEAD")


def _default_output_path(
    repository: Path, kind: str, head_sha: str, pull_request: int | None = None
) -> Path:
    root = repository / "artifacts" / "codex-review-runtime"
    stem = f"pr-{pull_request}-{head_sha}" if pull_request is not None else f"{kind}-{head_sha}"
    return root / f"{stem}.review.json"


def _review_request(
    metadata: PullRequestMetadata, diff: DiffBundle, context: ReviewContext
) -> ReviewRequest:
    return ReviewRequest(
        pull_request=metadata.number,
        repository=metadata.repository,
        base_sha=diff.base_sha,
        head_sha=diff.head_sha,
        title=metadata.title,
        body=metadata.body,
        commit_messages=diff.commit_messages,
        context=context,
        chunk_ids=tuple(chunk.chunk_id for chunk in diff.chunks),
        untrusted_diff_chunks=tuple(chunk.content for chunk in diff.chunks),
        reviewed_paths=diff.changed_paths,
    )


def _request_texts(request: ReviewRequest) -> tuple[tuple[str, str], ...]:
    return (
        ("trusted.repository_policy", request.context.repository_policy),
        ("trusted.target_policy", request.context.target_policy),
        ("trusted.task_specification", request.context.task_specification),
        ("trusted.confirmed_plan", request.context.confirmed_plan),
        ("trusted.protected_path_result", request.context.protected_path_result),
        ("trusted.learning_review_status", request.context.learning_review_status),
        ("trusted.test_evidence", json.dumps(request.context.test_evidence, sort_keys=True)),
        ("untrusted.pull_request_title", request.title),
        ("untrusted.pull_request_body", request.body),
        ("untrusted.commit_messages", "\n".join(request.commit_messages)),
        ("untrusted.reviewed_paths", "\n".join(request.reviewed_paths)),
        ("untrusted.chunk_ids", "\n".join(request.chunk_ids)),
        ("untrusted.diff_chunks", "\n".join(request.untrusted_diff_chunks)),
    )


def deterministic_data_gate(request: ReviewRequest) -> tuple[str, ...]:
    findings: list[str] = []
    policy = _outbound_data_policy()
    sensitive_paths = tuple(str(item) for item in policy["sensitive_path_patterns"])
    for path in request.reviewed_paths:
        if any(fnmatch.fnmatchcase(path, pattern) for pattern in sensitive_paths):
            findings.append("untrusted.reviewed_paths:sensitive_path")
    for label, text in _request_texts(request):
        for category, pattern in _SENSITIVE_PATTERNS:
            if pattern.search(text):
                findings.append(f"{label}:{category}")
    return tuple(sorted(dict.fromkeys(findings)))


def _placeholder_provider_review(reason: str) -> ProviderReview:
    return ProviderReview.from_dict(
        {
            "result": "review_incomplete",
            "summary": f"AI review incomplete: {reason}",
            "risk_level": "critical",
            "findings": [],
            "analysed_chunk_ids": [_PLACEHOLDER_CHUNK_ID],
        }
    )


def _provider_output_or_incomplete(path: Path, *, reason: str) -> ProviderReview:
    if not path.exists():
        return _placeholder_provider_review(reason)
    if path.stat().st_size > _positive_budget(
        "CODEX_REVIEW_MAX_OUTPUT_BYTES", _DEFAULT_MAX_OUTPUT_BYTES
    ):
        return _placeholder_provider_review(f"{reason}; provider output exceeded budget")
    try:
        return ProviderReview.from_dict(json.loads(path.read_text(encoding="utf-8")))
    except (OSError, json.JSONDecodeError, TypeError, ValueError) as error:
        return _placeholder_provider_review(f"{reason}; provider output was malformed: {error}")


def _incomplete_review(
    metadata: PullRequestMetadata,
    diff: DiffBundle,
    context: ReviewContext,
    reason: str,
    *,
    previous: AIReview | None,
    review_provider: str,
    required_reviewer_group: str,
) -> AIReview:
    return AIReview(
        pull_request=metadata.number,
        repository=metadata.repository,
        base_sha=diff.base_sha,
        head_sha=diff.head_sha,
        reviewed_at=datetime.now(UTC),
        result=ReviewResult.REVIEW_INCOMPLETE,
        summary=f"AI review incomplete: {reason}",
        risk_level=context.risk_classification,
        findings=(),
        resolved_findings=(),
        test_evidence=context.test_evidence,
        learning_evidence={"status": context.learning_review_status},
        required_reviewer_group=required_reviewer_group,
        diff_complete=False,
        analysed_file_paths=(),
        all_file_paths=diff.changed_paths,
        review_provider=review_provider,
        previous_head_sha="" if previous is None else previous.head_sha,
    )


def assemble_codex_review(
    metadata: PullRequestMetadata,
    diff: DiffBundle,
    context: ReviewContext,
    provider_review: ProviderReview,
    *,
    previous: AIReview | None,
    required_reviewer_group: str,
    review_provider: str,
) -> AIReview:
    if not diff.complete or set(diff.analysed_paths) != set(diff.changed_paths):
        return _incomplete_review(
            metadata,
            diff,
            context,
            diff.incomplete_reason or "full diff unavailable",
            previous=previous,
            review_provider=review_provider,
            required_reviewer_group=required_reviewer_group,
        )
    if provider_review.result is ReviewResult.REVIEW_INCOMPLETE:
        return _incomplete_review(
            metadata,
            diff,
            context,
            provider_review.summary,
            previous=previous,
            review_provider=review_provider,
            required_reviewer_group=required_reviewer_group,
        )
    expected_chunk_ids = {chunk.chunk_id for chunk in diff.chunks}
    if set(provider_review.analysed_chunk_ids) != expected_chunk_ids:
        return _incomplete_review(
            metadata,
            diff,
            context,
            "provider did not attest to every supplied complete-diff chunk",
            previous=previous,
            review_provider=review_provider,
            required_reviewer_group=required_reviewer_group,
        )
    merged_findings = tuple(provider_review.findings) + AIReviewService._injection_findings(diff)
    deduplicated = AIReviewService._deduplicate(merged_findings)
    current, resolved = AIReviewService._track_findings(deduplicated, previous)
    return AIReview(
        pull_request=metadata.number,
        repository=metadata.repository,
        base_sha=diff.base_sha,
        head_sha=diff.head_sha,
        reviewed_at=datetime.now(UTC),
        result=AIReviewService._result_for(current),
        summary=provider_review.summary,
        risk_level=provider_review.risk_level,
        findings=current,
        resolved_findings=resolved,
        test_evidence=context.test_evidence,
        learning_evidence={"status": context.learning_review_status},
        required_reviewer_group=required_reviewer_group,
        diff_complete=True,
        analysed_file_paths=diff.changed_paths,
        all_file_paths=diff.changed_paths,
        review_provider=review_provider,
        previous_head_sha="" if previous is None else previous.head_sha,
    )


def _read_schema(path: Path) -> dict[str, Any]:
    return cast(dict[str, Any], json.loads(path.read_text(encoding="utf-8")))


def _validate_schema(
    schema: dict[str, Any],
    value: Any,
    *,
    location: str = "$",
    root_schema: dict[str, Any] | None = None,
) -> None:
    root = schema if root_schema is None else root_schema
    reference = schema.get("$ref")
    if isinstance(reference, str):
        if not reference.startswith("#/"):
            raise ValueError(f"{location} uses unsupported schema reference {reference!r}")
        resolved: Any = root
        for part in reference[2:].split("/"):
            key = part.replace("~1", "/").replace("~0", "~")
            if not isinstance(resolved, dict) or key not in resolved:
                raise ValueError(f"{location} uses unresolved schema reference {reference!r}")
            resolved = resolved[key]
        if not isinstance(resolved, dict):
            raise ValueError(f"{location} schema reference must resolve to an object")
        _validate_schema(resolved, value, location=location, root_schema=root)
        return
    schema_type = schema.get("type")
    if isinstance(schema_type, list):
        for candidate in schema_type:
            try:
                _validate_schema(
                    {**schema, "type": candidate},
                    value,
                    location=location,
                    root_schema=root,
                )
                break
            except ValueError:
                continue
        else:
            raise ValueError(f"{location} does not match any allowed schema types")
        return
    if schema_type == "object":
        if not isinstance(value, dict):
            raise ValueError(f"{location} must be an object")
        required = schema.get("required", [])
        for key in required:
            if key not in value:
                raise ValueError(f"{location}.{key} is required")
        properties = schema.get("properties", {})
        if schema.get("additionalProperties") is False:
            extras = sorted(set(value) - set(properties))
            if extras:
                raise ValueError(f"{location} has unexpected properties: {', '.join(extras)}")
        for key, child in properties.items():
            if key in value:
                _validate_schema(
                    child,
                    value[key],
                    location=f"{location}.{key}",
                    root_schema=root,
                )
    elif schema_type == "array":
        if not isinstance(value, list):
            raise ValueError(f"{location} must be an array")
        item_schema = schema.get("items")
        if isinstance(item_schema, dict):
            for index, item in enumerate(value):
                _validate_schema(
                    item_schema,
                    item,
                    location=f"{location}[{index}]",
                    root_schema=root,
                )
    elif schema_type == "string":
        if not isinstance(value, str):
            raise ValueError(f"{location} must be a string")
    elif schema_type == "integer":
        if not isinstance(value, int) or isinstance(value, bool):
            raise ValueError(f"{location} must be an integer")
    elif schema_type == "boolean":
        if not isinstance(value, bool):
            raise ValueError(f"{location} must be a boolean")
    elif schema_type == "null":
        if value is not None:
            raise ValueError(f"{location} must be null")
    elif schema_type is not None:
        raise ValueError(f"{location} uses unsupported schema type {schema_type!r}")
    if "enum" in schema and value not in schema["enum"]:
        raise ValueError(f"{location} must be one of {schema['enum']!r}")
    if "const" in schema and value != schema["const"]:
        raise ValueError(f"{location} must equal {schema['const']!r}")


def _validated_final_artifact(artifact: dict[str, Any]) -> dict[str, Any]:
    schema = _read_schema(_FINAL_SCHEMA_PATH)
    _validate_schema(schema, artifact)
    required = {
        "schema_version",
        "review_kind",
        "repository",
        "pull_request",
        "base_sha",
        "head_sha",
        "previous_head_sha",
        "prompt_version",
        "policy_version",
        "model",
        "reviewed_at",
        "result",
        "risk_level",
        "complete",
        "reviewed_files",
        "omitted_files",
        "diff_digest",
        "findings",
        "new_findings",
        "still_present_findings",
        "changed_findings",
        "resolved_findings",
        "human_review_required",
    }
    missing = sorted(required - set(artifact))
    if missing:
        raise ValueError(f"final artifact is missing required keys: {', '.join(missing)}")
    if artifact["human_review_required"] is not True:
        raise ValueError("human_review_required must be true")
    if artifact["review_kind"] not in {"pull_request", "repository_audit"}:
        raise ValueError("review_kind must be pull_request or repository_audit")
    if artifact["result"] not in {"pass", "findings", "incomplete"}:
        raise ValueError("result must be pass, findings, or incomplete")
    if artifact["risk_level"] not in {"low", "medium", "high", "critical"}:
        raise ValueError("risk_level must be low, medium, high, or critical")
    if not isinstance(artifact["repository"], str) or not artifact["repository"]:
        raise ValueError("repository must be a non-empty string")
    for key in ("reviewed_files", "omitted_files"):
        value = artifact[key]
        if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
            raise ValueError(f"{key} must be a list of strings")
    for key in (
        "findings",
        "new_findings",
        "still_present_findings",
        "changed_findings",
        "resolved_findings",
    ):
        value = artifact[key]
        if not isinstance(value, list):
            raise ValueError(f"{key} must be a list")
        normalized = []
        for item in value:
            finding = Finding.from_dict(item)
            normalized.append(finding.to_dict())
        artifact[key] = normalized
    return artifact


def _delta_arrays(review: AIReview) -> dict[str, list[dict[str, Any]]]:
    buckets: dict[str, list[dict[str, Any]]] = {
        "new": [],
        "still_present": [],
        "changed": [],
        "resolved": [],
    }
    for finding in review.findings:
        buckets[finding.state.value].append(finding.to_dict())
    for finding in review.resolved_findings:
        buckets["resolved"].append(finding.to_dict())
    return buckets


def _safe_line_map(line_map: dict[str, set[int]]) -> dict[str, list[int]]:
    return {path: sorted(lines) for path, lines in line_map.items()}


def _artifact_result(review: AIReview) -> str:
    if review.result is ReviewResult.REVIEW_INCOMPLETE:
        return "incomplete"
    if review.findings or review.resolved_findings:
        return "findings"
    return "pass"


def _artifact_blocks(artifact: dict[str, Any]) -> bool:
    return artifact["result"] == "incomplete" or any(
        bool(finding["blocking"])
        for finding in artifact.get("findings", [])
        if isinstance(finding, dict)
    )


def _prompt_metadata(prompt_path: Path, prompt_version: str) -> dict[str, Any]:
    resolved = prompt_path.resolve()
    prompt_text = resolved.read_text(encoding="utf-8")
    return {
        "path": str(resolved.relative_to(_REPO_ROOT)),
        "version": prompt_version,
        "sha256": _sha256_text(prompt_text),
    }


def _policy_metadata(context: ReviewContext) -> dict[str, Any]:
    return {
        "version": _POLICY_VERSION,
        "repository_policy_sha256": _sha256_text(context.repository_policy),
        "target_policy_sha256": _sha256_text(context.target_policy),
        "confirmed_plan_sha256": _sha256_text(context.confirmed_plan),
        "task_specification_sha256": _sha256_text(context.task_specification),
    }


def _model_metadata(provider_name: str) -> dict[str, Any]:
    return {
        "provider": provider_name,
        "provider_schema_version": "provider-review-output-v1",
    }


def _review_identity(
    repository: str,
    pull_request: int | None,
    base_sha: str,
    head_sha: str,
    prompt_version: str,
    policy_version: str,
) -> str:
    raw = "\0".join(
        (
            repository,
            "" if pull_request is None else str(pull_request),
            base_sha,
            head_sha,
            prompt_version,
            policy_version,
        )
    )
    return hashlib.sha256(raw.encode("utf-8")).hexdigest()


def _encode_final_state(final_artifact: dict[str, Any]) -> str:
    compact = {
        key: final_artifact[key]
        for key in (
            "review_identity",
            "repository",
            "pull_request",
            "base_sha",
            "head_sha",
            "previous_head_sha",
            "prompt_version",
            "policy_version",
            "model",
            "reviewed_at",
            "result",
            "risk_level",
            "complete",
            "reviewed_files",
            "omitted_files",
            "diff_digest",
            "findings",
            "new_findings",
            "still_present_findings",
            "changed_findings",
            "resolved_findings",
            "human_review_required",
        )
    }
    raw = json.dumps(compact, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    encoded = base64.urlsafe_b64encode(raw).decode().rstrip("=")
    return f"{_FINAL_STATE_PREFIX}{encoded}{_FINAL_STATE_SUFFIX}"


def _decode_final_state(comment: str) -> dict[str, Any] | None:
    start = comment.find(_FINAL_STATE_PREFIX)
    if start < 0:
        return None
    start += len(_FINAL_STATE_PREFIX)
    end = comment.find(_FINAL_STATE_SUFFIX, start)
    if end < 0:
        return None
    encoded = comment[start:end]
    try:
        padding = "=" * (-len(encoded) % 4)
        return cast(dict[str, Any], json.loads(base64.urlsafe_b64decode(encoded + padding)))
    except (ValueError, TypeError, json.JSONDecodeError):
        return None


def _build_final_artifact(
    *,
    review: AIReview,
    diff: DiffBundle,
    event: PullRequestEvent | None,
    context: ReviewContext,
    line_map: dict[str, set[int]],
    artifact_name: str,
    current_head: bool,
    prompt_path: Path,
    prompt_version: str,
) -> dict[str, Any]:
    deltas = _delta_arrays(review)
    safe_line_map = _safe_line_map(line_map)
    ci_snapshot = review.test_evidence.get("github_checks_at_review_start", {})
    policy = _policy_metadata(context)
    result = _artifact_result(review)
    artifact_link = (
        f"{os.environ.get('GITHUB_SERVER_URL', '').rstrip('/')}/"
        f"{os.environ.get('GITHUB_REPOSITORY', review.repository)}/actions/runs/"
        f"{os.environ.get('GITHUB_RUN_ID', '')}#artifacts"
        if os.environ.get("GITHUB_RUN_ID")
        else str((Path.cwd() / _PUBLISH_FILE).resolve())
    )
    incomplete = {"reason": review.summary} if result == "incomplete" else None
    review_identity = _review_identity(
        review.repository,
        None if event is None else event.number,
        review.base_sha,
        review.head_sha,
        prompt_version,
        str(policy["version"]),
    )
    artifact = {
        "schema_version": "1",
        "review_kind": "repository_audit" if event is None else "pull_request",
        "repository": review.repository,
        "pull_request": None if event is None else event.number,
        "base_sha": review.base_sha,
        "head_sha": review.head_sha,
        "previous_head_sha": review.previous_head_sha or None,
        "prompt_version": prompt_version,
        "policy_version": str(policy["version"]),
        "model": review.review_provider,
        "reviewed_at": review.reviewed_at.astimezone(UTC).isoformat().replace("+00:00", "Z"),
        "result": result,
        "risk_level": review.risk_level,
        "complete": bool(review.diff_complete and current_head and result != "incomplete"),
        "reviewed_files": list(diff.changed_paths),
        "omitted_files": [] if review.diff_complete else list(diff.changed_paths),
        "diff_digest": diff.full_diff_sha256,
        "findings": [finding.to_dict() for finding in review.findings],
        "new_findings": deltas["new"],
        "still_present_findings": deltas["still_present"],
        "changed_findings": deltas["changed"],
        "resolved_findings": deltas["resolved"],
        "human_review_required": True,
        "review_identity": review_identity,
        "pass": review.result is ReviewResult.PASS,
        "incomplete": incomplete,
        "prompt": _prompt_metadata(prompt_path, prompt_version),
        "policy": policy,
        "model_details": _model_metadata(review.review_provider),
        "review_state": {
            "pull_request": None if event is None else event.number,
            "base_ref": None if event is None else event.base_ref,
            "head_ref": None if event is None else event.head_ref,
            "required_reviewer_group": review.required_reviewer_group,
            "diff": review.diff_complete,
            "current_head": current_head,
        },
        "reviewed": {
            "file_count": len(diff.files),
            "chunk_count": len(diff.chunks),
            "finding_count": len(review.findings),
            "resolved_finding_count": len(review.resolved_findings),
            "blocking_finding_count": sum(1 for finding in review.findings if finding.blocking),
            "delta_counts": {key: len(value) for key, value in deltas.items()},
            "ci_snapshot": ci_snapshot if isinstance(ci_snapshot, dict) else {},
        },
        "omitted": {
            "paths": [] if review.diff_complete else list(review.all_file_paths),
            "reasons": [] if review.diff_complete else [review.summary],
        },
        "digest": {
            "full_diff_sha256": diff.full_diff_sha256,
            "artifact_sha256": "",
        },
        "artifact": {
            "name": artifact_name,
            "path": _PUBLISH_FILE,
            "changed_new_lines": safe_line_map,
            "url": artifact_link,
        },
        "summary": review.summary,
    }
    serialized = json.dumps(artifact, sort_keys=True, ensure_ascii=False).encode("utf-8")
    digest = cast(dict[str, str], artifact["digest"])
    digest["artifact_sha256"] = _sha256_bytes(serialized)
    return _validated_final_artifact(artifact)


def _render_summary(final_artifact: dict[str, Any]) -> str:
    reviewed = final_artifact["reviewed"]
    counts = reviewed["delta_counts"]
    finding_lines = []
    blocking = [finding for finding in final_artifact["findings"] if bool(finding["blocking"])]
    non_blocking = [
        finding for finding in final_artifact["findings"] if not bool(finding["blocking"])
    ]
    for heading, findings in (
        ("Blocking Findings", blocking),
        ("Non-Blocking Findings", non_blocking),
    ):
        if not findings:
            continue
        finding_lines.append(f"### {heading}\n")
        for index, finding in enumerate(findings, start=1):
            location = finding["path"]
            if finding["start_line"] is not None:
                location += f":{finding['start_line']}"
            finding_lines.extend(
                (
                    f"{index}. **[{finding['severity'].title()}] {finding['title']}**",
                    f"   - Location: `{location}`",
                    f"   - Confidence: {finding['confidence']}",
                    f"   - Evidence: {finding['evidence']}",
                    f"   - Recommendation: {finding['recommendation']}",
                )
            )
        finding_lines.append("")
    ci_snapshot = final_artifact["reviewed"]["ci_snapshot"]
    delta_sections: list[str] = []
    for label, key in (
        ("New Findings", "new_findings"),
        ("Still Present Findings", "still_present_findings"),
        ("Changed Findings", "changed_findings"),
        ("Resolved Findings", "resolved_findings"),
    ):
        values = final_artifact[key]
        if not values:
            continue
        delta_sections.extend((f"### {label}", ""))
        for index, finding in enumerate(values, start=1):
            delta_sections.append(
                f"{index}. `{finding['stable_identifier']}` "
                f"{finding['severity']} {finding['title']}"
            )
        delta_sections.append("")
    return "\n".join(
        [
            SUMMARY_MARKER,
            _encode_final_state(final_artifact),
            "## Codex PR Review",
            "",
            f"**Review Identity:** `{final_artifact['review_identity']}`  ",
            f"**Model:** `{final_artifact['model']}`  ",
            (
                f"**Prompt / Policy:** `{final_artifact['prompt_version']}` / "
                f"`{final_artifact['policy_version']}`  "
            ),
            f"**Reviewed:** `{final_artifact['base_sha']}...{final_artifact['head_sha']}`  ",
            f"**Result:** `{final_artifact['result']}`  ",
            f"**Risk:** `{final_artifact['risk_level']}`  ",
            f"**Complete:** `{final_artifact['complete']}`  ",
            (
                f"**Counts:** files={reviewed['file_count']}, chunks={reviewed['chunk_count']}, "
                f"findings={reviewed['finding_count']}, "
                f"blocking={reviewed['blocking_finding_count']}"
            ),
            (
                f"**Delta:** new={counts['new']}, still_present={counts['still_present']}, "
                f"changed={counts['changed']}, resolved={counts['resolved']}"
            ),
            (
                f"**Artifact:** `{final_artifact['artifact']['name']}/"
                f"{final_artifact['artifact']['path']}`"
            ),
            f"**Artifact Link:** {final_artifact['artifact']['url']}",
            f"**Current CI Snapshot:** `{json.dumps(ci_snapshot, sort_keys=True)}`",
            f"**Reviewed Files:** `{json.dumps(final_artifact['reviewed_files'])}`",
            "",
            *finding_lines,
            *delta_sections,
            "### Human Review Required",
            "",
            (
                "**Codex review is evidence, not approval.** Human review is still required, "
                "critical/high findings remain blocking, and this review never approves, merges, "
                "or completes the student learning loop."
            ),
            "",
        ]
    )


def _write_final_artifacts(directory: Path, final_artifact: dict[str, Any]) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    pull_request = final_artifact["pull_request"]
    number = "repository" if pull_request is None else str(pull_request)
    head_sha = final_artifact["head_sha"]
    write_private_text(
        directory / f"pr-{number}-{head_sha}.json",
        _json_dump(final_artifact),
    )
    write_private_text(
        directory / f"pr-{number}-{head_sha}.md",
        _render_summary(final_artifact),
    )
    return write_private_text(directory / _PUBLISH_FILE, _json_dump(final_artifact))


def _normalize_pull_request_event(
    api: GitHubApi,
    event_payload: dict[str, Any],
    *,
    pull_request: int | None,
) -> PullRequestEvent:
    if isinstance(event_payload.get("pull_request"), dict):
        return PullRequestEvent.from_dict(event_payload)
    repository = event_payload.get("repository")
    if not isinstance(repository, dict) or not isinstance(repository.get("full_name"), str):
        raise ValueError("workflow event does not contain repository.full_name")
    repository_name = str(repository["full_name"])
    workflow_run = event_payload.get("workflow_run")
    if workflow_run is not None:
        if not isinstance(workflow_run, dict):
            raise ValueError("workflow_run event payload must be an object")
        if str(workflow_run.get("name", "")) != "Phase 0 Validation":
            raise ValueError("unexpected source workflow for AI review")
        if str(workflow_run.get("conclusion", "")) != "success":
            raise ValueError("source workflow did not succeed")
        head_repository = workflow_run.get("head_repository")
        if not isinstance(head_repository, dict) or not isinstance(
            head_repository.get("full_name"), str
        ):
            raise ValueError("workflow_run.head_repository.full_name is required")
        head_repository_name = str(head_repository["full_name"])
        head_sha = str(workflow_run.get("head_sha", ""))
        number = pull_request
        if number is None:
            pull_requests = workflow_run.get("pull_requests")
            candidates = []
            if isinstance(pull_requests, list):
                candidates = [
                    candidate
                    for candidate in pull_requests
                    if isinstance(candidate, dict) and isinstance(candidate.get("number"), int)
                ]
            if len(candidates) == 1:
                number = int(candidates[0]["number"])
            elif len(candidates) == 0:
                associated = api.get_associated_pull_requests(head_repository_name, head_sha)
                filtered = [
                    candidate
                    for candidate in associated
                    if str(candidate.get("state", "")) == "open"
                    and isinstance(candidate.get("number"), int)
                    and isinstance(candidate.get("base"), dict)
                    and isinstance(candidate["base"].get("repo"), dict)
                    and candidate["base"]["repo"].get("full_name") == repository_name
                ]
                if len(filtered) != 1:
                    raise ValueError(
                        "workflow_run head commit did not resolve to exactly one open pull request"
                    )
                number = int(filtered[0]["number"])
            else:
                raise ValueError(
                    "workflow_run referenced multiple pull requests; resolution is ambiguous"
                )
    else:
        number = pull_request
    if number is None:
        raise ValueError("workflow event does not identify a pull request")
    raw_pull_request = api.get_pull_request(repository_name, number)
    if workflow_run is not None:
        head_sha = str(workflow_run.get("head_sha", ""))
        if str(raw_pull_request.get("head", {}).get("sha", "")) != head_sha:
            raise ValueError("resolved pull request head does not match workflow_run head_sha")
        if (
            str(raw_pull_request.get("base", {}).get("repo", {}).get("full_name", ""))
            != repository_name
        ):
            raise ValueError(
                "resolved pull request base repository does not match workflow repository"
            )
    return PullRequestEvent.from_dict(
        {
            "action": "synchronize",
            "number": number,
            "repository": {"full_name": repository_name},
            "pull_request": {
                "title": raw_pull_request.get("title", ""),
                "body": raw_pull_request.get("body") or "",
                "base": raw_pull_request.get("base"),
                "head": raw_pull_request.get("head"),
            },
        }
    )


def _git_fetch(repository: Path, *args: str) -> None:
    completed = subprocess.run(
        ("git", *args),
        cwd=repository,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise DiffCollectionError(completed.stderr.strip() or f"git {' '.join(args)} failed")


def _prepare_diff(repository: Path, event: PullRequestEvent) -> DiffBundle:
    try:
        _git_fetch(
            repository,
            "fetch",
            "--no-tags",
            "origin",
            event.base_sha,
            event.head_sha,
        )
    except DiffCollectionError:
        _git_fetch(
            repository,
            "fetch",
            "--no-tags",
            "origin",
            event.base_sha,
            f"pull/{event.number}/head",
        )
    return collect_full_diff(
        repository,
        base_sha=event.base_sha,
        head_sha=event.head_sha,
        maximum_chunk_characters=6_000,
    )


def _prepared_payload(
    *,
    event: PullRequestEvent,
    diff: DiffBundle,
    context: ReviewContext,
    previous: AIReview | None,
    required_reviewer_group: str,
) -> dict[str, Any]:
    return {
        "event": {
            "action": event.action,
            "number": event.number,
            "repository": event.repository,
            "base_repository": event.base_repository,
            "head_repository": event.head_repository,
            "base_sha": event.base_sha,
            "head_sha": event.head_sha,
            "base_ref": event.base_ref,
            "head_ref": event.head_ref,
            "title": event.title,
            "body": event.body,
        },
        "diff": diff.to_dict(),
        "context": context.to_dict(),
        "previous_review": None if previous is None else previous.to_dict(),
        "required_reviewer_group": required_reviewer_group,
    }


def _metadata_from_event(event: PullRequestEvent) -> PullRequestMetadata:
    return PullRequestMetadata(
        number=event.number,
        repository=event.repository,
        title=event.title,
        body=event.body,
    )


def _state_to_previous_review(state: dict[str, Any]) -> AIReview:
    previous_head = state.get("previous_head_sha")
    return AIReview.from_dict(
        {
            "pull_request": 1 if state["pull_request"] is None else state["pull_request"],
            "repository": state["repository"],
            "base_sha": state["base_sha"],
            "head_sha": state["head_sha"],
            "reviewed_at": state["reviewed_at"],
            "result": "review_incomplete"
            if state["result"] == "incomplete"
            else "pass"
            if state["result"] == "pass"
            else "changes_requested"
            if any(item["blocking"] for item in state["findings"])
            else "pass_with_suggestions",
            "summary": "Prior Codex review state.",
            "risk_level": state["risk_level"],
            "findings": state["findings"],
            "resolved_findings": state["resolved_findings"],
            "test_evidence": {},
            "learning_evidence": {},
            "required_reviewer_group": "current Review Policy eligible reviewer",
            "diff_complete": bool(state["complete"]),
            "analysed_file_paths": state["reviewed_files"],
            "all_file_paths": state["reviewed_files"],
            "review_provider": state["model"] or "codex-action",
            "previous_head_sha": "" if previous_head is None else previous_head,
        }
    )


def prepare_workflow_review(args: argparse.Namespace) -> int:
    token = _github_token()
    api = GitHubApi(
        token=token,
        api_url=os.environ.get("GITHUB_API_URL", "https://api.github.com"),
        actor_login=os.environ.get("AI_REVIEW_COMMENT_ACTOR", ""),
    )
    payload = json.loads(args.event.read_text(encoding="utf-8"))
    event = _normalize_pull_request_event(api, payload, pull_request=args.pull_request)
    event_metadata = {
        "pull_request": event.number,
        "base_sha": event.base_sha,
        "head_sha": event.head_sha,
        "prompt_version": _PR_PROMPT_VERSION,
        "policy_version": _POLICY_VERSION,
    }
    emit_service_event(
        EventType.CODEX_REVIEW_REQUESTED,
        repository=event.repository,
        result="requested",
        metadata=event_metadata,
        branch=event.head_ref,
    )
    emit_service_event(
        EventType.CODEX_REVIEW_STARTED,
        repository=event.repository,
        result="started",
        metadata=event_metadata,
        branch=event.head_ref,
    )
    emit_service_event(
        EventType.AUTO_REVIEW_STARTED,
        repository=event.repository,
        result="started",
        metadata=event_metadata,
        branch=event.head_ref,
    )
    emit_service_event(
        EventType.CODEX_REVIEW_BASE_HEAD_RECORDED,
        repository=event.repository,
        result="recorded",
        metadata=event_metadata,
        branch=event.head_ref,
    )
    repository = args.repo.resolve()
    previous: AIReview | None = None
    previous_state: dict[str, Any] | None = None
    try:
        existing = api.get_summary_comment(event.repository, event.number)
    except GitHubApiError:
        existing = None
    if existing is not None:
        previous_state = _decode_final_state(str(existing.get("body", "")))
        if previous_state is not None:
            previous = _state_to_previous_review(previous_state)
    ci_status = api.get_ci_status(event.repository, event.head_sha)
    try:
        diff = _prepare_diff(repository, event)
    except DiffCollectionError as error:
        diff = DiffBundle(
            repository=str(repository),
            base_sha=event.base_sha,
            head_sha=event.head_sha,
            files=(),
            chunks=(),
            full_diff_sha256=_EMPTY_SHA256,
            full_diff_bytes=0,
            additions=0,
            deletions=0,
            commit_messages=(),
            complete=False,
            incomplete_reason=f"full diff unavailable: {error}",
        )
    assessment = _risk_assessment(diff, args.policy_root.resolve())
    emit_service_event(
        EventType.CODEX_REVIEW_DIFF_READY,
        repository=event.repository,
        result="complete" if diff.complete else "incomplete",
        metadata={
            **event_metadata,
            "file_count": len(diff.files),
            "chunk_count": len(diff.chunks),
            "diff_digest_sha256": diff.full_diff_sha256,
        },
        branch=event.head_ref,
    )
    context = ReviewContext(
        repository_policy=_trusted_policy(args.policy_root.resolve()),
        target_policy=_target_policy(repository, event.base_sha),
        task_specification=_read_optional(args.task_specification)
        or "Harness task specification was not supplied.",
        confirmed_plan=_read_optional(args.confirmed_plan)
        or "Confirmed implementation plan was not supplied.",
        risk_classification=assessment.risk.label,
        protected_path_result=json.dumps(assessment.to_dict(), sort_keys=True),
        learning_review_status=str(
            _load_json(args.learning_evidence).get("status", "not supplied")
        ),
        test_evidence={
            "github_checks_at_review_start": ci_status,
            "harness": _load_json(args.test_evidence),
        },
    )
    review_identity = _review_identity(
        event.repository,
        event.number,
        event.base_sha,
        event.head_sha,
        _PR_PROMPT_VERSION,
        _POLICY_VERSION,
    )
    write_private_text(
        args.prepared.resolve(),
        _json_dump(
            {
                **_prepared_payload(
                    event=event,
                    diff=diff,
                    context=context,
                    previous=previous,
                    required_reviewer_group=_reviewer_group(assessment),
                ),
                "review_identity": review_identity,
                "prompt_version": _PR_PROMPT_VERSION,
                "policy_version": _POLICY_VERSION,
                "skip_existing_review": (
                    previous_state
                    if previous_state is not None
                    and previous_state.get("review_identity") == review_identity
                    and previous_state.get("complete") is True
                    else None
                ),
            }
        ),
    )
    request_path, output_path = _runtime_paths(repository)
    output_path.unlink(missing_ok=True)
    if (
        previous_state is not None
        and previous_state.get("review_identity") == review_identity
        and previous_state.get("complete") is True
    ):
        emit_service_event(
            EventType.CODEX_REVIEW_DUPLICATE_DETECTED,
            repository=event.repository,
            result="current complete review exists",
            metadata={**event_metadata, "review_identity": review_identity},
            branch=event.head_ref,
        )
        emit_service_event(
            EventType.CODEX_REVIEW_SKIPPED,
            repository=event.repository,
            result="duplicate current review",
            metadata={**event_metadata, "review_identity": review_identity},
            branch=event.head_ref,
        )
        request_path.unlink(missing_ok=True)
        return 0
    request = _review_request(_metadata_from_event(event), diff, context)
    gate_findings = deterministic_data_gate(request)
    if gate_findings:
        emit_service_event(
            EventType.CODEX_REVIEW_GATE_BLOCKED,
            repository=event.repository,
            result="blocked",
            metadata={
                **event_metadata,
                "sanitized_categories": list(gate_findings),
            },
            branch=event.head_ref,
        )
        write_private_text(
            output_path,
            _json_dump(
                _placeholder_provider_review(
                    "deterministic outbound data gate blocked provider access after scanning "
                    "trusted and untrusted review context"
                ).to_dict()
            ),
        )
        return 0
    emit_service_event(
        EventType.CODEX_REVIEW_GATE_PASSED,
        repository=event.repository,
        result="passed",
        metadata=event_metadata,
        branch=event.head_ref,
    )
    if not diff.complete:
        write_private_text(
            output_path,
            _json_dump(
                _placeholder_provider_review(
                    diff.incomplete_reason or "full diff unavailable"
                ).to_dict()
            ),
        )
        return 0
    request_document = _provider_request_document(request, prompt_version=_PR_PROMPT_VERSION)
    if len(request_document.encode("utf-8")) > _positive_budget(
        "CODEX_REVIEW_MAX_INPUT_BYTES", _DEFAULT_MAX_INPUT_BYTES
    ):
        write_private_text(
            output_path,
            _json_dump(
                _placeholder_provider_review(
                    "provider input exceeded the configured outbound budget"
                ).to_dict()
            ),
        )
        return 0
    request_path.parent.mkdir(parents=True, exist_ok=True)
    write_private_text(request_path, request_document)
    return 0


def finalize_workflow_review(args: argparse.Namespace) -> int:
    prepared = json.loads(args.prepared.read_text(encoding="utf-8"))
    if isinstance(prepared.get("skip_existing_review"), dict):
        final_artifact = _validated_final_artifact(prepared["skip_existing_review"])
        bundle = _write_final_artifacts(args.output.resolve(), final_artifact)
        _emit_normalized_review_events(
            final_artifact,
            branch=str(prepared["event"]["head_ref"]),
            metadata={"reused_existing_review": True},
        )
        print(bundle)
        return 0
    event = PullRequestEvent.from_dict(
        {
            "action": prepared["event"]["action"],
            "number": prepared["event"]["number"],
            "repository": {"full_name": prepared["event"]["repository"]},
            "pull_request": {
                "title": prepared["event"]["title"],
                "body": prepared["event"]["body"],
                "base": {
                    "sha": prepared["event"]["base_sha"],
                    "ref": prepared["event"]["base_ref"],
                    "repo": {"full_name": prepared["event"]["base_repository"]},
                },
                "head": {
                    "sha": prepared["event"]["head_sha"],
                    "ref": prepared["event"]["head_ref"],
                    "repo": {"full_name": prepared["event"]["head_repository"]},
                },
            },
        }
    )
    diff = collect_full_diff(
        args.repo.resolve(),
        base_sha=event.base_sha,
        head_sha=event.head_sha,
        maximum_chunk_characters=6_000,
    )
    context = ReviewContext(**prepared["context"])
    previous = (
        None
        if prepared["previous_review"] is None
        else AIReview.from_dict(prepared["previous_review"])
    )
    provider_review = _provider_output_or_incomplete(
        args.provider_output.resolve(),
        reason="provider was unavailable, refused, exceeded budget, or produced no output",
    )
    emit_service_event(
        (
            EventType.CODEX_REVIEW_REQUEST_FAILED
            if provider_review.result is ReviewResult.REVIEW_INCOMPLETE
            else EventType.CODEX_REVIEW_REQUEST_COMPLETED
        ),
        repository=event.repository,
        result=(
            "incomplete"
            if provider_review.result is ReviewResult.REVIEW_INCOMPLETE
            else "completed"
        ),
        metadata={
            "pull_request": event.number,
            "base_sha": event.base_sha,
            "head_sha": event.head_sha,
        },
        branch=event.head_ref,
    )
    review = assemble_codex_review(
        _metadata_from_event(event),
        diff,
        context,
        provider_review,
        previous=previous,
        required_reviewer_group=str(prepared["required_reviewer_group"]),
        review_provider="codex-action",
    )
    artifact_name = os.environ.get("AI_REVIEW_ARTIFACT_NAME", "codex-ai-review")
    final_artifact = _build_final_artifact(
        review=review,
        diff=diff,
        event=event,
        context=context,
        line_map=changed_new_lines(diff),
        artifact_name=artifact_name,
        current_head=True,
        prompt_path=_PR_PROMPT_PATH,
        prompt_version=str(prepared["prompt_version"]),
    )
    bundle = _write_final_artifacts(args.output.resolve(), final_artifact)
    _emit_normalized_review_events(final_artifact, branch=event.head_ref)
    _emit_completed_review_events(final_artifact, branch=event.head_ref)
    print(bundle)
    return 0


def publish_workflow_review(args: argparse.Namespace) -> int:
    token = _github_token()
    final_artifact = _validated_final_artifact(json.loads(args.bundle.read_text(encoding="utf-8")))
    pull_request = final_artifact["pull_request"]
    if pull_request is None:
        raise ValueError("workflow publication requires a pull request artifact")
    api = GitHubApi(
        token=token,
        api_url=os.environ.get("GITHUB_API_URL", "https://api.github.com"),
        actor_login=os.environ.get("AI_REVIEW_COMMENT_ACTOR", ""),
    )
    repository = str(final_artifact["repository"])
    number = int(pull_request)
    current = api.get_pull_request(repository, number)
    current_head = str(current.get("head", {}).get("sha", ""))
    if current_head != str(final_artifact["head_sha"]):
        print("AI review did not produce a current non-blocking result.", file=sys.stderr)
        return 1
    review = AIReview.from_dict(
        {
            "pull_request": number,
            "repository": repository,
            "base_sha": final_artifact["base_sha"],
            "head_sha": final_artifact["head_sha"],
            "reviewed_at": final_artifact["reviewed_at"],
            "result": "review_incomplete"
            if final_artifact["result"] == "incomplete"
            else (
                "pass"
                if not final_artifact["findings"]
                else "changes_requested"
                if any(finding["blocking"] for finding in final_artifact["findings"])
                else "pass_with_suggestions"
            ),
            "summary": final_artifact["summary"],
            "risk_level": final_artifact["risk_level"],
            "findings": final_artifact["findings"],
            "resolved_findings": final_artifact["resolved_findings"],
            "test_evidence": {
                "github_checks_at_review_start": final_artifact["reviewed"]["ci_snapshot"]
            },
            "learning_evidence": {},
            "required_reviewer_group": str(
                final_artifact["review_state"]["required_reviewer_group"]
            ),
            "diff_complete": bool(final_artifact["review_state"]["diff"]),
            "analysed_file_paths": list(final_artifact["reviewed_files"]),
            "all_file_paths": list(final_artifact["reviewed_files"]),
            "review_provider": final_artifact["model"] or "codex-action",
            "previous_head_sha": ""
            if final_artifact["previous_head_sha"] is None
            else final_artifact["previous_head_sha"],
        }
    )
    summary = _render_summary(final_artifact)
    api.upsert_summary_comment(repository, number, summary)
    emit_service_event(
        EventType.CODEX_STABLE_COMMENT_UPDATED,
        repository=repository,
        result="updated",
        metadata={
            "pull_request": number,
            "head_sha": review.head_sha,
            "review_identity": final_artifact["review_identity"],
        },
    )
    line_map = {
        path: {int(value) for value in lines}
        for path, lines in dict(final_artifact["artifact"]["changed_new_lines"]).items()
    }
    api.post_inline_findings(repository, number, review.head_sha, review.findings, line_map)
    api.create_check(repository, review, summary)
    emit_service_event(
        EventType.REVIEW_PUBLISHED,
        repository=repository,
        result="published",
        metadata={
            "pull_request": number,
            "base_sha": review.base_sha,
            "head_sha": review.head_sha,
            "review_identity": final_artifact["review_identity"],
            "review_result": final_artifact["result"],
            "complete": final_artifact["complete"],
            "blocking": _artifact_blocks(final_artifact),
            "check_conclusion": review.check_conclusion.value,
            "summary_comment_updated": True,
            "pull_request_head_check_created": True,
            "artifact_name": final_artifact["artifact"]["name"],
            "artifact_path": final_artifact["artifact"]["path"],
        },
        branch=str(final_artifact["review_state"]["head_ref"] or ""),
    )
    return 1 if _artifact_blocks(final_artifact) else 0


def _emit_normalized_review_events(
    final_artifact: dict[str, Any],
    *,
    branch: str = "",
    metadata: dict[str, Any] | None = None,
) -> None:
    repository = str(final_artifact["repository"])
    common = {
        "pull_request": final_artifact["pull_request"],
        "base_sha": final_artifact["base_sha"],
        "head_sha": final_artifact["head_sha"],
        "review_identity": final_artifact["review_identity"],
        "review_result": final_artifact["result"],
        "complete": final_artifact["complete"],
    }
    if metadata:
        common.update(metadata)
    emit_service_event(
        EventType.AUTO_REVIEW_COMPLETED,
        repository=repository,
        result="completed",
        metadata=common,
        branch=branch,
    )
    for finding in final_artifact["findings"]:
        emit_service_event(
            EventType.REVIEW_FINDING_CREATED,
            repository=repository,
            result="created",
            metadata={
                **common,
                "finding_id": finding["stable_identifier"],
                "severity": finding["severity"],
                "path": finding["path"],
                "blocking": finding["blocking"],
                "safety_review": finding["safety_review"],
                "state": finding["state"],
            },
            branch=branch,
        )


def _emit_completed_review_events(final_artifact: dict[str, Any], *, branch: str = "") -> None:
    repository = str(final_artifact["repository"])
    common = {
        "pull_request": final_artifact["pull_request"],
        "base_sha": final_artifact["base_sha"],
        "head_sha": final_artifact["head_sha"],
        "review_identity": final_artifact["review_identity"],
        "result": final_artifact["result"],
        "complete": final_artifact["complete"],
    }
    emit_service_event(
        EventType.CODEX_REVIEW_COMPLETED,
        repository=repository,
        result=str(final_artifact["result"]),
        metadata=common,
        branch=branch,
    )
    buckets = (
        ("new_findings", EventType.CODEX_FINDING_NEW),
        ("still_present_findings", EventType.CODEX_FINDING_STILL_PRESENT),
        ("changed_findings", EventType.CODEX_FINDING_CHANGED),
        ("resolved_findings", EventType.CODEX_FINDING_RESOLVED),
    )
    for key, event_type in buckets:
        for finding in final_artifact[key]:
            emit_service_event(
                event_type,
                repository=repository,
                result=str(finding["severity"]),
                metadata={
                    **common,
                    "finding_id": finding["stable_identifier"],
                    "severity": finding["severity"],
                    "path": finding["path"],
                    "blocking": finding["blocking"],
                    "safety_review": finding["safety_review"],
                },
                branch=branch,
            )


def _run_codex_exec(
    request: ReviewRequest,
    *,
    runtime_root: Path,
    prompt_path: Path,
    prompt_version: str,
) -> ProviderReview:
    gate_findings = deterministic_data_gate(request)
    if gate_findings:
        return _placeholder_provider_review(
            "deterministic outbound data gate blocked provider access after scanning "
            "trusted and untrusted review context"
        )
    request_path, output_path = _runtime_paths(runtime_root)
    request_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.unlink(missing_ok=True)
    request_document = _provider_request_document(request, prompt_version=prompt_version)
    if len(request_document.encode("utf-8")) > _positive_budget(
        "CODEX_REVIEW_MAX_INPUT_BYTES", _DEFAULT_MAX_INPUT_BYTES
    ):
        return _placeholder_provider_review(
            "provider input exceeded the configured outbound budget"
        )
    write_private_text(request_path, request_document)
    prompt = prompt_path.read_text(encoding="utf-8")
    try:
        completed = subprocess.run(
            (
                "codex",
                "exec",
                "-",
                "--sandbox",
                "read-only",
                "--output-schema",
                str(_PROVIDER_SCHEMA_PATH),
                "-o",
                str(output_path),
                "--color",
                "never",
            ),
            cwd=runtime_root,
            input=prompt,
            text=True,
            check=False,
            capture_output=True,
            timeout=float(
                _positive_budget("CODEX_REVIEW_TIMEOUT_SECONDS", int(_DEFAULT_TIMEOUT_SECONDS))
            ),
        )
    except subprocess.TimeoutExpired:
        return _placeholder_provider_review("provider timed out")
    if completed.returncode != 0:
        stderr = completed.stderr.strip().lower()
        if "refus" in stderr:
            return _placeholder_provider_review("provider refused the review request")
        if "budget" in stderr or "token" in stderr:
            return _placeholder_provider_review("provider exceeded its configured budget")
        return _placeholder_provider_review(
            "provider was unavailable or execution failed before structured output"
        )
    return _provider_output_or_incomplete(
        output_path,
        reason="provider produced no structured output",
    )


def _local_context(repository: Path, base_sha: str) -> ReviewContext:
    return ReviewContext(
        repository_policy=_trusted_policy(_REPO_ROOT),
        target_policy=_target_policy(repository, base_sha),
        task_specification="Local Codex review invocation.",
        confirmed_plan="Review only the supplied immutable diff.",
        risk_classification="medium",
        protected_path_result="not evaluated",
        learning_review_status="not supplied",
        test_evidence={},
    )


def _event_from_pull_request(
    repository_name: str, pull_request: int, raw_pull_request: dict[str, Any]
) -> PullRequestEvent:
    return PullRequestEvent.from_dict(
        {
            "action": "synchronize",
            "number": pull_request,
            "repository": {"full_name": repository_name},
            "pull_request": {
                "title": raw_pull_request.get("title", ""),
                "body": raw_pull_request.get("body") or "",
                "base": raw_pull_request.get("base"),
                "head": raw_pull_request.get("head"),
            },
        }
    )


def _run_local_review(
    *,
    repository: Path,
    metadata: PullRequestMetadata,
    diff: DiffBundle,
    context: ReviewContext,
    event: PullRequestEvent | None,
    output: Path,
    current_head: bool,
) -> tuple[dict[str, Any], AIReview]:
    provider_review = _run_codex_exec(
        _review_request(metadata, diff, context),
        runtime_root=_REPO_ROOT,
        prompt_path=_PR_PROMPT_PATH if event is not None else _REPOSITORY_PROMPT_PATH,
        prompt_version=_PR_PROMPT_VERSION if event is not None else _REPOSITORY_PROMPT_VERSION,
    )
    review = assemble_codex_review(
        metadata,
        diff,
        context,
        provider_review,
        previous=None,
        required_reviewer_group="current Review Policy eligible reviewer",
        review_provider="codex-cli",
    )
    final_artifact = _build_final_artifact(
        review=review,
        diff=diff,
        event=event,
        context=context,
        line_map=changed_new_lines(diff) if diff.complete else {},
        artifact_name="local-codex-review",
        current_head=current_head,
        prompt_path=_PR_PROMPT_PATH if event is not None else _REPOSITORY_PROMPT_PATH,
        prompt_version=_PR_PROMPT_VERSION if event is not None else _REPOSITORY_PROMPT_VERSION,
    )
    _write_final_artifacts(output.parent, final_artifact)
    write_private_text(output, _json_dump(final_artifact))
    _emit_completed_review_events(final_artifact, branch=_current_branch_name(repository))
    return final_artifact, review


def repository_review(args: argparse.Namespace) -> int:
    repository = _git_root(args.repo.resolve())
    repository_name = _origin_repository_identity(repository)
    head_sha = _head_sha(repository)
    base_sha = _merge_base_sha(repository, args.base)
    comparison = collect_full_diff(
        repository,
        base_sha=base_sha,
        head_sha=head_sha,
        maximum_chunk_characters=6_000,
    )
    snapshot = collect_full_diff(
        repository,
        base_sha=_empty_tree_sha(repository),
        head_sha=head_sha,
        maximum_chunk_characters=6_000,
        direct_tree_comparison=True,
    )
    diff = replace(
        snapshot,
        base_sha=base_sha,
        commit_messages=comparison.commit_messages,
    )
    base_context = _local_context(repository, base_sha)
    audit_context = ReviewContext(
        repository_policy=base_context.repository_policy,
        target_policy=base_context.target_policy,
        task_specification=(
            "Manual repository-wide audit of every tracked file at the current head."
        ),
        confirmed_plan=(
            f"Compare the full tracked-tree snapshot at {head_sha} against explicit base "
            f"{base_sha}. Paths changed since that base: "
            + json.dumps(list(comparison.changed_paths), sort_keys=True)
        ),
        risk_classification=base_context.risk_classification,
        protected_path_result=base_context.protected_path_result,
        learning_review_status=base_context.learning_review_status,
        test_evidence=base_context.test_evidence,
    )
    output = (
        args.output.resolve()
        if args.output is not None
        else _default_output_path(repository, "repository", head_sha)
    )
    artifact, _review = _run_local_review(
        repository=repository,
        metadata=PullRequestMetadata(
            number=1,
            repository=repository_name,
            title="Local repository audit",
            body="",
        ),
        diff=diff,
        context=audit_context,
        event=None,
        output=output,
        current_head=True,
    )
    print(output)
    return 1 if _artifact_blocks(artifact) else 0


def pr_review(args: argparse.Namespace) -> int:
    token = _github_token()
    repository = _git_root(args.repo.resolve())
    repository_name = _origin_repository_identity(repository)
    api = GitHubApi(
        token=token,
        api_url=os.environ.get("GITHUB_API_URL", "https://api.github.com"),
    )
    raw_pull_request = api.get_pull_request(repository_name, args.number)
    event = _event_from_pull_request(repository_name, args.number, raw_pull_request)
    diff = _prepare_diff(repository, event)
    output = (
        args.output.resolve()
        if args.output is not None
        else _default_output_path(repository, "pr", event.head_sha, pull_request=args.number)
    )
    context = _local_context(repository, event.base_sha)
    context = ReviewContext(
        repository_policy=context.repository_policy,
        target_policy=context.target_policy,
        task_specification=context.task_specification,
        confirmed_plan=context.confirmed_plan,
        risk_classification=context.risk_classification,
        protected_path_result=context.protected_path_result,
        learning_review_status=context.learning_review_status,
        test_evidence={
            "github_checks_at_review_start": api.get_ci_status(repository_name, event.head_sha)
        },
    )
    artifact, _review = _run_local_review(
        repository=repository,
        metadata=_metadata_from_event(event),
        diff=diff,
        context=context,
        event=event,
        output=output,
        current_head=True,
    )
    if args.post:
        publish_workflow_review(argparse.Namespace(bundle=output))
    print(output)
    return 1 if _artifact_blocks(artifact) else 0


def validate_assets(_args: argparse.Namespace) -> int:
    """Validate trusted prompts, schemas, and outbound policy without a provider call."""

    for schema_path in (_PROVIDER_SCHEMA_PATH, _FINAL_SCHEMA_PATH):
        schema = _read_schema(schema_path)
        if schema.get("type") != "object":
            raise ValueError(f"{schema_path} must define an object schema")
        if schema.get("additionalProperties") is not False:
            raise ValueError(f"{schema_path} must reject additional properties")
        required = schema.get("required")
        properties = schema.get("properties")
        if not isinstance(required, list) or not required:
            raise ValueError(f"{schema_path} must define required properties")
        if not isinstance(properties, dict) or not set(required).issubset(properties):
            raise ValueError(f"{schema_path} required properties must be declared")
    for prompt_path in (_PR_PROMPT_PATH, _REPOSITORY_PROMPT_PATH):
        prompt = prompt_path.read_text(encoding="utf-8")
        required_phrases = ("untrusted", "human review", "schema")
        missing = [phrase for phrase in required_phrases if phrase not in prompt.casefold()]
        if missing:
            raise ValueError(
                f"{prompt_path} is missing required trust language: {', '.join(missing)}"
            )
    outbound_policy = json.loads(_OUTBOUND_POLICY_PATH.read_text(encoding="utf-8"))
    if not isinstance(outbound_policy, dict):
        raise ValueError("outbound data policy must be an object")
    sensitive_paths = outbound_policy.get("sensitive_path_patterns")
    if not isinstance(sensitive_paths, list) or not sensitive_paths:
        raise ValueError("outbound data policy must configure sensitive paths")
    print("Codex review assets valid")
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Prepare and publish trusted Codex pull-request reviews"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("validate")

    prepare = subparsers.add_parser("prepare-workflow")
    prepare.add_argument("--event", type=Path, required=True)
    prepare.add_argument("--repo", type=Path, required=True)
    prepare.add_argument("--policy-root", type=Path, required=True)
    prepare.add_argument("--prepared", type=Path, required=True)
    prepare.add_argument("--pull-request", type=int)
    prepare.add_argument("--task-specification", type=Path)
    prepare.add_argument("--confirmed-plan", type=Path)
    prepare.add_argument("--test-evidence", type=Path)
    prepare.add_argument("--learning-evidence", type=Path)

    finalize = subparsers.add_parser("finalize-workflow")
    finalize.add_argument("--prepared", type=Path, required=True)
    finalize.add_argument("--provider-output", type=Path, required=True)
    finalize.add_argument("--repo", type=Path, required=True)
    finalize.add_argument("--output", type=Path, required=True)

    publish = subparsers.add_parser("publish-workflow")
    publish.add_argument("--bundle", type=Path, required=True)

    repository_cmd = subparsers.add_parser("repository", aliases=["repo"])
    repository_cmd.add_argument("--repo", type=Path, default=Path.cwd())
    repository_cmd.add_argument("--base", default="main")
    repository_cmd.add_argument("--output", type=Path)

    pr_cmd = subparsers.add_parser("pr", aliases=["pull-request"])
    pr_cmd.add_argument("number", type=int)
    pr_cmd.add_argument("--repo", type=Path, default=Path.cwd())
    pr_cmd.add_argument("--output", type=Path)
    pr_cmd.add_argument("--post", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.command == "validate":
        return validate_assets(args)
    if args.command == "prepare-workflow":
        return prepare_workflow_review(args)
    if args.command == "finalize-workflow":
        return finalize_workflow_review(args)
    if args.command == "publish-workflow":
        return publish_workflow_review(args)
    if args.command in {"repository", "repo"}:
        return repository_review(args)
    if args.command in {"pr", "pull-request"}:
        return pr_review(args)
    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
