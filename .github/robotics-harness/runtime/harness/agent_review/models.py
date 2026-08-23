"""Standard-library-only domain models for automated pull-request review state."""

from __future__ import annotations

import base64
import hashlib
import json
import re
from dataclasses import dataclass, field, replace
from datetime import UTC, datetime
from enum import StrEnum
from pathlib import PurePosixPath
from typing import Any

_OBJECT_ID = re.compile(r"^[0-9a-fA-F]{40,64}$")
_REPOSITORY = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$")
_FINDING_ID = re.compile(r"^AR-\d{3}$")
STATE_MARKER_PREFIX = "<!-- robotics-harness-agent-review-state:"
STATE_MARKER_SUFFIX = " -->"


def _require_text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


def _require_text_preserve(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


def _canonical_json(value: object) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _format_timestamp(value: datetime) -> str:
    if value.tzinfo is None:
        raise ValueError("timestamps must include a timezone")
    return value.astimezone(UTC).isoformat().replace("+00:00", "Z")


def _parse_timestamp(value: object, name: str) -> datetime:
    text = _require_text(value, name)
    normalized = text[:-1] + "+00:00" if text.endswith("Z") else text
    parsed = datetime.fromisoformat(normalized)
    if parsed.tzinfo is None:
        raise ValueError(f"{name} must include a timezone")
    return parsed.astimezone(UTC)


def _normalize_path(path: object, name: str = "path") -> str:
    text = _require_text(path, name)
    parsed = PurePosixPath(text)
    if parsed.is_absolute() or ".." in parsed.parts or any(ord(char) < 32 for char in text):
        raise ValueError(f"{name} must be a repository-relative POSIX path")
    return text


class ReviewState(StrEnum):
    NOT_STARTED = "not_started"
    RUNNING = "running"
    PASS = "pass"
    COMMENT = "comment"
    CHANGES_REQUESTED = "changes_requested"
    FAILED = "failed"
    STALE = "stale"


@dataclass(frozen=True, slots=True)
class PullRequestRef:
    repository: str = ""
    number: int = 0
    base_sha: str = ""
    head_sha: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.number, int) or isinstance(self.number, bool) or self.number < 1:
            raise ValueError("pull request number must be a positive integer")
        if self.repository and not _REPOSITORY.fullmatch(self.repository):
            raise ValueError("repository must use owner/name form")
        for name in ("base_sha", "head_sha"):
            value = getattr(self, name)
            if value and not _OBJECT_ID.fullmatch(value):
                raise ValueError(f"{name} must be a full hexadecimal object ID")

    def to_dict(self) -> dict[str, Any]:
        return {
            "repository": self.repository,
            "number": self.number,
            "base_sha": self.base_sha,
            "head_sha": self.head_sha,
        }

    @classmethod
    def from_dict(cls, value: object) -> PullRequestRef:
        if not isinstance(value, dict):
            raise ValueError("pull_request must be an object")
        return cls(
            repository=str(value.get("repository", "")),
            number=int(value.get("number", 0)),
            base_sha=str(value.get("base_sha", "")),
            head_sha=str(value.get("head_sha", "")),
        )


@dataclass(frozen=True, slots=True)
class ReviewRequest:
    raw_text: str
    pull_request: PullRequestRef
    requested_by_session_id: str = ""
    automatic: bool = False

    def __post_init__(self) -> None:
        _require_text(self.raw_text, "raw_text")
        if self.requested_by_session_id:
            _require_text(self.requested_by_session_id, "requested_by_session_id")

    @property
    def repository(self) -> str:
        return self.pull_request.repository

    @property
    def number(self) -> int:
        return self.pull_request.number

    @property
    def head_sha(self) -> str:
        return self.pull_request.head_sha

    def with_pull_request(self, pull_request: PullRequestRef) -> ReviewRequest:
        return replace(self, pull_request=pull_request)

    def to_dict(self) -> dict[str, Any]:
        return {
            "raw_text": self.raw_text,
            "pull_request": self.pull_request.to_dict(),
            "requested_by_session_id": self.requested_by_session_id,
            "automatic": self.automatic,
        }

    @classmethod
    def from_dict(cls, value: object) -> ReviewRequest:
        if not isinstance(value, dict):
            raise ValueError("review request must be an object")
        return cls(
            raw_text=_require_text(value.get("raw_text", ""), "raw_text"),
            pull_request=PullRequestRef.from_dict(value.get("pull_request", {})),
            requested_by_session_id=str(value.get("requested_by_session_id", "")),
            automatic=bool(value.get("automatic", False)),
        )


@dataclass(frozen=True, slots=True)
class PullRequestContext:
    pull_request: PullRequestRef
    title: str
    body: str
    author: str
    files: tuple[str, ...]
    diff_text: str
    opened_at: datetime

    def __post_init__(self) -> None:
        if not self.pull_request.repository:
            raise ValueError("pull_request.repository is required for live context")
        if not self.pull_request.base_sha or not self.pull_request.head_sha:
            raise ValueError("pull_request context must include exact base and head SHAs")
        _require_text(self.title, "title")
        _require_text(self.author, "author")
        if self.opened_at.tzinfo is None:
            raise ValueError("opened_at must include a timezone")
        object.__setattr__(
            self, "files", tuple(_normalize_path(path, "files") for path in self.files)
        )
        _require_text_preserve(self.diff_text, "diff_text")

    @property
    def diff_sha256(self) -> str:
        return _sha256_text(self.diff_text)

    def to_dict(self) -> dict[str, Any]:
        return {
            "pull_request": self.pull_request.to_dict(),
            "title": self.title,
            "body": self.body,
            "author": self.author,
            "files": list(self.files),
            "diff_text": self.diff_text,
            "opened_at": _format_timestamp(self.opened_at),
        }

    @classmethod
    def from_dict(cls, value: object) -> PullRequestContext:
        if not isinstance(value, dict):
            raise ValueError("pull request context must be an object")
        files = value.get("files", ())
        if not isinstance(files, (list, tuple)):
            raise ValueError("files must be a list or tuple")
        return cls(
            pull_request=PullRequestRef.from_dict(value.get("pull_request", {})),
            title=_require_text(value.get("title", ""), "title"),
            body=str(value.get("body", "")),
            author=_require_text(value.get("author", ""), "author"),
            files=tuple(str(path) for path in files),
            diff_text=_require_text_preserve(value.get("diff_text", ""), "diff_text"),
            opened_at=_parse_timestamp(value.get("opened_at", ""), "opened_at"),
        )


@dataclass(frozen=True, slots=True)
class ReviewContext:
    request: ReviewRequest
    pull_request: PullRequestContext
    requested_by_session_id: str
    objective: str
    diff_context: str
    trusted_policy: str = ""
    ci_status: dict[str, str] = field(default_factory=dict)
    prior_review_state: dict[str, Any] = field(default_factory=dict)
    request_digest_sha256: str = ""

    def __post_init__(self) -> None:
        _require_text(self.requested_by_session_id, "requested_by_session_id")
        _require_text(self.objective, "objective")
        _require_text(self.diff_context, "diff_context")
        if self.request.number != self.pull_request.pull_request.number:
            raise ValueError("review request does not match the pull request number")
        if (
            self.request.repository
            and self.request.repository != self.pull_request.pull_request.repository
        ):
            raise ValueError("review request repository does not match the pull request context")
        computed = self.compute_request_digest()
        if self.request_digest_sha256:
            if self.request_digest_sha256 != computed:
                raise ValueError("request digest does not match the normalized context payload")
        else:
            object.__setattr__(self, "request_digest_sha256", computed)

    @property
    def pull_request_ref(self) -> PullRequestRef:
        return self.pull_request.pull_request

    @property
    def repository(self) -> str:
        return self.pull_request_ref.repository

    @property
    def head_sha(self) -> str:
        return self.pull_request_ref.head_sha

    def compute_request_digest(self) -> str:
        payload = {
            "request": self.request.to_dict(),
            "pull_request": self.pull_request.to_dict(),
            "requested_by_session_id": self.requested_by_session_id,
            "objective": self.objective,
            "diff_context": self.diff_context,
            "trusted_policy": self.trusted_policy,
            "ci_status": self.ci_status,
            "prior_review_state": self.prior_review_state,
        }
        return _sha256_text(_canonical_json(payload))

    def to_dict(self) -> dict[str, Any]:
        return {
            "request": self.request.to_dict(),
            "pull_request": self.pull_request.to_dict(),
            "requested_by_session_id": self.requested_by_session_id,
            "objective": self.objective,
            "diff_context": self.diff_context,
            "trusted_policy": self.trusted_policy,
            "ci_status": dict(self.ci_status),
            "prior_review_state": dict(self.prior_review_state),
            "request_digest_sha256": self.request_digest_sha256,
        }

    @classmethod
    def from_dict(cls, value: object) -> ReviewContext:
        if not isinstance(value, dict):
            raise ValueError("review context must be an object")
        return cls(
            request=ReviewRequest.from_dict(value.get("request", {})),
            pull_request=PullRequestContext.from_dict(value.get("pull_request", {})),
            requested_by_session_id=_require_text(
                value.get("requested_by_session_id", ""), "requested_by_session_id"
            ),
            objective=_require_text(value.get("objective", ""), "objective"),
            diff_context=_require_text(value.get("diff_context", ""), "diff_context"),
            trusted_policy=str(value.get("trusted_policy", "")),
            ci_status={
                str(key): str(item) for key, item in dict(value.get("ci_status", {})).items()
            },
            prior_review_state=dict(value.get("prior_review_state", {})),
            request_digest_sha256=str(value.get("request_digest_sha256", "")),
        )


@dataclass(frozen=True, slots=True)
class ReviewFinding:
    finding_id: str = ""
    severity: str = ""
    category: str = ""
    title: str = ""
    summary: str = ""
    recommendation: str = ""
    blocking: bool = False
    path: str = ""
    start_line: int | None = None
    end_line: int | None = None

    def __post_init__(self) -> None:
        if self.finding_id and not _FINDING_ID.fullmatch(self.finding_id):
            raise ValueError("finding_id must use AR-NNN format")
        for name in ("severity", "category", "title", "summary", "recommendation"):
            _require_text(getattr(self, name), name)
        if self.path:
            object.__setattr__(self, "path", _normalize_path(self.path))
        for name in ("start_line", "end_line"):
            value = getattr(self, name)
            if value is not None and (not isinstance(value, int) or value < 1):
                raise ValueError(f"{name} must be a positive integer")
        if self.start_line and self.end_line and self.end_line < self.start_line:
            raise ValueError("end_line must be greater than or equal to start_line")

    @property
    def fingerprint(self) -> str:
        return _sha256_text(
            _canonical_json(
                {
                    "severity": self.severity.casefold(),
                    "category": self.category.casefold(),
                    "title": self.title,
                    "summary": self.summary,
                    "recommendation": self.recommendation,
                    "blocking": self.blocking,
                    "path": self.path,
                    "start_line": self.start_line,
                    "end_line": self.end_line,
                }
            )
        )

    def with_finding_id(self, finding_id: str) -> ReviewFinding:
        return replace(self, finding_id=finding_id)

    def to_dict(self) -> dict[str, Any]:
        return {
            "finding_id": self.finding_id,
            "severity": self.severity,
            "category": self.category,
            "title": self.title,
            "summary": self.summary,
            "recommendation": self.recommendation,
            "blocking": self.blocking,
            "path": self.path,
            "start_line": self.start_line,
            "end_line": self.end_line,
            "fingerprint": self.fingerprint,
        }

    @classmethod
    def from_dict(cls, value: object) -> ReviewFinding:
        if not isinstance(value, dict):
            raise ValueError("review finding must be an object")
        return cls(
            finding_id=str(value.get("finding_id", "")),
            severity=_require_text(value.get("severity", ""), "severity"),
            category=_require_text(value.get("category", ""), "category"),
            title=_require_text(value.get("title", ""), "title"),
            summary=_require_text(value.get("summary", ""), "summary"),
            recommendation=_require_text(value.get("recommendation", ""), "recommendation"),
            blocking=bool(value.get("blocking", False)),
            path=str(value.get("path", "")),
            start_line=(int(value["start_line"]) if value.get("start_line") is not None else None),
            end_line=int(value["end_line"]) if value.get("end_line") is not None else None,
        )


@dataclass(frozen=True, slots=True)
class ReviewResponse:
    finding_id: str
    decision: str
    rationale: str
    evidence: tuple[str, ...] = ()
    responder_role: str = "Software Team Member"

    def __post_init__(self) -> None:
        if not _FINDING_ID.fullmatch(_require_text(self.finding_id, "finding_id")):
            raise ValueError("finding_id must use AR-NNN format")
        decision = _require_text(self.decision, "decision").casefold()
        if decision not in {"accept", "reject"}:
            raise ValueError("decision must be accept or reject")
        object.__setattr__(self, "decision", decision)
        _require_text(self.responder_role, "responder_role")
        object.__setattr__(
            self, "evidence", tuple(_require_text(item, "evidence") for item in self.evidence)
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "finding_id": self.finding_id,
            "decision": self.decision,
            "rationale": self.rationale,
            "evidence": list(self.evidence),
            "responder_role": self.responder_role,
        }

    @classmethod
    def from_dict(cls, value: object) -> ReviewResponse:
        if not isinstance(value, dict):
            raise ValueError("review response must be an object")
        evidence = value.get("evidence", ())
        if not isinstance(evidence, (list, tuple)):
            raise ValueError("evidence must be a list or tuple")
        return cls(
            finding_id=_require_text(value.get("finding_id", ""), "finding_id"),
            decision=_require_text(value.get("decision", ""), "decision"),
            rationale=str(value.get("rationale", "")),
            evidence=tuple(str(item) for item in evidence),
            responder_role=_require_text(value.get("responder_role", ""), "responder_role"),
        )


@dataclass(frozen=True, slots=True)
class ReviewCycle:
    context: ReviewContext
    reviewer_role: str
    reviewer_session_id: str
    state: ReviewState = ReviewState.RUNNING
    findings: tuple[ReviewFinding, ...] = ()
    responses: tuple[ReviewResponse, ...] = ()
    recorded_at: datetime = field(default_factory=lambda: datetime.now(UTC))
    stale_head_sha: str = ""
    integrity_hash: str = ""
    schema_version: int = 1

    def __post_init__(self) -> None:
        _require_text(self.reviewer_role, "reviewer_role")
        _require_text(self.reviewer_session_id, "reviewer_session_id")
        if self.recorded_at.tzinfo is None:
            raise ValueError("recorded_at must include a timezone")
        if self.stale_head_sha and not _OBJECT_ID.fullmatch(self.stale_head_sha):
            raise ValueError("stale_head_sha must be a full hexadecimal object ID")
        computed = self.compute_integrity_hash()
        if self.integrity_hash:
            if self.integrity_hash != computed:
                raise ValueError(
                    "integrity hash does not match the normalized review-cycle payload"
                )
        else:
            object.__setattr__(self, "integrity_hash", computed)

    @property
    def pull_request(self) -> int:
        return self.context.pull_request_ref.number

    @property
    def repository(self) -> str:
        return self.context.repository

    @property
    def base_sha(self) -> str:
        return self.context.pull_request_ref.base_sha

    @property
    def head_sha(self) -> str:
        return self.context.head_sha

    @property
    def blocking_findings(self) -> tuple[ReviewFinding, ...]:
        return tuple(finding for finding in self.findings if finding.blocking)

    def _payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "context": self.context.to_dict(),
            "reviewer_role": self.reviewer_role,
            "reviewer_session_id": self.reviewer_session_id,
            "state": self.state.value,
            "findings": [finding.to_dict() for finding in self.findings],
            "responses": [response.to_dict() for response in self.responses],
            "recorded_at": _format_timestamp(self.recorded_at),
            "stale_head_sha": self.stale_head_sha,
        }

    def compute_integrity_hash(self) -> str:
        return _sha256_text(_canonical_json(self._payload()))

    def to_dict(self, *, include_integrity: bool = True) -> dict[str, Any]:
        payload = self._payload()
        if include_integrity:
            payload["integrity_hash"] = self.integrity_hash
        return payload

    def with_integrity_hash(self, integrity_hash: str) -> ReviewCycle:
        clone = replace(self, integrity_hash="")
        object.__setattr__(clone, "integrity_hash", integrity_hash)
        return clone

    def with_state(self, state: ReviewState, *, stale_head_sha: str = "") -> ReviewCycle:
        return replace(self, state=state, stale_head_sha=stale_head_sha, integrity_hash="")

    def with_findings(
        self, findings: tuple[ReviewFinding, ...], *, overall_state: ReviewState | None = None
    ) -> ReviewCycle:
        return replace(
            self,
            findings=tuple(findings),
            state=self.state if overall_state is None else overall_state,
            integrity_hash="",
        )

    def with_responses(self, responses: tuple[ReviewResponse, ...]) -> ReviewCycle:
        return replace(self, responses=tuple(responses), integrity_hash="")

    @classmethod
    def from_dict(cls, value: object) -> ReviewCycle:
        if not isinstance(value, dict):
            raise ValueError("review cycle must be an object")
        findings = value.get("findings", ())
        responses = value.get("responses", ())
        if not isinstance(findings, list) or not isinstance(responses, list):
            raise ValueError("findings and responses must be lists")
        return cls(
            context=ReviewContext.from_dict(value.get("context", {})),
            reviewer_role=_require_text(value.get("reviewer_role", ""), "reviewer_role"),
            reviewer_session_id=_require_text(
                value.get("reviewer_session_id", ""), "reviewer_session_id"
            ),
            state=ReviewState(_require_text(value.get("state", ""), "state")),
            findings=tuple(ReviewFinding.from_dict(item) for item in findings),
            responses=tuple(ReviewResponse.from_dict(item) for item in responses),
            recorded_at=_parse_timestamp(value.get("recorded_at", ""), "recorded_at"),
            stale_head_sha=str(value.get("stale_head_sha", "")),
            integrity_hash=str(value.get("integrity_hash", "")),
            schema_version=int(value.get("schema_version", 1)),
        )


def encode_state_marker(cycle: ReviewCycle) -> str:
    encoded = (
        base64.urlsafe_b64encode(_canonical_json(cycle.to_dict()).encode("utf-8"))
        .decode("ascii")
        .rstrip("=")
    )
    return f"{STATE_MARKER_PREFIX}{encoded}{STATE_MARKER_SUFFIX}"


def decode_state_marker(text: str) -> ReviewCycle | None:
    start = text.find(STATE_MARKER_PREFIX)
    if start < 0:
        return None
    start += len(STATE_MARKER_PREFIX)
    end = text.find(STATE_MARKER_SUFFIX, start)
    if end < 0:
        return None
    encoded = text[start:end]
    padding = "=" * (-len(encoded) % 4)
    try:
        payload = json.loads(base64.urlsafe_b64decode(encoded + padding))
        return ReviewCycle.from_dict(payload)
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return None
