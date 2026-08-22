"""Typed schemas for Codex full-diff review inputs, outputs, and evidence."""

from __future__ import annotations

import hashlib
import json
import re
from dataclasses import asdict, dataclass, field
from datetime import UTC, datetime
from enum import StrEnum
from pathlib import PurePosixPath
from typing import Any

_OBJECT_ID = re.compile(r"^[0-9a-fA-F]{40,64}$")


class Severity(StrEnum):
    CRITICAL = "critical"
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"

    @property
    def blocks(self) -> bool:
        return self in {Severity.CRITICAL, Severity.HIGH}


class Confidence(StrEnum):
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


class ReviewCategory(StrEnum):
    CORRECTNESS = "correctness"
    ROBOTICS_SAFETY = "robotics_safety"
    SCOPE = "scope"
    TESTS = "tests"
    MAINTAINABILITY = "maintainability"
    SECURITY = "security"
    GOVERNANCE = "governance"


class ReviewResult(StrEnum):
    PASS = "pass"
    PASS_WITH_SUGGESTIONS = "pass_with_suggestions"
    CHANGES_REQUESTED = "changes_requested"
    ESCALATE_TO_SAFETY_REVIEWER = "escalate_to_safety_reviewer"
    REVIEW_INCOMPLETE = "review_incomplete"

    @property
    def check_conclusion(self) -> CheckConclusion:
        if self in {ReviewResult.PASS, ReviewResult.PASS_WITH_SUGGESTIONS}:
            return CheckConclusion.SUCCESS
        return CheckConclusion.FAILURE


class CheckConclusion(StrEnum):
    SUCCESS = "success"
    FAILURE = "failure"


class FindingState(StrEnum):
    NEW = "new"
    STILL_PRESENT = "still_present"
    CHANGED = "changed"
    RESOLVED = "resolved"


def _required_text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    text = value.strip()
    if len(text.encode("utf-8")) > 20_000:
        raise ValueError(f"{name} exceeds the structured-review size limit")
    return text


def _optional_line(value: object, name: str) -> int | None:
    if value is None:
        return None
    if not isinstance(value, int) or isinstance(value, bool) or value < 1:
        raise ValueError(f"{name} must be a positive integer or null")
    return value


def _safe_relative_path(value: object) -> str:
    path = _required_text(value, "path")
    parsed = PurePosixPath(path)
    if (
        parsed.is_absolute()
        or ".." in parsed.parts
        or any(ord(character) < 32 for character in path)
    ):
        raise ValueError("finding path must be a repository-relative POSIX path")
    return path


@dataclass(frozen=True, slots=True)
class Finding:
    """One evidence-backed review finding returned by a provider."""

    severity: Severity
    confidence: Confidence
    category: ReviewCategory
    path: str
    start_line: int | None
    end_line: int | None
    title: str
    explanation: str
    evidence: str
    impact: str
    recommendation: str
    blocking: bool
    state: FindingState = FindingState.NEW

    def __post_init__(self) -> None:
        _safe_relative_path(self.path)
        for name in ("title", "explanation", "evidence", "impact", "recommendation"):
            _required_text(getattr(self, name), name)
        if len(self.title.encode("utf-8")) > 200:
            raise ValueError("finding title exceeds 200 UTF-8 bytes")
        for name in ("explanation", "evidence", "impact", "recommendation"):
            if len(getattr(self, name).encode("utf-8")) > 800:
                raise ValueError(f"finding {name} exceeds 800 UTF-8 bytes")
        if self.start_line is None and self.end_line is not None:
            raise ValueError("end_line requires start_line")
        if (
            self.start_line is not None
            and self.end_line is not None
            and self.end_line < self.start_line
        ):
            raise ValueError("end_line must be greater than or equal to start_line")
        if self.severity.blocks != self.blocking:
            raise ValueError("blocking must be true exactly for critical and high findings")
        if self.confidence is Confidence.LOW and not (
            "possible" in self.explanation.lower()
            or "could" in self.explanation.lower()
            or "question" in self.explanation.lower()
            or "verify" in self.recommendation.lower()
        ):
            raise ValueError("low-confidence findings must be phrased as a concern or question")

    @property
    def identity(self) -> str:
        raw = "\0".join((self.category.value, self.path, self.title.casefold()))
        return hashlib.sha256(raw.encode()).hexdigest()[:20]

    @property
    def fingerprint(self) -> str:
        value = self._fields()
        raw = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
        return hashlib.sha256(raw).hexdigest()

    def with_state(self, state: FindingState) -> Finding:
        value = self.to_dict()
        value["state"] = state.value
        return Finding.from_dict(value)

    def to_dict(self) -> dict[str, Any]:
        value = self._fields()
        value["state"] = self.state.value
        value["finding_id"] = self.identity
        value["fingerprint"] = self.fingerprint
        value["stable_identifier"] = self.identity
        value["line"] = self.start_line
        value["diff_location"] = (
            None
            if self.start_line is None
            else f"{self.path}:{self.start_line}"
            if self.end_line in {None, self.start_line}
            else f"{self.path}:{self.start_line}-{self.end_line}"
        )
        value["safety_review"] = self.category is ReviewCategory.ROBOTICS_SAFETY
        return value

    def _fields(self) -> dict[str, Any]:
        return {
            "severity": self.severity.value,
            "confidence": self.confidence.value,
            "category": self.category.value,
            "path": self.path,
            "start_line": self.start_line,
            "end_line": self.end_line,
            "title": self.title,
            "explanation": self.explanation,
            "evidence": self.evidence,
            "impact": self.impact,
            "recommendation": self.recommendation,
            "blocking": self.blocking,
        }

    @classmethod
    def from_dict(cls, value: object) -> Finding:
        if not isinstance(value, dict):
            raise ValueError("finding must be an object")
        required = {
            "severity",
            "confidence",
            "category",
            "path",
            "start_line",
            "end_line",
            "title",
            "explanation",
            "evidence",
            "impact",
            "recommendation",
            "blocking",
        }
        missing = sorted(required - value.keys())
        if missing:
            raise ValueError(f"finding is missing fields: {', '.join(missing)}")
        blocking = value["blocking"]
        if not isinstance(blocking, bool):
            raise ValueError("finding blocking must be a boolean")
        severity_text = _required_text(value["severity"], "severity")
        if severity_text == "suggestion":
            severity_text = Severity.LOW.value
        return cls(
            severity=Severity(severity_text),
            confidence=Confidence(_required_text(value["confidence"], "confidence")),
            category=ReviewCategory(_required_text(value["category"], "category")),
            path=_safe_relative_path(value["path"]),
            start_line=_optional_line(value["start_line"], "start_line"),
            end_line=_optional_line(value["end_line"], "end_line"),
            title=_required_text(value["title"], "title"),
            explanation=_required_text(value["explanation"], "explanation"),
            evidence=_required_text(value["evidence"], "evidence"),
            impact=_required_text(value["impact"], "impact"),
            recommendation=_required_text(value["recommendation"], "recommendation"),
            blocking=blocking,
            state=FindingState(str(value.get("state", FindingState.NEW.value))),
        )


@dataclass(frozen=True, slots=True)
class ProviderReview:
    """Strict response schema produced by a review provider."""

    result: ReviewResult
    summary: str
    risk_level: str
    findings: tuple[Finding, ...]
    analysed_chunk_ids: tuple[str, ...]

    def __post_init__(self) -> None:
        _required_text(self.summary, "summary")
        if len(self.summary.encode("utf-8")) > 4_000:
            raise ValueError("provider summary exceeds 4,000 UTF-8 bytes")
        if len(self.findings) > 5:
            raise ValueError("provider returned more than 5 high-value findings")
        if self.risk_level not in {"low", "medium", "high", "critical"}:
            raise ValueError("risk_level must be low, medium, high, or critical")
        if not self.analysed_chunk_ids:
            raise ValueError("analysed_chunk_ids must not be empty")
        blocking = tuple(finding for finding in self.findings if finding.blocking)
        safety_blocking = tuple(
            finding for finding in blocking if finding.category is ReviewCategory.ROBOTICS_SAFETY
        )
        if self.result in {ReviewResult.PASS, ReviewResult.PASS_WITH_SUGGESTIONS} and blocking:
            raise ValueError("a passing provider result cannot include blocking findings")
        if self.result is ReviewResult.PASS and self.findings:
            raise ValueError("pass_with_suggestions is required when findings exist")
        if self.result is ReviewResult.CHANGES_REQUESTED and not blocking:
            raise ValueError("changes_requested requires a blocking finding")
        if self.result is ReviewResult.ESCALATE_TO_SAFETY_REVIEWER and not safety_blocking:
            raise ValueError(
                "escalate_to_safety_reviewer requires a blocking robotics-safety finding"
            )

    def to_dict(self) -> dict[str, Any]:
        return {
            "result": self.result.value,
            "summary": self.summary,
            "risk_level": self.risk_level,
            "findings": [finding.to_dict() for finding in self.findings],
            "analysed_chunk_ids": list(self.analysed_chunk_ids),
        }

    @classmethod
    def from_dict(cls, value: object) -> ProviderReview:
        if not isinstance(value, dict):
            raise ValueError("provider response must be an object")
        required = {"result", "summary", "risk_level", "findings", "analysed_chunk_ids"}
        missing = sorted(required - value.keys())
        if missing:
            raise ValueError(f"provider response is missing fields: {', '.join(missing)}")
        findings = value["findings"]
        chunk_ids = value["analysed_chunk_ids"]
        if not isinstance(findings, list):
            raise ValueError("findings must be a list")
        if not isinstance(chunk_ids, list) or not all(
            isinstance(item, str) and item for item in chunk_ids
        ):
            raise ValueError("analysed_chunk_ids must be a list of non-empty strings")
        return cls(
            result=ReviewResult(_required_text(value["result"], "result")),
            summary=_required_text(value["summary"], "summary"),
            risk_level=_required_text(value["risk_level"], "risk_level"),
            findings=tuple(Finding.from_dict(item) for item in findings),
            analysed_chunk_ids=tuple(chunk_ids),
        )


@dataclass(frozen=True, slots=True)
class ReviewContext:
    """Trusted policy and workflow evidence supplied separately from PR data."""

    repository_policy: str
    target_policy: str
    task_specification: str
    confirmed_plan: str
    risk_classification: str
    protected_path_result: str
    learning_review_status: str
    test_evidence: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class ReviewRequest:
    """Provider input with explicit trust boundaries and bounded diff chunks."""

    pull_request: int
    repository: str
    base_sha: str
    head_sha: str
    title: str
    body: str
    commit_messages: tuple[str, ...]
    context: ReviewContext
    chunk_ids: tuple[str, ...]
    untrusted_diff_chunks: tuple[str, ...]
    reviewed_paths: tuple[str, ...] = ()
    prior_findings: tuple[Finding, ...] = ()
    synthesis: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "pull_request": self.pull_request,
            "repository": self.repository,
            "base_sha": self.base_sha,
            "head_sha": self.head_sha,
            "title": self.title,
            "body": self.body,
            "commit_messages": list(self.commit_messages),
            "context": self.context.to_dict(),
            "chunk_ids": list(self.chunk_ids),
            "untrusted_diff_chunks": list(self.untrusted_diff_chunks),
            "reviewed_paths": list(self.reviewed_paths),
            "prior_findings": [finding.to_dict() for finding in self.prior_findings],
            "synthesis": self.synthesis,
        }


def format_timestamp(value: datetime) -> str:
    if value.tzinfo is None:
        raise ValueError("review timestamp must include a timezone")
    return value.astimezone(UTC).isoformat().replace("+00:00", "Z")


def parse_timestamp(value: object) -> datetime:
    text = _required_text(value, "reviewed_at")
    normalized = text[:-1] + "+00:00" if text.endswith("Z") else text
    parsed = datetime.fromisoformat(normalized)
    if parsed.tzinfo is None:
        raise ValueError("reviewed_at must include a timezone")
    return parsed.astimezone(UTC)


@dataclass(frozen=True, slots=True)
class AIReview:
    """Persisted review result tied to one immutable PR head."""

    pull_request: int
    repository: str
    base_sha: str
    head_sha: str
    reviewed_at: datetime
    result: ReviewResult
    summary: str
    risk_level: str
    findings: tuple[Finding, ...]
    resolved_findings: tuple[Finding, ...]
    test_evidence: dict[str, Any]
    learning_evidence: dict[str, Any]
    required_reviewer_group: str
    diff_complete: bool
    analysed_file_paths: tuple[str, ...]
    all_file_paths: tuple[str, ...]
    review_provider: str
    previous_head_sha: str = ""

    def __post_init__(self) -> None:
        if self.pull_request < 1:
            raise ValueError("pull_request must be positive")
        for name in (
            "repository",
            "base_sha",
            "head_sha",
            "summary",
            "risk_level",
            "required_reviewer_group",
            "review_provider",
        ):
            _required_text(getattr(self, name), name)
        if not _OBJECT_ID.fullmatch(self.base_sha) or not _OBJECT_ID.fullmatch(self.head_sha):
            raise ValueError("base_sha and head_sha must be full hexadecimal object IDs")
        if self.risk_level not in {"low", "medium", "high", "critical"}:
            raise ValueError("risk_level must be low, medium, high, or critical")
        if self.result is ReviewResult.REVIEW_INCOMPLETE and self.diff_complete:
            # Provider failure with a complete diff is allowed; completeness here means the
            # end-to-end review, not only Git collection.
            pass
        if self.diff_complete and set(self.analysed_file_paths) != set(self.all_file_paths):
            raise ValueError("a complete review must account for every changed file")
        if self.result in {ReviewResult.PASS, ReviewResult.PASS_WITH_SUGGESTIONS} and any(
            finding.blocking for finding in self.findings
        ):
            raise ValueError("a passing review cannot contain blocking findings")

    @property
    def check_conclusion(self) -> CheckConclusion:
        return self.result.check_conclusion

    def to_dict(self) -> dict[str, Any]:
        return {
            "pull_request": self.pull_request,
            "repository": self.repository,
            "base_sha": self.base_sha,
            "head_sha": self.head_sha,
            "reviewed_at": format_timestamp(self.reviewed_at),
            "result": self.result.value,
            "summary": self.summary,
            "risk_level": self.risk_level,
            "findings": [finding.to_dict() for finding in self.findings],
            "resolved_findings": [finding.to_dict() for finding in self.resolved_findings],
            "test_evidence": self.test_evidence,
            "learning_evidence": self.learning_evidence,
            "required_reviewer_group": self.required_reviewer_group,
            "diff_complete": self.diff_complete,
            "analysed_file_paths": list(self.analysed_file_paths),
            "all_file_paths": list(self.all_file_paths),
            "review_provider": self.review_provider,
            "previous_head_sha": self.previous_head_sha,
            "check_conclusion": self.check_conclusion.value,
        }

    @classmethod
    def from_dict(cls, value: object) -> AIReview:
        if not isinstance(value, dict):
            raise ValueError("AI review must be an object")
        required = {
            "pull_request",
            "repository",
            "base_sha",
            "head_sha",
            "reviewed_at",
            "result",
            "summary",
            "risk_level",
            "findings",
            "resolved_findings",
            "test_evidence",
            "learning_evidence",
            "required_reviewer_group",
            "diff_complete",
            "analysed_file_paths",
            "all_file_paths",
            "review_provider",
        }
        missing = sorted(required - value.keys())
        if missing:
            raise ValueError(f"AI review is missing fields: {', '.join(missing)}")
        number = value["pull_request"]
        complete = value["diff_complete"]
        findings = value["findings"]
        resolved = value["resolved_findings"]
        analysed = value["analysed_file_paths"]
        all_paths = value["all_file_paths"]
        if not isinstance(number, int) or isinstance(number, bool):
            raise ValueError("pull_request must be an integer")
        if not isinstance(complete, bool):
            raise ValueError("diff_complete must be a boolean")
        if not isinstance(findings, list) or not isinstance(resolved, list):
            raise ValueError("findings and resolved_findings must be lists")
        if not isinstance(analysed, list) or not all(isinstance(item, str) for item in analysed):
            raise ValueError("analysed_file_paths must be a list of strings")
        if not isinstance(all_paths, list) or not all(isinstance(item, str) for item in all_paths):
            raise ValueError("all_file_paths must be a list of strings")
        for evidence_name in ("test_evidence", "learning_evidence"):
            if not isinstance(value[evidence_name], dict):
                raise ValueError(f"{evidence_name} must be an object")
        return cls(
            pull_request=number,
            repository=_required_text(value["repository"], "repository"),
            base_sha=_required_text(value["base_sha"], "base_sha"),
            head_sha=_required_text(value["head_sha"], "head_sha"),
            reviewed_at=parse_timestamp(value["reviewed_at"]),
            result=ReviewResult(_required_text(value["result"], "result")),
            summary=_required_text(value["summary"], "summary"),
            risk_level=_required_text(value["risk_level"], "risk_level"),
            findings=tuple(Finding.from_dict(item) for item in findings),
            resolved_findings=tuple(Finding.from_dict(item) for item in resolved),
            test_evidence=dict(value["test_evidence"]),
            learning_evidence=dict(value["learning_evidence"]),
            required_reviewer_group=_required_text(
                value["required_reviewer_group"], "required_reviewer_group"
            ),
            diff_complete=complete,
            analysed_file_paths=tuple(analysed),
            all_file_paths=tuple(all_paths),
            review_provider=_required_text(value["review_provider"], "review_provider"),
            previous_head_sha=str(value.get("previous_head_sha", "")),
        )
