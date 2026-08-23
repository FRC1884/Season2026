"""Monitoring event schema for auditable local agent activity."""

from __future__ import annotations

import hashlib
import json
import re
import uuid
from dataclasses import asdict, dataclass, field
from datetime import UTC, datetime
from enum import StrEnum
from typing import Any

from harness.models import RiskLevel


class EventType(StrEnum):
    SESSION_STARTED = "session_started"
    SESSION_ENDED = "session_ended"
    SOFTWARE_MEMBER_IDENTIFIED = "software_member_identified"
    TASK_OPENED = "task_opened"
    PROMPT_RECORDED = "prompt_recorded"
    TOOL_INVOKED = "tool_invoked"
    COMMAND_EXECUTED = "command_executed"
    FILE_READ = "file_read"
    FILE_MODIFIED = "file_modified"
    TEST_STARTED = "test_started"
    TEST_COMPLETED = "test_completed"
    COMMIT_CREATED = "commit_created"
    PUSH_ATTEMPTED = "push_attempted"
    PULL_REQUEST_OPENED = "pull_request_opened"
    PROTECTED_PATH_TOUCHED = "protected_path_touched"
    LEARNING_LOOP_STARTED = "learning_loop_started"
    LEARNING_LOOP_COMPLETED = "learning_loop_completed"
    APPROVAL_REQUESTED = "approval_requested"
    APPROVAL_RECORDED = "approval_recorded"
    POLICY_VIOLATION = "policy_violation"
    COMPETITION_MODE_ACTIVATED = "competition_mode_activated"
    COMPETITION_MODE_RENEWED = "competition_mode_renewed"
    COMPETITION_MODE_EXPIRED = "competition_mode_expired"
    COMPETITION_MODE_DEACTIVATED = "competition_mode_deactivated"
    COMPETITION_SCOPE_DENIED = "competition_scope_denied"
    COMPETITION_RECONCILIATION_REQUIRED = "competition_reconciliation_required"
    COMPETITION_OUT_OF_WINDOW_DENIED = "competition_out_of_window_denied"
    COMPETITION_PUSH_RECORDED = "competition_push_recorded"
    COMPETITION_TEST_RECORDED = "competition_test_recorded"
    COMPETITION_FAILURE_RECORDED = "competition_failure_recorded"
    COMPETITION_NOTIFICATION_RECORDED = "competition_notification_recorded"
    COMPETITION_RECONCILED = "competition_reconciled"
    DEPLOYMENT_ATTEMPTED = "deployment_attempted"
    DEPLOYMENT_COMPLETED = "deployment_completed"
    ROLLBACK_CREATED = "rollback_created"
    ROLLBACK_EXECUTED = "rollback_executed"
    STUDENT_REQUEST_SUBMITTED = "student_request_submitted"
    REPOSITORY_INSPECTION_STARTED = "repository_inspection_started"
    RELEVANT_FILE_IDENTIFIED = "relevant_file_identified"
    RELEVANT_SYMBOL_IDENTIFIED = "relevant_symbol_identified"
    CLARIFICATION_QUESTION_ASKED = "clarification_question_asked"
    CLARIFICATION_ANSWER_RECORDED = "clarification_answer_recorded"
    AMBIGUITY_DETECTED = "ambiguity_detected"
    SAFETY_AMBIGUITY_DETECTED = "safety_ambiguity_detected"
    CLARIFICATION_LIMIT_REACHED = "clarification_limit_reached"
    MENTOR_ESCALATION_REQUESTED = "mentor_escalation_requested"
    TASK_SPECIFICATION_GENERATED = "task_specification_generated"
    IMPLEMENTATION_PLAN_GENERATED = "implementation_plan_generated"
    PLAN_CORRECTED = "plan_corrected"
    PLAN_CONFIRMED = "plan_confirmed"
    PLAN_REJECTED = "plan_rejected"
    IMPLEMENTATION_UNLOCKED = "implementation_unlocked"
    IMPLEMENTATION_STARTED = "implementation_started"
    IMPLEMENTATION_COMPLETED = "implementation_completed"
    POST_IMPLEMENTATION_QUESTION_ASKED = "post_implementation_question_asked"
    STUDENT_ANSWER_RECORDED = "student_answer_recorded"
    VERIFICATION_CHALLENGE_ASSIGNED = "verification_challenge_assigned"
    VERIFICATION_EVIDENCE_RECORDED = "verification_evidence_recorded"
    LEARNING_REVIEW_COMPLETED = "learning_review_completed"
    LEARNING_REVIEW_FAILED = "learning_review_failed"
    RISK_CLASSIFICATION_CHANGED = "risk_classification_changed"
    TASK_STATE_CHANGED = "task_state_changed"
    SCOPE_EXPANDED = "scope_expanded"
    REVIEWER_APPROVAL_RECORDED = "reviewer_approval_recorded"
    BOOTSTRAP_AUTHORIZATION_RECORDED = "bootstrap_authorization_recorded"
    BOOTSTRAP_AUTHORIZATION_CORRECTED = "bootstrap_authorization_corrected"
    BOOTSTRAP_AUTHORIZATION_EXPIRED = "bootstrap_authorization_expired"
    EXTERNAL_REPOSITORY_REGISTERED = "external_repository_registered"
    REPOSITORY_PATH_RESOLVED = "repository_path_resolved"
    REPOSITORY_CLEANLINESS_CHECKED = "repository_cleanliness_checked"
    REPOSITORY_BASE_RECORDED = "repository_base_recorded"
    DEMO_WORKTREE_CREATED = "demo_worktree_created"
    CANDIDATE_EVALUATED = "candidate_evaluated"
    SCOPE_CHECK_COMPLETED = "scope_check_completed"
    SCOPE_VIOLATION_DETECTED = "scope_violation_detected"
    MENTOR_REVIEW_PACKAGE_GENERATED = "mentor_review_package_generated"
    REMOTE_PUSH_STATUS_RECORDED = "remote_push_status_recorded"
    MERGE_STATUS_RECORDED = "merge_status_recorded"
    CLEANUP_STATUS_RECORDED = "cleanup_status_recorded"
    PULL_REQUEST_DETECTED = "pull_request_detected"
    AI_REVIEW_STARTED = "ai_review_started"
    AI_BASE_SHA_RECORDED = "ai_base_sha_recorded"
    AI_HEAD_SHA_RECORDED = "ai_head_sha_recorded"
    AI_FULL_DIFF_COLLECTED = "ai_full_diff_collected"
    AI_DIFF_COLLECTION_INCOMPLETE = "ai_diff_collection_incomplete"
    AI_REVIEW_CONTEXT_COLLECTED = "ai_review_context_collected"
    AI_REVIEW_COMPLETED = "ai_review_completed"
    AI_FINDING_CREATED = "ai_finding_created"
    AI_BLOCKING_FINDING_CREATED = "ai_blocking_finding_created"
    AI_SAFETY_ESCALATION_CREATED = "ai_safety_escalation_created"
    AI_SUMMARY_POSTED = "ai_summary_posted"
    AI_INLINE_COMMENT_POSTED = "ai_inline_comment_posted"
    AI_REVIEW_RERUN = "ai_review_rerun"
    AI_FINDING_RESOLVED = "ai_finding_resolved"
    AI_FINDING_STILL_PRESENT = "ai_finding_still_present"
    AI_CHECK_PASSED = "ai_check_passed"
    AI_CHECK_FAILED = "ai_check_failed"
    MENTOR_REVIEW_STARTED = "mentor_review_started"
    MENTOR_REVIEWED_AI_FINDINGS = "mentor_reviewed_ai_findings"
    MENTOR_APPROVAL_RECORDED = "mentor_approval_recorded"
    MENTOR_CHANGES_REQUESTED = "mentor_changes_requested"
    MERGE_NOT_ATTEMPTED = "merge_not_attempted"
    MERGE_ATTEMPTED = "merge_attempted"
    MERGE_COMPLETED = "merge_completed"
    MENTOR_DIGEST_GENERATED = "mentor_digest_generated"
    MENTOR_DIGEST_EMAILED = "mentor_digest_emailed"
    MENTOR_DIGEST_EMAIL_FAILED = "mentor_digest_email_failed"
    MENTOR_DIGEST_SKIPPED = "mentor_digest_skipped"
    GOVERNANCE_DESIRED_STATE_LOADED = "governance_desired_state_loaded"
    GOVERNANCE_VALIDATION_FAILED = "governance_validation_failed"
    GITHUB_GOVERNANCE_INSPECTED = "github_governance_inspected"
    GITHUB_GOVERNANCE_DRIFT_DETECTED = "github_governance_drift_detected"
    TEMPORARY_ACCESS_PLANNED = "temporary_access_planned"
    TEMPORARY_ACCESS_EXPIRY_RECORDED = "temporary_access_expiry_recorded"
    TEMPORARY_ACCESS_REMOVAL_PLANNED = "temporary_access_removal_planned"
    TEMPORARY_ACCESS_EXPIRY_DUE = "temporary_access_expiry_due"
    TEMPORARY_ACCESS_REMOVAL_REQUIRED = "temporary_access_removal_required"
    CODEX_REVIEW_REQUESTED = "codex_review_requested"
    CODEX_REVIEW_DUPLICATE_DETECTED = "codex_review_duplicate_detected"
    CODEX_REVIEW_SKIPPED = "codex_review_skipped"
    CODEX_REVIEW_STARTED = "codex_review_started"
    CODEX_REVIEW_BASE_HEAD_RECORDED = "codex_review_base_head_recorded"
    CODEX_REVIEW_DIFF_READY = "codex_review_diff_ready"
    CODEX_REVIEW_GATE_PASSED = "codex_review_gate_passed"
    CODEX_REVIEW_GATE_BLOCKED = "codex_review_gate_blocked"
    CODEX_REVIEW_REQUEST_COMPLETED = "codex_review_request_completed"
    CODEX_REVIEW_REQUEST_FAILED = "codex_review_request_failed"
    CODEX_REVIEW_COMPLETED = "codex_review_completed"
    CODEX_REVIEW_FINDING_DELTA_RECORDED = "codex_review_finding_delta_recorded"
    CODEX_REVIEW_COMMENT_RECORDED = "codex_review_comment_recorded"
    CODEX_FINDING_NEW = "codex_finding_new"
    CODEX_FINDING_STILL_PRESENT = "codex_finding_still_present"
    CODEX_FINDING_CHANGED = "codex_finding_changed"
    CODEX_FINDING_RESOLVED = "codex_finding_resolved"
    CODEX_STABLE_COMMENT_UPDATED = "codex_stable_comment_updated"
    CI_AUTO_REQUESTED = "ci_auto_requested"
    CI_AUTOMATIC_OBSERVED = "ci_automatic_observed"
    CI_MANUAL_REQUESTED = "ci_manual_requested"
    CI_MANUAL_AUTHORIZED = "ci_manual_authorized"
    CI_MANUAL_AUTH_DENIED = "ci_manual_auth_denied"
    CI_CHECK_RECORDED = "ci_check_recorded"
    SYNC_DRIFT_DETECTED = "sync_drift_detected"
    SYNC_GENERATED = "sync_generated"
    SYNC_ATTESTED = "sync_attested"
    SYNC_PROVENANCE_RECORDED = "sync_provenance_recorded"
    SYNC_ELIGIBILITY_DISABLED = "sync_eligibility_disabled"
    SYNC_SOURCE_APPROVAL_ATTESTED = "sync_source_approval_attested"
    SYNC_TARGET_PROVENANCE_VERIFIED = "sync_target_provenance_verified"
    SYNC_AUTO_MERGE_ELIGIBLE = "sync_auto_merge_eligible"
    SYNC_AUTO_MERGE_DISABLED = "sync_auto_merge_disabled"
    COMPETITION_PR_OPENED = "competition_pr_opened"
    COMPETITION_PR_MERGED = "competition_pr_merged"
    COMPETITION_FAILED_TEST_NOTIFICATION = "competition_failed_test_notification"
    COMPETITION_ROLLBACK_DECISION = "competition_rollback_decision"
    COMPETITION_OUT_OF_WINDOW_PUSH = "competition_out_of_window_push"
    COMPETITION_POST_EVENT_PR_OPENED = "competition_post_event_pr_opened"
    COMPETITION_RECONCILIATION_COMPLETED = "competition_reconciliation_completed"
    DIGEST_THRESHOLD_EVALUATED = "digest_threshold_evaluated"
    DIGEST_THRESHOLD_REACHED = "digest_threshold_reached"
    DIGEST_WEEKLY_GENERATED = "digest_weekly_generated"
    DIGEST_THRESHOLD_GENERATED = "digest_threshold_generated"
    DIGEST_EMAIL_ATTEMPTED = "digest_email_attempted"
    DIGEST_EMAIL_COMPLETED = "digest_email_completed"
    DIGEST_EMAIL_FAILED = "digest_email_failed"
    DIGEST_GITHUB_UPDATED = "digest_github_updated"
    PR_CREATED = "pr_created"
    REVIEW_AGENT_REQUESTED = "review_agent_requested"
    REVIEW_AGENT_STARTED = "review_agent_started"
    REVIEW_PREPARED = "review_prepared"
    REVIEW_CONTEXT_PREPARED = "review_context_prepared"
    REVIEW_DIFF_PREPARED = "review_diff_prepared"
    REVIEW_CONTEXT_LOADED = "review_context_loaded"
    REVIEW_DIFF_LOADED = "review_diff_loaded"
    REVIEW_COMPLETED = "review_completed"
    AUTO_REVIEW_STARTED = "auto_review_started"
    AUTO_REVIEW_COMPLETED = "auto_review_completed"
    REVIEW_FINDING_CREATED = "review_finding_created"
    REVIEW_PUBLISHED = "review_published"
    REVIEW_RESPONSE_ADDED = "review_response_added"
    REVIEW_VALIDATED = "review_validated"
    REVIEW_STALE = "review_stale"
    REVIEW_STATE_RECORDED = "review_state_recorded"
    REVIEW_MARKED_STALE = "review_marked_stale"
    REVIEW_RESPONSE_STARTED = "review_response_started"
    REVIEW_FINDING_ACCEPTED = "review_finding_accepted"
    REVIEW_FINDING_REJECTED = "review_finding_rejected"
    REVIEW_FIX_RECORDED = "review_fix_recorded"
    REVIEW_FIX_PUSHED = "review_fix_pushed"
    REVIEW_RERUN_REQUESTED = "review_rerun_requested"
    REVIEW_RERUN_COMPLETED = "review_rerun_completed"
    LEARNING_ANSWER_EVALUATED = "learning_answer_evaluated"
    FLEET_REPOSITORY_ASSESSED = "fleet_repository_assessed"
    FLEET_UPDATE_PLANNED = "fleet_update_planned"
    FLEET_UPDATE_FAILED = "fleet_update_failed"
    PRE_PR_CHECK_STARTED = "pre_pr_check_started"
    PRE_PR_CHECK_COMPLETED = "pre_pr_check_completed"


def utc_now() -> datetime:
    return datetime.now(UTC)


def format_timestamp(value: datetime) -> str:
    if value.tzinfo is None:
        raise ValueError("monitoring timestamps must include a timezone")
    return value.astimezone(UTC).isoformat().replace("+00:00", "Z")


def parse_timestamp(value: str) -> datetime:
    normalized = value[:-1] + "+00:00" if value.endswith("Z") else value
    parsed = datetime.fromisoformat(normalized)
    if parsed.tzinfo is None:
        raise ValueError("monitoring timestamps must include a timezone")
    return parsed.astimezone(UTC)


_SENSITIVE_KEY_FRAGMENTS = (
    "token",
    "secret",
    "password",
    "api_key",
    "smtp_username",
    "smtp_password",
    "recipient",
    "address",
    "prompt",
    "diff",
    "transcript",
)
_EMAIL_PATTERN = re.compile(r"([A-Za-z0-9._%+-]+)@([A-Za-z0-9.-]+\.[A-Za-z]{2,})")

_AUTOMATED_REVIEW_EVENTS = frozenset(
    {
        EventType.AI_REVIEW_STARTED,
        EventType.AI_BASE_SHA_RECORDED,
        EventType.AI_HEAD_SHA_RECORDED,
        EventType.AI_FULL_DIFF_COLLECTED,
        EventType.AI_DIFF_COLLECTION_INCOMPLETE,
        EventType.AI_REVIEW_CONTEXT_COLLECTED,
        EventType.AI_REVIEW_COMPLETED,
        EventType.AI_FINDING_CREATED,
        EventType.AI_BLOCKING_FINDING_CREATED,
        EventType.AI_SAFETY_ESCALATION_CREATED,
        EventType.AI_SUMMARY_POSTED,
        EventType.AI_INLINE_COMMENT_POSTED,
        EventType.AI_REVIEW_RERUN,
        EventType.AI_FINDING_RESOLVED,
        EventType.AI_FINDING_STILL_PRESENT,
        EventType.AI_CHECK_PASSED,
        EventType.AI_CHECK_FAILED,
        EventType.CODEX_REVIEW_REQUESTED,
        EventType.CODEX_REVIEW_DUPLICATE_DETECTED,
        EventType.CODEX_REVIEW_SKIPPED,
        EventType.CODEX_REVIEW_STARTED,
        EventType.CODEX_REVIEW_BASE_HEAD_RECORDED,
        EventType.CODEX_REVIEW_DIFF_READY,
        EventType.CODEX_REVIEW_GATE_PASSED,
        EventType.CODEX_REVIEW_GATE_BLOCKED,
        EventType.CODEX_REVIEW_REQUEST_COMPLETED,
        EventType.CODEX_REVIEW_REQUEST_FAILED,
        EventType.CODEX_REVIEW_COMPLETED,
        EventType.CODEX_REVIEW_FINDING_DELTA_RECORDED,
        EventType.CODEX_REVIEW_COMMENT_RECORDED,
        EventType.CODEX_FINDING_NEW,
        EventType.CODEX_FINDING_STILL_PRESENT,
        EventType.CODEX_FINDING_CHANGED,
        EventType.CODEX_FINDING_RESOLVED,
        EventType.CODEX_STABLE_COMMENT_UPDATED,
        EventType.REVIEW_AGENT_STARTED,
        EventType.REVIEW_PREPARED,
        EventType.REVIEW_CONTEXT_PREPARED,
        EventType.REVIEW_DIFF_PREPARED,
        EventType.REVIEW_CONTEXT_LOADED,
        EventType.REVIEW_DIFF_LOADED,
        EventType.REVIEW_COMPLETED,
        EventType.AUTO_REVIEW_STARTED,
        EventType.AUTO_REVIEW_COMPLETED,
        EventType.REVIEW_FINDING_CREATED,
        EventType.REVIEW_PUBLISHED,
        EventType.REVIEW_STATE_RECORDED,
        EventType.REVIEW_RERUN_COMPLETED,
    }
)

_MENTOR_EVENTS = frozenset(
    {
        EventType.MENTOR_REVIEW_STARTED,
        EventType.MENTOR_REVIEWED_AI_FINDINGS,
        EventType.MENTOR_APPROVAL_RECORDED,
        EventType.MENTOR_CHANGES_REQUESTED,
        EventType.MENTOR_REVIEW_PACKAGE_GENERATED,
        EventType.APPROVAL_REQUESTED,
        EventType.APPROVAL_RECORDED,
        EventType.REVIEWER_APPROVAL_RECORDED,
    }
)

_HARNESS_EVENTS = frozenset(
    {
        EventType.REPOSITORY_INSPECTION_STARTED,
        EventType.TASK_SPECIFICATION_GENERATED,
        EventType.IMPLEMENTATION_PLAN_GENERATED,
        EventType.PLAN_CORRECTED,
        EventType.IMPLEMENTATION_UNLOCKED,
        EventType.POLICY_VIOLATION,
        EventType.SCOPE_CHECK_COMPLETED,
        EventType.SCOPE_VIOLATION_DETECTED,
        EventType.PRE_PR_CHECK_STARTED,
        EventType.PRE_PR_CHECK_COMPLETED,
        EventType.REVIEW_AGENT_REQUESTED,
        EventType.REVIEW_VALIDATED,
        EventType.REVIEW_STALE,
        EventType.REVIEW_MARKED_STALE,
        EventType.REVIEW_RERUN_REQUESTED,
        EventType.SYNC_DRIFT_DETECTED,
        EventType.SYNC_GENERATED,
        EventType.SYNC_ATTESTED,
        EventType.SYNC_PROVENANCE_RECORDED,
        EventType.SYNC_TARGET_PROVENANCE_VERIFIED,
        EventType.SYNC_AUTO_MERGE_ELIGIBLE,
        EventType.SYNC_AUTO_MERGE_DISABLED,
    }
)

_SOFTWARE_MEMBER_EVENTS = frozenset(
    {
        EventType.SESSION_STARTED,
        EventType.SESSION_ENDED,
        EventType.SOFTWARE_MEMBER_IDENTIFIED,
        EventType.STUDENT_REQUEST_SUBMITTED,
        EventType.CLARIFICATION_ANSWER_RECORDED,
        EventType.PLAN_CONFIRMED,
        EventType.IMPLEMENTATION_STARTED,
        EventType.IMPLEMENTATION_COMPLETED,
        EventType.FILE_READ,
        EventType.FILE_MODIFIED,
        EventType.COMMAND_EXECUTED,
        EventType.TEST_STARTED,
        EventType.TEST_COMPLETED,
        EventType.COMMIT_CREATED,
        EventType.PUSH_ATTEMPTED,
        EventType.PULL_REQUEST_OPENED,
        EventType.PR_CREATED,
        EventType.REVIEW_RESPONSE_ADDED,
        EventType.REVIEW_RESPONSE_STARTED,
        EventType.REVIEW_FINDING_ACCEPTED,
        EventType.REVIEW_FINDING_REJECTED,
        EventType.REVIEW_FIX_RECORDED,
        EventType.REVIEW_FIX_PUSHED,
        EventType.STUDENT_ANSWER_RECORDED,
        EventType.VERIFICATION_EVIDENCE_RECORDED,
    }
)


def _sanitize_value(value: Any, *, key: str = "") -> Any:
    lowered_key = key.casefold()
    if any(fragment in lowered_key for fragment in _SENSITIVE_KEY_FRAGMENTS):
        return "<redacted>"
    if isinstance(value, dict):
        return {
            str(item_key): _sanitize_value(item_value, key=str(item_key))
            for item_key, item_value in value.items()
        }
    if isinstance(value, list):
        return [_sanitize_value(item, key=key) for item in value]
    if isinstance(value, tuple):
        return tuple(_sanitize_value(item, key=key) for item in value)
    if isinstance(value, str):
        if _EMAIL_PATTERN.fullmatch(value):
            return "<redacted>"
        return value
    return value


def _default_actor_role(
    event_type: EventType,
    *,
    student_identifier: str,
    reviewer_identifier: str,
    metadata: dict[str, Any],
) -> str:
    explicit = str(metadata.get("actor_role", "")).strip()
    if explicit:
        return explicit
    if event_type in _AUTOMATED_REVIEW_EVENTS:
        return "Automated Reviewer"
    if reviewer_identifier.strip():
        return "Mentor" if event_type in _MENTOR_EVENTS else "Code Owner"
    if (
        event_type in _SOFTWARE_MEMBER_EVENTS
        and student_identifier.strip()
        and student_identifier != "system-service"
    ):
        return "Software Team Member"
    return "Harness"


def _default_identity_assurance(
    *,
    student_identifier: str,
    reviewer_identifier: str,
    metadata: dict[str, Any],
) -> str:
    explicit = str(metadata.get("identity_assurance", "")).strip()
    if explicit:
        return explicit
    if reviewer_identifier.strip():
        return "declared_reviewer"
    if student_identifier == "anonymous-student":
        return "anonymous"
    if student_identifier.strip() and student_identifier != "system-service":
        return "declared_local"
    return "system_generated"


def _default_status(event_type: EventType, *, result: str, policy_status: str) -> str:
    lowered_result = result.strip().casefold()
    lowered_policy = policy_status.strip().casefold()
    if lowered_policy in {"violation", "blocked", "failed"}:
        return lowered_policy
    if lowered_result in {
        "passed",
        "pass",
        "success",
        "completed",
        "approved",
        "current",
        "eligible",
    }:
        return "completed"
    if lowered_result in {
        "failed",
        "blocked",
        "rejected",
        "denied",
        "stale",
        "missing",
        "disabled",
        "changes_requested",
        "review_incomplete",
    }:
        return "blocked"
    if event_type in {EventType.SESSION_STARTED, EventType.IMPLEMENTATION_STARTED}:
        return "started"
    if event_type in {EventType.SESSION_ENDED, EventType.LEARNING_LOOP_COMPLETED}:
        return "completed"
    return "observed"


def _default_summary(event_type: EventType, *, result: str, files_affected: tuple[str, ...]) -> str:
    summary = result.strip()
    if summary:
        return summary
    if files_affected:
        return f"{event_type.value}: {', '.join(files_affected[:3])}"
    return event_type.value


def _default_references(
    *,
    commit_or_pr_reference: str,
    metadata: dict[str, Any],
) -> tuple[str, ...]:
    references: list[str] = []
    if commit_or_pr_reference.strip():
        references.append(commit_or_pr_reference.strip())
    for key, value in metadata.items():
        if not isinstance(value, str) or not value.strip():
            continue
        lowered = key.casefold()
        if (
            lowered.endswith("_path")
            or lowered.endswith("_sha256")
            or lowered
            in {
                "pull_request",
                "head_sha",
                "base_sha",
                "worktree_path",
            }
        ):
            references.append(value.strip())
    return tuple(dict.fromkeys(references))


@dataclass(frozen=True, slots=True)
class MonitoringEvent:
    """One append-only observation from a harness or agent session."""

    timestamp: datetime
    event_id: str
    session_id: str
    student_identifier: str
    device_identifier: str
    repository: str
    branch: str
    task_identifier: str
    event_type: EventType
    tool_or_command: str = ""
    files_affected: tuple[str, ...] = ()
    result: str = ""
    risk_level: RiskLevel = RiskLevel.LOW
    policy_status: str = "not_evaluated"
    commit_or_pr_reference: str = ""
    reviewer_identifier: str = ""
    actor_role: str = ""
    identity_assurance: str = ""
    status: str = ""
    summary: str = ""
    references: tuple[str, ...] = ()
    previous_event_sha256: str = ""
    event_sha256: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def create(
        cls,
        *,
        event_type: EventType,
        session_id: str,
        student_identifier: str = "anonymous-student",
        device_identifier: str = "local-device",
        repository: str = "",
        branch: str = "",
        task_identifier: str = "",
        tool_or_command: str = "",
        files_affected: tuple[str, ...] = (),
        result: str = "",
        risk_level: RiskLevel = RiskLevel.LOW,
        policy_status: str = "not_evaluated",
        commit_or_pr_reference: str = "",
        reviewer_identifier: str = "",
        actor_role: str = "",
        identity_assurance: str = "",
        status: str = "",
        summary: str = "",
        references: tuple[str, ...] = (),
        previous_event_sha256: str = "",
        event_sha256: str = "",
        metadata: dict[str, Any] | None = None,
        timestamp: datetime | None = None,
        event_id: str | None = None,
    ) -> MonitoringEvent:
        if not session_id.strip():
            raise ValueError("session_id is required")
        sanitized = {} if metadata is None else _sanitize_value(metadata)
        derived_actor_role = actor_role.strip() or _default_actor_role(
            event_type,
            student_identifier=student_identifier,
            reviewer_identifier=reviewer_identifier,
            metadata=sanitized,
        )
        derived_identity_assurance = identity_assurance.strip() or _default_identity_assurance(
            student_identifier=student_identifier,
            reviewer_identifier=reviewer_identifier,
            metadata=sanitized,
        )
        derived_status = status.strip() or _default_status(
            event_type,
            result=result,
            policy_status=policy_status,
        )
        derived_summary = summary.strip() or _default_summary(
            event_type,
            result=result,
            files_affected=files_affected,
        )
        derived_references = references or _default_references(
            commit_or_pr_reference=commit_or_pr_reference,
            metadata=sanitized,
        )
        return cls(
            timestamp=timestamp or utc_now(),
            event_id=event_id or str(uuid.uuid4()),
            session_id=session_id,
            student_identifier=student_identifier,
            device_identifier=device_identifier,
            repository=repository,
            branch=branch,
            task_identifier=task_identifier,
            event_type=event_type,
            tool_or_command=tool_or_command,
            files_affected=files_affected,
            result=result,
            risk_level=risk_level,
            policy_status=policy_status,
            commit_or_pr_reference=commit_or_pr_reference,
            reviewer_identifier=reviewer_identifier,
            actor_role=derived_actor_role,
            identity_assurance=derived_identity_assurance,
            status=derived_status,
            summary=derived_summary,
            references=derived_references,
            previous_event_sha256=previous_event_sha256,
            event_sha256=event_sha256,
            metadata=sanitized,
        )

    def to_dict(self, *, include_integrity: bool = True) -> dict[str, Any]:
        value = asdict(self)
        value["timestamp"] = format_timestamp(self.timestamp)
        value["event_type"] = self.event_type.value
        value["risk_level"] = self.risk_level.label
        value["files_affected"] = list(self.files_affected)
        value["references"] = list(self.references)
        if not include_integrity:
            value.pop("previous_event_sha256", None)
            value.pop("event_sha256", None)
        return value

    def payload_sha256(self) -> str:
        payload = self.to_dict()
        # The current digest is excluded, but the previous digest remains part
        # of the payload so reordering records requires recomputing the chain.
        payload.pop("event_sha256", None)
        normalized = json_dumps(payload)
        return hashlib.sha256(normalized.encode("utf-8")).hexdigest()

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> MonitoringEvent:
        required = (
            "timestamp",
            "event_id",
            "session_id",
            "student_identifier",
            "device_identifier",
            "repository",
            "branch",
            "task_identifier",
            "event_type",
        )
        missing = [name for name in required if name not in value]
        if missing:
            raise ValueError(f"monitoring event is missing fields: {', '.join(missing)}")
        metadata = value.get("metadata", {})
        if not isinstance(metadata, dict):
            raise ValueError("monitoring event metadata must be an object")
        files = value.get("files_affected", [])
        if not isinstance(files, list) or not all(isinstance(item, str) for item in files):
            raise ValueError("files_affected must be a list of strings")
        references = value.get("references", [])
        if not isinstance(references, list) or not all(
            isinstance(item, str) for item in references
        ):
            raise ValueError("references must be a list of strings")
        return cls(
            timestamp=parse_timestamp(str(value["timestamp"])),
            event_id=str(value["event_id"]),
            session_id=str(value["session_id"]),
            student_identifier=str(value["student_identifier"]),
            device_identifier=str(value["device_identifier"]),
            repository=str(value["repository"]),
            branch=str(value["branch"]),
            task_identifier=str(value["task_identifier"]),
            event_type=EventType(str(value["event_type"])),
            tool_or_command=str(value.get("tool_or_command", "")),
            files_affected=tuple(files),
            result=str(value.get("result", "")),
            risk_level=RiskLevel.parse(str(value.get("risk_level", "low"))),
            policy_status=str(value.get("policy_status", "not_evaluated")),
            commit_or_pr_reference=str(value.get("commit_or_pr_reference", "")),
            reviewer_identifier=str(value.get("reviewer_identifier", "")),
            actor_role=str(value.get("actor_role", "")),
            identity_assurance=str(value.get("identity_assurance", "")),
            status=str(value.get("status", "")),
            summary=str(value.get("summary", "")),
            references=tuple(references),
            previous_event_sha256=str(value.get("previous_event_sha256", "")),
            event_sha256=str(value.get("event_sha256", "")),
            metadata=metadata,
        )


def json_dumps(value: dict[str, Any]) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))
