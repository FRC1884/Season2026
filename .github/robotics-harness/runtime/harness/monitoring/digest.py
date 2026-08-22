"""Mentor-readable summaries computed from append-only monitoring events."""

from __future__ import annotations

from collections import defaultdict
from typing import Any

from harness.monitoring.models import EventType, MonitoringEvent


def _canonical_repository(event: MonitoringEvent) -> str:
    if str(event.metadata.get("repository_role", "")) == "target":
        return str(
            event.metadata.get("repository_id")
            or event.metadata.get("source_repository")
            or event.repository
        )
    return event.repository


def _task_summaries(events: tuple[MonitoringEvent, ...]) -> dict[str, dict[str, Any]]:
    grouped: dict[str, list[MonitoringEvent]] = defaultdict(list)
    for event in events:
        if event.task_identifier:
            grouped[event.task_identifier].append(event)
    summaries: dict[str, dict[str, Any]] = {}
    approval_types = {
        EventType.APPROVAL_REQUESTED,
        EventType.APPROVAL_RECORDED,
        EventType.REVIEWER_APPROVAL_RECORDED,
        EventType.MENTOR_APPROVAL_RECORDED,
        EventType.MENTOR_CHANGES_REQUESTED,
    }
    review_types = {
        EventType.AI_REVIEW_STARTED,
        EventType.AI_REVIEW_COMPLETED,
        EventType.CODEX_REVIEW_STARTED,
        EventType.CODEX_REVIEW_COMPLETED,
        EventType.CODEX_REVIEW_REQUEST_FAILED,
        EventType.REVIEW_AGENT_REQUESTED,
        EventType.REVIEW_AGENT_STARTED,
        EventType.REVIEW_PREPARED,
        EventType.REVIEW_CONTEXT_LOADED,
        EventType.REVIEW_DIFF_LOADED,
        EventType.REVIEW_COMPLETED,
        EventType.AUTO_REVIEW_STARTED,
        EventType.AUTO_REVIEW_COMPLETED,
        EventType.REVIEW_FINDING_CREATED,
        EventType.CODEX_FINDING_NEW,
        EventType.CODEX_FINDING_STILL_PRESENT,
        EventType.CODEX_FINDING_CHANGED,
        EventType.REVIEW_PUBLISHED,
        EventType.REVIEW_RESPONSE_ADDED,
        EventType.REVIEW_VALIDATED,
        EventType.REVIEW_STALE,
        EventType.REVIEW_STATE_RECORDED,
        EventType.REVIEW_MARKED_STALE,
        EventType.REVIEW_RESPONSE_STARTED,
        EventType.REVIEW_FINDING_ACCEPTED,
        EventType.REVIEW_FINDING_REJECTED,
        EventType.REVIEW_FIX_PUSHED,
        EventType.REVIEW_RERUN_REQUESTED,
        EventType.REVIEW_RERUN_COMPLETED,
    }
    for task_id, recorded in sorted(grouped.items()):
        ordered = sorted(recorded, key=lambda item: item.timestamp)
        state_events = [item for item in ordered if item.event_type is EventType.TASK_STATE_CHANGED]
        scope_events = [
            item for item in ordered if item.event_type is EventType.SCOPE_CHECK_COMPLETED
        ]
        pr_events = [
            item
            for item in ordered
            if item.event_type in {EventType.PULL_REQUEST_OPENED, EventType.PULL_REQUEST_DETECTED}
        ]
        changed_files = sorted(
            {
                path
                for item in ordered
                if item.event_type in {EventType.FILE_MODIFIED, EventType.IMPLEMENTATION_COMPLETED}
                for path in item.files_affected
            }
        )
        commands = [
            {
                "event_type": item.event_type.value,
                "command": item.tool_or_command,
                "result": item.result,
                "reference": item.commit_or_pr_reference,
            }
            for item in ordered
            if item.event_type in {EventType.COMMAND_EXECUTED, EventType.TEST_COMPLETED}
        ]
        approvals = [
            {
                "event_type": item.event_type.value,
                "actor": item.reviewer_identifier,
                "group": str(item.metadata.get("reviewer_group", "")),
                "result": item.result,
                "reference": item.commit_or_pr_reference,
            }
            for item in ordered
            if item.event_type in approval_types
        ]
        last = ordered[-1]
        summaries[task_id] = {
            "task_identifier": task_id,
            "state": (
                str(state_events[-1].metadata.get("to_state", state_events[-1].result))
                if state_events
                else "not_recorded"
            ),
            "status": last.status,
            "summary": last.summary,
            "risk_level": max(ordered, key=lambda item: item.risk_level).risk_level.label,
            "repository": _canonical_repository(last),
            "repository_role": str(last.metadata.get("repository_role", "unspecified")),
            "branch": next((item.branch for item in reversed(ordered) if item.branch), ""),
            "session_ids": sorted({item.session_id for item in ordered}),
            "event_count": len(ordered),
            "changed_files": changed_files,
            "commands": commands,
            "approvals": approvals,
            "scope": scope_events[-1].to_dict() if scope_events else None,
            "pull_request": pr_events[-1].to_dict() if pr_events else None,
            "review_event_count": sum(item.event_type in review_types for item in ordered),
            "last_event_id": last.event_id,
        }
    return summaries


def _pull_request_number(event: MonitoringEvent) -> str:
    number = str(event.metadata.get("pull_request", "")).strip()
    if number:
        return number
    reference = event.commit_or_pr_reference.strip()
    if reference.startswith("pr:"):
        return reference.removeprefix("pr:")
    return ""


def _head_sha(event: MonitoringEvent) -> str:
    return str(event.metadata.get("head_sha", "")).strip() or event.commit_or_pr_reference.strip()


def _review_result(event: MonitoringEvent) -> str:
    return str(event.metadata.get("result", event.result)).strip()


def _finding_is_blocking(event: MonitoringEvent) -> bool:
    if isinstance(event.metadata.get("blocking"), bool):
        return bool(event.metadata["blocking"])
    return str(event.metadata.get("severity", "")).casefold() in {"critical", "high"}


def _finding_is_safety(event: MonitoringEvent) -> bool:
    category = str(event.metadata.get("category", "")).casefold()
    title = str(event.metadata.get("title", "")).casefold()
    return "safety" in category or "safety" in title


def _ai_review_digest(events: tuple[MonitoringEvent, ...]) -> dict[str, Any]:
    """Reduce AI pull-request review events into mentor attention queues."""

    pull_requests: dict[str, dict[str, Any]] = {}
    for event in sorted(events, key=lambda item: item.timestamp):
        number = _pull_request_number(event)
        if not number:
            continue
        key = f"{_canonical_repository(event)}#{number}"
        summary = pull_requests.setdefault(
            key,
            {
                "repository": _canonical_repository(event),
                "pull_request": number,
                "current_head_sha": "",
                "reviewed_head_sha": "",
                "request_state": "not_requested",
                "gate_state": "not_started",
                "completion_state": "not_completed",
                "result": "not_reviewed",
                "safety_escalation": False,
                "mentor_status": "not_started",
                "findings": 0,
                "blocking_findings": 0,
            },
        )
        if event.event_type in {
            EventType.PR_CREATED,
            EventType.PULL_REQUEST_OPENED,
            EventType.PULL_REQUEST_DETECTED,
            EventType.AI_HEAD_SHA_RECORDED,
            EventType.CODEX_REVIEW_BASE_HEAD_RECORDED,
            EventType.REVIEW_AGENT_REQUESTED,
            EventType.REVIEW_AGENT_STARTED,
            EventType.REVIEW_PREPARED,
            EventType.REVIEW_CONTEXT_LOADED,
            EventType.REVIEW_DIFF_LOADED,
            EventType.REVIEW_VALIDATED,
            EventType.REVIEW_STALE,
            EventType.REVIEW_MARKED_STALE,
            EventType.REVIEW_RERUN_REQUESTED,
        } and _head_sha(event):
            summary["current_head_sha"] = _head_sha(event)
        elif event.event_type in {
            EventType.CODEX_REVIEW_REQUESTED,
            EventType.REVIEW_AGENT_REQUESTED,
        }:
            summary["request_state"] = event.result or "requested"
        elif event.event_type is EventType.CODEX_REVIEW_REQUEST_COMPLETED:
            summary["request_state"] = event.result or "completed"
        elif event.event_type is EventType.CODEX_REVIEW_REQUEST_FAILED:
            summary["request_state"] = event.result or "failed"
            summary["reviewed_head_sha"] = _head_sha(event)
            summary["result"] = _review_result(event) or "review_incomplete"
        elif event.event_type is EventType.CODEX_REVIEW_GATE_PASSED:
            summary["gate_state"] = event.result or "passed"
        elif event.event_type is EventType.CODEX_REVIEW_GATE_BLOCKED:
            summary["gate_state"] = event.result or "blocked"
            summary["reviewed_head_sha"] = _head_sha(event)
            summary["result"] = _review_result(event) or "review_incomplete"
        elif event.event_type in {
            EventType.AI_REVIEW_COMPLETED,
            EventType.CODEX_REVIEW_COMPLETED,
            EventType.REVIEW_COMPLETED,
            EventType.AUTO_REVIEW_COMPLETED,
            EventType.REVIEW_RERUN_COMPLETED,
        }:
            summary["reviewed_head_sha"] = _head_sha(event)
            summary["completion_state"] = event.result or _review_result(event) or "completed"
            summary["result"] = _review_result(event)
        elif event.event_type is EventType.REVIEW_PUBLISHED:
            summary["reviewed_head_sha"] = _head_sha(event)
            summary["completion_state"] = event.result or "published"
            summary["result"] = str(event.metadata.get("review_result", event.result)).strip()
        elif event.event_type is EventType.REVIEW_VALIDATED:
            summary["gate_state"] = event.result or "validated"
        elif event.event_type in {EventType.REVIEW_STALE, EventType.REVIEW_MARKED_STALE}:
            summary["reviewed_head_sha"] = str(event.metadata.get("previous_head_sha", "")).strip()
            summary["result"] = "stale"
        elif event.event_type is EventType.REVIEW_RERUN_REQUESTED:
            summary["request_state"] = event.result or "requested"
        elif event.event_type in {
            EventType.AI_CHECK_FAILED,
        }:
            summary["reviewed_head_sha"] = _head_sha(event)
            summary["result"] = _review_result(event) or "review_incomplete"
        elif event.event_type in {
            EventType.AI_BLOCKING_FINDING_CREATED,
            EventType.CODEX_FINDING_NEW,
            EventType.CODEX_FINDING_STILL_PRESENT,
            EventType.CODEX_FINDING_CHANGED,
            EventType.REVIEW_FINDING_CREATED,
        }:
            summary["findings"] = int(summary["findings"]) + 1
            if _finding_is_blocking(event):
                summary["blocking_findings"] = int(summary["blocking_findings"]) + 1
            if _finding_is_safety(event):
                summary["safety_escalation"] = True
        elif event.event_type is EventType.AI_SAFETY_ESCALATION_CREATED:
            summary["safety_escalation"] = True
        elif event.event_type in {EventType.MENTOR_APPROVAL_RECORDED, EventType.APPROVAL_RECORDED}:
            summary["mentor_status"] = "approved"
        elif event.event_type is EventType.MENTOR_CHANGES_REQUESTED:
            summary["mentor_status"] = "changes_requested"

    awaiting_ai: list[str] = []
    blocking: list[str] = []
    awaiting_student: list[str] = []
    awaiting_mentor: list[str] = []
    safety: list[str] = []
    stale: list[str] = []
    for key, summary in pull_requests.items():
        current = str(summary["current_head_sha"])
        reviewed = str(summary["reviewed_head_sha"])
        result = str(summary["result"])
        has_blocking_findings = int(summary["blocking_findings"]) > 0
        if not reviewed:
            awaiting_ai.append(key)
        elif not current or reviewed != current:
            stale.append(key)
            awaiting_ai.append(key)
        elif has_blocking_findings or result in {
            "changes_requested",
            "review_incomplete",
            "blocked",
            "stale",
        }:
            blocking.append(key)
            awaiting_student.append(key)
        elif result == "escalate_to_safety_reviewer":
            blocking.append(key)
            safety.append(key)
        elif result in {"pass", "pass_with_suggestions"} and summary["mentor_status"] != "approved":
            awaiting_mentor.append(key)
        if summary["safety_escalation"] and key not in safety:
            safety.append(key)

    return {
        "pull_requests": pull_requests,
        "awaiting_ai_review": sorted(awaiting_ai),
        "blocking_ai_findings": sorted(blocking),
        "awaiting_student_correction": sorted(awaiting_student),
        "awaiting_mentor_review": sorted(awaiting_mentor),
        "requiring_safety_review": sorted(safety),
        "stale_reviews": sorted(stale),
    }


def list_sessions(events: tuple[MonitoringEvent, ...]) -> list[dict[str, Any]]:
    """Aggregate events into recent session summaries, newest first."""

    grouped: dict[str, list[MonitoringEvent]] = defaultdict(list)
    for event in events:
        grouped[event.session_id].append(event)

    summaries: list[dict[str, Any]] = []
    for session_id, session_events in grouped.items():
        ordered = sorted(session_events, key=lambda event: event.timestamp)
        event_types = {event.event_type for event in ordered}
        active = (
            EventType.SESSION_STARTED in event_types and EventType.SESSION_ENDED not in event_types
        )
        last = ordered[-1]
        summaries.append(
            {
                "session_id": session_id,
                "student_identifier": last.student_identifier,
                "repository": _canonical_repository(last),
                "branch": last.branch,
                "task_identifier": last.task_identifier,
                "started_at": ordered[0].to_dict()["timestamp"],
                "last_event_at": last.to_dict()["timestamp"],
                "active": active,
                "event_count": len(ordered),
                "repository_role": str(last.metadata.get("repository_role", "unspecified")),
                "actor_role": last.actor_role,
                "identity_assurance": last.identity_assurance,
                "status": last.status,
                "summary": last.summary,
            }
        )
    summaries.sort(key=lambda item: str(item["last_event_at"]), reverse=True)
    return summaries


def build_digest(events: tuple[MonitoringEvent, ...]) -> dict[str, Any]:
    """Build the Phase 0 monitoring digest from persisted evidence."""

    sessions = list_sessions(events)
    modified_files = sorted(
        {
            path
            for event in events
            if event.event_type in {EventType.FILE_MODIFIED, EventType.IMPLEMENTATION_COMPLETED}
            for path in event.files_affected
        }
    )
    test_events = [event for event in events if event.event_type is EventType.TEST_COMPLETED]
    passed_tests = sum(
        event.result.lower() in {"passed", "pass", "success"} for event in test_events
    )
    failed_tests = len(test_events) - passed_tests

    loop_balance: dict[str, int] = defaultdict(int)
    approval_balance: dict[str, int] = defaultdict(int)
    for event in events:
        if event.event_type is EventType.LEARNING_LOOP_STARTED:
            loop_balance[event.session_id] += 1
        elif event.event_type is EventType.LEARNING_LOOP_COMPLETED:
            loop_balance[event.session_id] -= 1
        elif event.event_type is EventType.APPROVAL_REQUESTED and not event.metadata.get(
            "reviewer_group"
        ):
            approval_balance[event.task_identifier or event.session_id] += 1
        elif event.event_type is EventType.APPROVAL_RECORDED:
            approval_balance[event.task_identifier or event.session_id] -= 1
    latest_reviewer_requests: dict[tuple[str, str], MonitoringEvent] = {}
    reviewer_grants: set[tuple[str, str, str, str]] = set()
    for event in events:
        task_key = event.task_identifier or event.session_id
        group = str(event.metadata.get("reviewer_group", ""))
        revision = str(event.metadata.get("revision", ""))
        digest = str(event.metadata.get("plan_digest", ""))
        if event.event_type is EventType.APPROVAL_REQUESTED and group:
            approval_key = (task_key, group)
            current = latest_reviewer_requests.get(approval_key)
            if current is None or event.timestamp > current.timestamp:
                latest_reviewer_requests[approval_key] = event
        elif event.event_type is EventType.REVIEWER_APPROVAL_RECORDED and group:
            reviewer_grants.add((task_key, group, revision, digest))

    protected = [
        event.to_dict() for event in events if event.event_type is EventType.PROTECTED_PATH_TOUCHED
    ]
    violations = [
        event.to_dict() for event in events if event.event_type is EventType.POLICY_VIOLATION
    ]
    task_events: dict[str, list[MonitoringEvent]] = defaultdict(list)
    for event in events:
        if event.task_identifier:
            task_events[event.task_identifier].append(event)

    task_statuses: dict[str, list[str]] = {
        "waiting_for_clarification": [],
        "waiting_for_plan_confirmation": [],
        "blocked_by_safety_ambiguity": [],
        "approved_for_implementation": [],
        "awaiting_post_implementation_student_review": [],
        "awaiting_practical_verification": [],
        "awaiting_mentor_review": [],
        "implemented_without_valid_plan_confirmation": [],
    }
    state_to_digest = {
        "awaiting_clarification": "waiting_for_clarification",
        "awaiting_plan_confirmation": "waiting_for_plan_confirmation",
        "approved_for_implementation": "approved_for_implementation",
        "awaiting_learning_review": "awaiting_post_implementation_student_review",
        "awaiting_mentor_review": "awaiting_mentor_review",
    }
    for task_id, recorded in task_events.items():
        ordered = sorted(recorded, key=lambda item: item.timestamp)
        states = [
            str(item.metadata.get("to_state", item.metadata.get("state", "")))
            for item in ordered
            if item.event_type is EventType.TASK_STATE_CHANGED
        ]
        if states and states[-1] in state_to_digest:
            task_statuses[state_to_digest[states[-1]]].append(task_id)
        if any(item.event_type is EventType.SAFETY_AMBIGUITY_DETECTED for item in ordered):
            request = latest_reviewer_requests.get((task_id, "Safety Code Owners"))
            resolved = bool(
                request
                and (
                    task_id,
                    "Safety Code Owners",
                    str(request.metadata.get("revision", "")),
                    str(request.metadata.get("plan_digest", "")),
                )
                in reviewer_grants
            )
            if not resolved:
                task_statuses["blocked_by_safety_ambiguity"].append(task_id)
        challenge_assigned = any(
            item.event_type is EventType.VERIFICATION_CHALLENGE_ASSIGNED for item in ordered
        )
        challenge_passed = any(
            item.event_type is EventType.VERIFICATION_EVIDENCE_RECORDED
            and item.result.lower() in {"passed", "pass", "success"}
            for item in ordered
        )
        if challenge_assigned and not challenge_passed:
            task_statuses["awaiting_practical_verification"].append(task_id)

        confirmed: set[tuple[str, str, str]] = set()
        unlocked: set[tuple[str, str, str]] = set()
        for item in ordered:
            key = (
                str(item.metadata.get("confirmation_id", "")),
                str(item.metadata.get("plan_digest", "")),
                str(item.metadata.get("revision", "")),
            )
            if item.event_type in {
                EventType.PLAN_CORRECTED,
                EventType.SCOPE_EXPANDED,
                EventType.RISK_CLASSIFICATION_CHANGED,
            }:
                confirmed.clear()
                unlocked.clear()
            elif item.event_type is EventType.PLAN_CONFIRMED:
                confirmed.add(key)
            elif item.event_type is EventType.IMPLEMENTATION_UNLOCKED:
                unlocked.add(key)
            elif item.event_type is EventType.IMPLEMENTATION_STARTED and (
                key not in confirmed or key not in unlocked or not all(key)
            ):
                task_statuses["implemented_without_valid_plan_confirmation"].append(task_id)
                violations.append(
                    {
                        "event_type": EventType.POLICY_VIOLATION.value,
                        "task_identifier": task_id,
                        "result": "implementation started without valid plan confirmation",
                        "policy_status": "violation",
                        "metadata": {
                            "code": "implementation-without-valid-plan-confirmation",
                            "implementation_event_id": item.event_id,
                        },
                    }
                )

    for identifiers in task_statuses.values():
        identifiers[:] = sorted(set(identifiers))

    competition = [
        event.to_dict()
        for event in events
        if event.event_type
        in {
            EventType.COMPETITION_MODE_ACTIVATED,
            EventType.COMPETITION_MODE_RENEWED,
            EventType.COMPETITION_MODE_EXPIRED,
            EventType.COMPETITION_MODE_DEACTIVATED,
            EventType.COMPETITION_SCOPE_DENIED,
            EventType.COMPETITION_RECONCILIATION_REQUIRED,
            EventType.DEPLOYMENT_ATTEMPTED,
            EventType.DEPLOYMENT_COMPLETED,
            EventType.ROLLBACK_CREATED,
            EventType.ROLLBACK_EXECUTED,
        }
        and (
            event.event_type not in {EventType.ROLLBACK_CREATED, EventType.ROLLBACK_EXECUTED}
            or bool(event.metadata.get("event_slug"))
            or bool(event.metadata.get("competition_mode"))
        )
    ]
    competition_summary: dict[str, Any] = {
        "active_count": 0,
        "expired_count": 0,
        "renewal_count": 0,
        "pending_reconciliation_count": 0,
        "protected_governance_denials": 0,
        "notification_failures": 0,
        "leases": [],
    }
    lease_state: dict[str, dict[str, Any]] = {}
    for event in sorted(events, key=lambda item: item.timestamp):
        event_slug = str(event.metadata.get("event_slug", "")).strip()
        if not event_slug:
            continue
        lease = lease_state.setdefault(
            event_slug,
            {
                "event_slug": event_slug,
                "status": "unknown",
                "actor": event.reviewer_identifier,
                "actor_role": event.actor_role,
                "renewal_count": 0,
                "reconciliation_required": False,
            },
        )
        if event.event_type is EventType.COMPETITION_MODE_ACTIVATED:
            lease["status"] = "active"
            lease["actor"] = event.reviewer_identifier
            lease["actor_role"] = event.actor_role
        elif event.event_type is EventType.COMPETITION_MODE_RENEWED:
            lease["renewal_count"] = int(event.metadata.get("renewal_count", 0))
            competition_summary["renewal_count"] = int(competition_summary["renewal_count"]) + 1
        elif event.event_type is EventType.COMPETITION_MODE_EXPIRED:
            lease["status"] = "expired"
            competition_summary["expired_count"] = int(competition_summary["expired_count"]) + 1
        elif event.event_type is EventType.COMPETITION_MODE_DEACTIVATED:
            lease["status"] = "inactive"
        elif event.event_type is EventType.COMPETITION_SCOPE_DENIED:
            competition_summary["protected_governance_denials"] = (
                int(competition_summary["protected_governance_denials"]) + 1
            )
        elif event.event_type is EventType.COMPETITION_RECONCILIATION_REQUIRED:
            lease["reconciliation_required"] = True
            competition_summary["pending_reconciliation_count"] = (
                int(competition_summary["pending_reconciliation_count"]) + 1
            )
        elif event.event_type is EventType.MENTOR_DIGEST_EMAIL_FAILED:
            competition_summary["notification_failures"] = (
                int(competition_summary["notification_failures"]) + 1
            )
    competition_summary["leases"] = sorted(
        lease_state.values(),
        key=lambda item: str(item["event_slug"]),
    )
    competition_summary["active_count"] = sum(
        1 for lease in lease_state.values() if str(lease["status"]) == "active"
    )

    pending_post_event_reviews = sorted(
        str(event.metadata["post_event_review_task"])
        for event in events
        if event.event_type is EventType.TASK_OPENED and "post_event_review_task" in event.metadata
    )
    role_activity: dict[str, dict[str, Any]] = {}
    for role in ("harness", "target", "unspecified"):
        role_events = [
            event
            for event in events
            if str(event.metadata.get("repository_role", "unspecified")) == role
        ]
        role_activity[role] = {
            "event_count": len(role_events),
            "repositories": sorted(
                {_canonical_repository(event) for event in role_events if event.repository}
            ),
            "task_identifiers": sorted(
                {event.task_identifier for event in role_events if event.task_identifier}
            ),
        }

    external_types = {
        EventType.EXTERNAL_REPOSITORY_REGISTERED,
        EventType.REPOSITORY_PATH_RESOLVED,
        EventType.REPOSITORY_CLEANLINESS_CHECKED,
        EventType.REPOSITORY_BASE_RECORDED,
        EventType.DEMO_WORKTREE_CREATED,
        EventType.CANDIDATE_EVALUATED,
        EventType.CLARIFICATION_ANSWER_RECORDED,
        EventType.SCOPE_CHECK_COMPLETED,
        EventType.SCOPE_VIOLATION_DETECTED,
        EventType.MENTOR_REVIEW_PACKAGE_GENERATED,
        EventType.REMOTE_PUSH_STATUS_RECORDED,
        EventType.MERGE_STATUS_RECORDED,
        EventType.CLEANUP_STATUS_RECORDED,
    }
    external_repositories: dict[str, dict[str, Any]] = {}
    for event in events:
        if (
            event.event_type not in external_types
            or str(event.metadata.get("repository_role", "")) != "target"
        ):
            continue
        alias = _canonical_repository(event) or "unknown"
        summary = external_repositories.setdefault(
            alias,
            {
                "repository_role": "target",
                "repository": _canonical_repository(event),
                "events": [],
                "base_commit": "",
                "branch": "",
                "worktree": "",
                "cleanliness": "not_checked",
                "candidate_identifiers": [],
                "candidate_evaluations": [],
                "selected_candidate": None,
                "rejected_candidates": [],
                "scope_check": "not_recorded",
                "mentor_review_status": "not_recorded",
                "remote_push_status": "not_recorded",
                "merge_status": "not_recorded",
                "cleanup_status": "not_recorded",
                "clarification_answers_recorded": 0,
                "learning_answers_recorded": 0,
            },
        )
        summary["events"].append(event.to_dict())
        if event.event_type is EventType.REPOSITORY_BASE_RECORDED:
            summary["base_commit"] = event.commit_or_pr_reference
            summary["branch"] = event.result
        elif event.event_type is EventType.REPOSITORY_CLEANLINESS_CHECKED:
            summary["cleanliness"] = event.result
        elif event.event_type is EventType.DEMO_WORKTREE_CREATED:
            summary["branch"] = event.result
            summary["worktree"] = event.repository
        elif event.event_type is EventType.CANDIDATE_EVALUATED:
            summary["candidate_identifiers"].append(event.result)
            summary["candidate_evaluations"].append(event.metadata)
        elif (
            event.event_type is EventType.CLARIFICATION_ANSWER_RECORDED
            and event.metadata.get("category") == "candidate_selection"
        ):
            summary["selected_candidate"] = event.metadata.get("candidate")
        elif event.event_type is EventType.SCOPE_CHECK_COMPLETED:
            summary["scope_check"] = event.result
        elif event.event_type is EventType.MENTOR_REVIEW_PACKAGE_GENERATED:
            summary["mentor_review_status"] = event.result
        elif event.event_type is EventType.REMOTE_PUSH_STATUS_RECORDED:
            summary["remote_push_status"] = event.result
        elif event.event_type is EventType.MERGE_STATUS_RECORDED:
            summary["merge_status"] = event.result
        elif event.event_type is EventType.CLEANUP_STATUS_RECORDED:
            summary["cleanup_status"] = event.result

    for summary in external_repositories.values():
        summary["candidate_identifiers"] = sorted(set(summary["candidate_identifiers"]))
        selected = summary["selected_candidate"]
        selected_identifier = (
            str(selected.get("identifier", "")) if isinstance(selected, dict) else ""
        )
        summary["rejected_candidates"] = [
            {
                "candidate": candidate,
                "reason": (
                    f"not selected after the student chose {selected_identifier}"
                    if selected_identifier
                    else "awaiting student candidate selection"
                ),
            }
            for candidate in summary["candidate_evaluations"]
            if str(candidate.get("identifier", "")) != selected_identifier
        ]
        task_ids = {
            str(event["task_identifier"])
            for event in summary["events"]
            if event.get("task_identifier")
        }
        related = [
            event
            for event in events
            if event.task_identifier in task_ids
            and str(event.metadata.get("repository_role", "")) == "target"
        ]
        summary["task_identifiers"] = sorted(task_ids)
        summary["clarification_questions"] = [
            event.result
            for event in related
            if event.event_type is EventType.CLARIFICATION_QUESTION_ASKED
        ]
        summary["clarification_answers_recorded"] = sum(
            event.event_type is EventType.CLARIFICATION_ANSWER_RECORDED for event in related
        )
        summary["learning_answers_recorded"] = sum(
            event.event_type is EventType.STUDENT_ANSWER_RECORDED for event in related
        )
        summary["student_answers_recorded"] = summary["learning_answers_recorded"]
        summary["plan_confirmed"] = any(
            event.event_type is EventType.PLAN_CONFIRMED for event in related
        )
        summary["implementation_unlocked"] = any(
            event.event_type is EventType.IMPLEMENTATION_UNLOCKED for event in related
        )
        summary["tests"] = [
            {
                "command": event.tool_or_command,
                "result": event.result,
                "commit": event.commit_or_pr_reference,
            }
            for event in related
            if event.event_type is EventType.TEST_COMPLETED
        ]
        summary["learning_review_status"] = (
            "completed"
            if any(event.event_type is EventType.LEARNING_REVIEW_COMPLETED for event in related)
            else (
                "failed"
                if any(event.event_type is EventType.LEARNING_REVIEW_FAILED for event in related)
                else "not_recorded"
            )
        )

    pending_reviewer_tasks = {
        task_key
        for (task_key, group), request in latest_reviewer_requests.items()
        if (
            task_key,
            group,
            str(request.metadata.get("revision", "")),
            str(request.metadata.get("plan_digest", "")),
        )
        not in reviewer_grants
    }
    return {
        "sessions": sessions,
        "active_session_count": sum(bool(session["active"]) for session in sessions),
        "changes_made": modified_files,
        "tests": {"passed": passed_tests, "failed": failed_tests},
        "protected_path_activity": protected,
        "incomplete_learning_loops": sorted(
            session_id for session_id, balance in loop_balance.items() if balance > 0
        ),
        "pending_approvals": sorted(
            {identifier for identifier, balance in approval_balance.items() if balance > 0}
            | pending_reviewer_tasks
        ),
        "policy_violations": violations,
        "task_statuses": task_statuses,
        "competition_mode_activity": competition,
        "competition_summary": competition_summary,
        "repository_role_activity": role_activity,
        "external_repositories": external_repositories,
        "ai_pull_request_review": _ai_review_digest(events),
        "tasks": _task_summaries(events),
        "pending_post_event_reviews": pending_post_event_reviews,
    }
