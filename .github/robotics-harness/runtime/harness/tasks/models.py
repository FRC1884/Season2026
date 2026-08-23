"""Persisted models for the clarification-gated task lifecycle."""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from enum import StrEnum
from typing import Any

from harness.models import RiskLevel

TASK_SCHEMA_VERSION = 4


class TaskState(StrEnum):
    CREATED = "created"
    INSPECTING = "inspecting"
    AWAITING_CLARIFICATION = "awaiting_clarification"
    SPECIFICATION_READY = "specification_ready"
    AWAITING_PLAN_CONFIRMATION = "awaiting_plan_confirmation"
    APPROVED_FOR_IMPLEMENTATION = "approved_for_implementation"
    IMPLEMENTING = "implementing"
    IMPLEMENTATION_COMPLETE = "implementation_complete"
    AWAITING_LEARNING_REVIEW = "awaiting_learning_review"
    AWAITING_MENTOR_REVIEW = "awaiting_mentor_review"
    APPROVED = "approved"
    REJECTED = "rejected"
    CANCELLED = "cancelled"


TERMINAL_STATES = {
    TaskState.APPROVED,
    TaskState.REJECTED,
    TaskState.CANCELLED,
}

ALLOWED_TRANSITIONS: dict[TaskState, frozenset[TaskState]] = {
    TaskState.CREATED: frozenset({TaskState.INSPECTING, TaskState.CANCELLED}),
    TaskState.INSPECTING: frozenset(
        {
            TaskState.AWAITING_CLARIFICATION,
            TaskState.SPECIFICATION_READY,
            TaskState.CANCELLED,
        }
    ),
    TaskState.AWAITING_CLARIFICATION: frozenset(
        {
            TaskState.SPECIFICATION_READY,
            TaskState.REJECTED,
            TaskState.CANCELLED,
        }
    ),
    TaskState.SPECIFICATION_READY: frozenset(
        {TaskState.AWAITING_PLAN_CONFIRMATION, TaskState.CANCELLED}
    ),
    TaskState.AWAITING_PLAN_CONFIRMATION: frozenset(
        {
            TaskState.APPROVED_FOR_IMPLEMENTATION,
            TaskState.AWAITING_CLARIFICATION,
            TaskState.REJECTED,
            TaskState.CANCELLED,
        }
    ),
    TaskState.APPROVED_FOR_IMPLEMENTATION: frozenset(
        {
            TaskState.IMPLEMENTING,
            TaskState.AWAITING_PLAN_CONFIRMATION,
            TaskState.REJECTED,
            TaskState.CANCELLED,
        }
    ),
    TaskState.IMPLEMENTING: frozenset(
        {
            TaskState.IMPLEMENTATION_COMPLETE,
            TaskState.AWAITING_PLAN_CONFIRMATION,
            TaskState.REJECTED,
            TaskState.CANCELLED,
        }
    ),
    TaskState.IMPLEMENTATION_COMPLETE: frozenset(
        {TaskState.AWAITING_LEARNING_REVIEW, TaskState.REJECTED, TaskState.CANCELLED}
    ),
    TaskState.AWAITING_LEARNING_REVIEW: frozenset(
        {TaskState.AWAITING_MENTOR_REVIEW, TaskState.REJECTED, TaskState.CANCELLED}
    ),
    TaskState.AWAITING_MENTOR_REVIEW: frozenset(
        {TaskState.APPROVED, TaskState.REJECTED, TaskState.CANCELLED}
    ),
    TaskState.APPROVED: frozenset(),
    TaskState.REJECTED: frozenset(),
    TaskState.CANCELLED: frozenset(),
}


class InvalidTaskTransition(ValueError):
    """Raised when a lifecycle transition would bypass a required gate."""


@dataclass(slots=True)
class ClarificationQuestion:
    identifier: str
    category: str
    prompt: str
    grounded_files: list[str] = field(default_factory=list)
    grounded_symbols: list[str] = field(default_factory=list)
    answer: str = ""
    answered_at: str = ""

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> ClarificationQuestion:
        return cls(
            identifier=str(value.get("identifier", "")),
            category=str(value.get("category", "expected_behaviour")),
            prompt=str(value.get("prompt", "")),
            grounded_files=[str(item) for item in value.get("grounded_files", [])],
            grounded_symbols=[str(item) for item in value.get("grounded_symbols", [])],
            answer=str(value.get("answer", "")),
            answered_at=str(value.get("answered_at", "")),
        )


@dataclass(slots=True)
class TaskSpecification:
    task_id: str
    original_request: str
    clarified_problem_statement: str
    expected_behaviour: str
    reproduction_steps: list[str]
    affected_subsystem: str
    relevant_files: list[str]
    relevant_symbols: list[str]
    in_scope: list[str]
    out_of_scope: list[str]
    acceptance_criteria: list[str]
    required_tests: list[str]
    safety_constraints: list[str]
    protected_paths_unchanged: list[str]
    initial_risk_classification: str
    required_reviewer_group: str
    student_answers: list[dict[str, str]]

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> TaskSpecification:
        return cls(
            task_id=str(value.get("task_id", "")),
            original_request=str(value.get("original_request", "")),
            clarified_problem_statement=str(value.get("clarified_problem_statement", "")),
            expected_behaviour=str(value.get("expected_behaviour", "")),
            reproduction_steps=[str(item) for item in value.get("reproduction_steps", [])],
            affected_subsystem=str(value.get("affected_subsystem", "repository")),
            relevant_files=[str(item) for item in value.get("relevant_files", [])],
            relevant_symbols=[str(item) for item in value.get("relevant_symbols", [])],
            in_scope=[str(item) for item in value.get("in_scope", [])],
            out_of_scope=[str(item) for item in value.get("out_of_scope", [])],
            acceptance_criteria=[str(item) for item in value.get("acceptance_criteria", [])],
            required_tests=[str(item) for item in value.get("required_tests", [])],
            safety_constraints=[str(item) for item in value.get("safety_constraints", [])],
            protected_paths_unchanged=[
                str(item) for item in value.get("protected_paths_unchanged", [])
            ],
            initial_risk_classification=str(value.get("initial_risk_classification", "low")),
            required_reviewer_group=str(
                value.get("required_reviewer_group", "General Code Owners")
            ),
            student_answers=[
                {str(key): str(item) for key, item in answer.items()}
                for answer in value.get("student_answers", [])
                if isinstance(answer, dict)
            ],
        )


@dataclass(slots=True)
class ImplementationPlan:
    revision: int
    implementation_steps: list[str]
    files_to_inspect: list[str]
    files_to_modify: list[str]
    behavioural_changes: list[str]
    tests_to_add_or_update: list[str]
    validation_commands: list[str]
    safety_checks: list[str]
    expected_change_scope: str
    potential_failure_modes: list[str]
    rollback_strategy: str
    planned_verification_challenge: str

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> ImplementationPlan:
        return cls(
            revision=int(value.get("revision", 1)),
            implementation_steps=[str(item) for item in value.get("implementation_steps", [])],
            files_to_inspect=[str(item) for item in value.get("files_to_inspect", [])],
            files_to_modify=[str(item) for item in value.get("files_to_modify", [])],
            behavioural_changes=[str(item) for item in value.get("behavioural_changes", [])],
            tests_to_add_or_update=[str(item) for item in value.get("tests_to_add_or_update", [])],
            validation_commands=[str(item) for item in value.get("validation_commands", [])],
            safety_checks=[str(item) for item in value.get("safety_checks", [])],
            expected_change_scope=str(value.get("expected_change_scope", "")),
            potential_failure_modes=[
                str(item) for item in value.get("potential_failure_modes", [])
            ],
            rollback_strategy=str(value.get("rollback_strategy", "")),
            planned_verification_challenge=str(
                value.get(
                    "planned_verification_challenge",
                    "Identify the exact changed condition and predict the relevant test result.",
                )
            ),
        )


@dataclass(slots=True)
class PlanConfirmation:
    confirmation_id: str
    student_identifier: str
    confirmed_at: str
    revision: int
    plan_digest: str
    confirmation_event_id: str
    unlock_event_id: str

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> PlanConfirmation:
        return cls(
            confirmation_id=str(value.get("confirmation_id", "")),
            student_identifier=str(value.get("student_identifier", "")),
            confirmed_at=str(value.get("confirmed_at", "")),
            revision=int(value.get("revision", 0)),
            plan_digest=str(value.get("plan_digest", "")),
            confirmation_event_id=str(value.get("confirmation_event_id", "")),
            unlock_event_id=str(value.get("unlock_event_id", "")),
        )


@dataclass(slots=True)
class TaskRecord:
    task_id: str
    original_request: str
    repository: str
    student_identifier: str
    session_id: str
    repository_role: str = "target"
    state: TaskState = TaskState.CREATED
    schema_version: int = TASK_SCHEMA_VERSION
    created_at: str = ""
    updated_at: str = ""
    max_questions: int = 4
    questions: list[ClarificationQuestion] = field(default_factory=list)
    inspection: dict[str, Any] = field(default_factory=dict)
    specification: TaskSpecification | None = None
    plan: ImplementationPlan | None = None
    revision: int = 1
    risk_level: str = RiskLevel.LOW.label
    risk_history: list[dict[str, str]] = field(default_factory=list)
    required_reviewer_group: str = "General Code Owners"
    required_approval_groups: list[str] = field(default_factory=list)
    approvals: list[dict[str, str]] = field(default_factory=list)
    bootstrap_authorizations: list[dict[str, str]] = field(default_factory=list)
    confirmation: PlanConfirmation | None = None
    implementation_permitted: bool = False
    mentor_escalation_required: bool = False
    escalation_reason: str = ""
    implementation: dict[str, Any] = field(default_factory=dict)
    learning_review: dict[str, Any] = field(default_factory=dict)
    transition_history: list[dict[str, str]] = field(default_factory=list)

    def transition(self, new_state: TaskState, *, timestamp: str, reason: str) -> None:
        if new_state not in ALLOWED_TRANSITIONS[self.state]:
            raise InvalidTaskTransition(
                f"invalid task transition: {self.state.value} -> {new_state.value}"
            )
        old_state = self.state
        self.state = new_state
        self.updated_at = timestamp
        self.transition_history.append(
            {
                "from": old_state.value,
                "to": new_state.value,
                "timestamp": timestamp,
                "reason": reason,
            }
        )

    @property
    def current_question(self) -> ClarificationQuestion | None:
        return next((question for question in self.questions if not question.answer), None)

    def canonical_plan_payload(self) -> dict[str, Any]:
        return {
            "repository": self.repository,
            "repository_branch": str(self.inspection.get("repository_branch", "")),
            "repository_commit": str(self.inspection.get("repository_commit", "")),
            "revision": self.revision,
            "specification": None if self.specification is None else asdict(self.specification),
            "plan": None if self.plan is None else asdict(self.plan),
            "risk_level": self.risk_level,
            "required_reviewer_group": self.required_reviewer_group,
            "required_approval_groups": sorted(self.required_approval_groups),
        }

    def plan_digest(self) -> str:
        payload = json.dumps(
            self.canonical_plan_payload(),
            sort_keys=True,
            separators=(",", ":"),
        )
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["state"] = self.state.value
        value["current_question"] = (
            None if self.current_question is None else asdict(self.current_question)
        )
        value["relevant_paths"] = list(self.inspection.get("relevant_files", []))
        value["inspection"].setdefault(
            "repo_unchanged", bool(value["inspection"].get("repository_unchanged", False))
        )
        return value

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> TaskRecord:
        persisted_schema_version = int(value.get("schema_version", 1))
        if persisted_schema_version > TASK_SCHEMA_VERSION:
            raise ValueError(
                f"task record uses unsupported future schema version {persisted_schema_version}"
            )
        questions = value.get("questions", [])
        inspection = value.get("inspection", {})
        risk_level = str(value.get("risk_level", value.get("initial_risk_classification", "low")))
        raw_state = str(value.get("state", TaskState.CREATED.value))
        try:
            state = TaskState(raw_state)
        except ValueError as error:
            raise ValueError(f"unknown persisted task state: {raw_state}") from error
        return cls(
            schema_version=TASK_SCHEMA_VERSION,
            task_id=str(value.get("task_id", value.get("identifier", ""))),
            original_request=str(value.get("original_request", value.get("request", ""))),
            repository=str(value.get("repository", "")),
            student_identifier=str(
                value.get("student_identifier", value.get("student_id", "anonymous-student"))
            ),
            session_id=str(value.get("session_id", f"task-{value.get('task_id', 'legacy')}")),
            repository_role=str(value.get("repository_role", "target")),
            state=state,
            created_at=str(value.get("created_at", "")),
            updated_at=str(value.get("updated_at", "")),
            max_questions=int(value.get("max_questions", 4)),
            questions=[
                ClarificationQuestion.from_dict(item)
                for item in questions
                if isinstance(item, dict)
            ],
            inspection=dict(inspection) if isinstance(inspection, dict) else {},
            specification=(
                TaskSpecification.from_dict(value["specification"])
                if isinstance(value.get("specification"), dict)
                else None
            ),
            plan=(
                ImplementationPlan.from_dict(value["plan"])
                if isinstance(value.get("plan"), dict)
                else None
            ),
            revision=int(value.get("revision", 1)),
            risk_level=risk_level,
            risk_history=[
                {str(key): str(item) for key, item in entry.items()}
                for entry in value.get("risk_history", [])
                if isinstance(entry, dict)
            ],
            required_reviewer_group=str(
                value.get("required_reviewer_group", "General Code Owners")
            ),
            required_approval_groups=[
                str(item) for item in value.get("required_approval_groups", [])
            ],
            approvals=[
                {str(key): str(item) for key, item in entry.items()}
                for entry in value.get("approvals", [])
                if isinstance(entry, dict)
            ],
            bootstrap_authorizations=[
                {str(key): str(item) for key, item in entry.items()}
                for entry in value.get("bootstrap_authorizations", [])
                if isinstance(entry, dict)
            ],
            confirmation=(
                PlanConfirmation.from_dict(value["confirmation"])
                if isinstance(value.get("confirmation"), dict)
                else None
            ),
            implementation_permitted=bool(value.get("implementation_permitted", False)),
            mentor_escalation_required=bool(value.get("mentor_escalation_required", False)),
            escalation_reason=str(value.get("escalation_reason", "")),
            implementation=(
                dict(value.get("implementation", {}))
                if isinstance(value.get("implementation"), dict)
                else {}
            ),
            learning_review=(
                dict(value.get("learning_review", {}))
                if isinstance(value.get("learning_review"), dict)
                else {}
            ),
            transition_history=[
                {str(key): str(item) for key, item in entry.items()}
                for entry in value.get("transition_history", [])
                if isinstance(entry, dict)
            ],
        )
