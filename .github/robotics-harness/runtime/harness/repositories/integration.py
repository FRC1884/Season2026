"""Audited orchestration helpers for an external robotics repository demo."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from harness.git import resolve_commit, run_git
from harness.models import RiskLevel
from harness.monitoring import EventStore, EventType, MonitoringEvent
from harness.policy.validator import _matches
from harness.private_io import write_private_text
from harness.repositories.binding import require_worktree_binding
from harness.repositories.candidates import CandidateEvaluation, evaluate_test_candidates
from harness.repositories.registry import (
    ExternalRepositoryInspector,
    RegisteredRepository,
    RepositoryInspection,
    RepositoryRegistry,
    ScopeBudget,
)


@dataclass(frozen=True, slots=True)
class ExternalPreflight:
    registration: RegisteredRepository
    inspection: RepositoryInspection
    candidates: tuple[CandidateEvaluation, ...]
    eligible: bool
    blocking_reasons: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "registration": self.registration.to_dict(),
            "inspection": self.inspection.to_dict(),
            "candidates": [candidate.to_dict() for candidate in self.candidates],
            "eligible": self.eligible,
            "blocking_reasons": list(self.blocking_reasons),
        }


@dataclass(frozen=True, slots=True)
class ScopeCheck:
    task_identifier: str
    plan_digest: str
    worktree_path: str
    base_commit: str
    head_commit: str
    changed_paths: tuple[str, ...]
    unexpected_paths: tuple[str, ...]
    protected_paths: tuple[str, ...]
    blocked_protected_paths: tuple[str, ...]
    harness_runtime_paths: tuple[str, ...]
    line_counts: dict[str, int]
    budget: ScopeBudget
    budget_violations: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return (
            not self.unexpected_paths
            and not self.blocked_protected_paths
            and not self.harness_runtime_paths
            and not self.budget_violations
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "passed": self.passed,
            "task_identifier": self.task_identifier,
            "plan_digest": self.plan_digest,
            "worktree_path": self.worktree_path,
            "base_commit": self.base_commit,
            "head_commit": self.head_commit,
            "changed_paths": list(self.changed_paths),
            "unexpected_paths": list(self.unexpected_paths),
            "protected_paths": list(self.protected_paths),
            "blocked_protected_paths": list(self.blocked_protected_paths),
            "harness_runtime_paths": list(self.harness_runtime_paths),
            "line_counts": dict(self.line_counts),
            "budget": self.budget.to_dict(),
            "budget_violations": list(self.budget_violations),
        }


class ExternalDemoManager:
    """Record fail-closed external-repository registration and preflight evidence."""

    def __init__(
        self,
        *,
        registry: RepositoryRegistry,
        event_store: EventStore,
        inspector: ExternalRepositoryInspector | None = None,
        session_id: str = "external-demo-preflight",
        task_identifier: str = "external-demo",
    ) -> None:
        self.registry = registry
        self.event_store = event_store
        self.inspector = inspector or ExternalRepositoryInspector()
        self.session_id = session_id
        self.task_identifier = task_identifier

    def _event(
        self,
        event_type: EventType,
        registration: RegisteredRepository,
        *,
        result: str,
        files: tuple[str, ...] = (),
        risk: RiskLevel = RiskLevel.LOW,
        policy_status: str = "not_evaluated",
        metadata: dict[str, Any] | None = None,
        reference: str = "",
        repository: str = "",
        branch: str = "",
    ) -> None:
        self.event_store.append(
            MonitoringEvent.create(
                event_type=event_type,
                session_id=self.session_id,
                repository=repository or registration.path,
                branch=branch,
                task_identifier=self.task_identifier,
                files_affected=files,
                result=result,
                risk_level=risk,
                policy_status=policy_status,
                commit_or_pr_reference=reference,
                metadata={
                    "repository_alias": registration.name,
                    "repository_id": registration.repository_id,
                    "repository_role": "target",
                    **({} if metadata is None else metadata),
                },
            )
        )

    def register(
        self,
        *,
        name: str,
        path: Path | str,
        repository_type: str,
        default_branch: str = "",
        expected_remote: str,
        protected_paths: tuple[str, ...] | None = None,
        scope_budgets: Any | None = None,
    ) -> RegisteredRepository:
        registration = self.registry.register(
            name=name,
            path=path,
            repository_type=repository_type,
            default_branch=default_branch,
            protected_paths=protected_paths,
            expected_remote=expected_remote,
            scope_budgets=scope_budgets,
        )
        preparation = self.inspector.prepare_workspace(
            registration,
            workspace=registration.path,
            runtime_parent=self.registry.path.parent / "external-runtime",
            configure_git_identity=False,
        )
        self._event(
            EventType.EXTERNAL_REPOSITORY_REGISTERED,
            registration,
            result="registered",
            metadata={
                "repository_type": registration.repository_type,
                "expected_remote": registration.expected_remote,
                "default_branch": registration.default_branch,
                "repository_id": registration.repository_id,
                "workspace_preparation": preparation.to_dict(),
            },
        )
        return registration

    def preflight(self, name: str, *, request: str) -> ExternalPreflight:
        registration = self.registry.get(name)
        self._event(
            EventType.STUDENT_REQUEST_SUBMITTED,
            registration,
            result=request,
        )
        self._event(
            EventType.REPOSITORY_INSPECTION_STARTED,
            registration,
            result="read-only",
        )
        inspection = self.inspector.inspect(registration, request=request)
        self._event(
            EventType.REPOSITORY_PATH_RESOLVED,
            registration,
            result=inspection.resolved_path,
            metadata={"remote_identity_verified": inspection.remote_identity_verified},
        )
        self._event(
            EventType.REPOSITORY_CLEANLINESS_CHECKED,
            registration,
            result="clean" if inspection.clean else "blocked_dirty",
            files=inspection.dirty_paths,
            policy_status="passed" if inspection.clean else "blocked",
        )
        self._event(
            EventType.REPOSITORY_BASE_RECORDED,
            registration,
            result=inspection.current_branch,
            reference=inspection.head_commit,
            metadata={"configured_default_branch": registration.default_branch},
        )
        for path in inspection.relevant_files:
            self._event(
                EventType.RELEVANT_FILE_IDENTIFIED,
                registration,
                result=path,
                files=(path,),
            )
        for symbol in inspection.relevant_symbols:
            self._event(
                EventType.RELEVANT_SYMBOL_IDENTIFIED,
                registration,
                result=symbol,
                metadata={"symbol": symbol},
            )
        blocking: list[str] = []
        if not registration.expected_remote:
            blocking.append("expected origin remote is not configured")
        if not inspection.remote_identity_verified:
            blocking.append("configured expected remote does not match the checkout remotes")
        if not inspection.clean:
            blocking.append("target working tree contains tracked or untracked changes")
        if inspection.current_branch != registration.default_branch:
            blocking.append(
                "target checkout is not on the configured default branch "
                f"{registration.default_branch!r}"
            )
        if (
            inspection.remote_default_branch
            and inspection.remote_default_branch != registration.default_branch
        ):
            blocking.append(
                "configured default branch does not match remote default branch "
                f"{inspection.remote_default_branch!r}"
            )
        candidates: tuple[CandidateEvaluation, ...] = ()
        if not blocking:
            candidates = evaluate_test_candidates(registration)
            if not candidates:
                blocking.append("no safe, repository-grounded test candidate was identified")
            for candidate in candidates:
                self._event(
                    EventType.CANDIDATE_EVALUATED,
                    registration,
                    result=candidate.identifier,
                    files=(candidate.source_file, candidate.proposed_test_file),
                    metadata=candidate.to_dict(),
                )
            if candidates:
                self._event(
                    EventType.CLARIFICATION_QUESTION_ASKED,
                    registration,
                    result=(
                        "Which repository-grounded test-only candidate should "
                        "the demonstration plan use?"
                    ),
                    files=tuple(candidate.source_file for candidate in candidates),
                    metadata={
                        "category": "candidate_selection",
                        "candidate_identifiers": [candidate.identifier for candidate in candidates],
                    },
                )
        return ExternalPreflight(
            registration=registration,
            inspection=inspection,
            candidates=candidates,
            eligible=not blocking,
            blocking_reasons=tuple(blocking),
        )

    def record_candidate_selection(
        self,
        registration: RegisteredRepository,
        candidate: CandidateEvaluation,
        *,
        student_identifier: str,
    ) -> None:
        self._event(
            EventType.CLARIFICATION_ANSWER_RECORDED,
            registration,
            result=candidate.identifier,
            files=(candidate.source_file, candidate.proposed_test_file),
            metadata={
                "category": "candidate_selection",
                "student_identifier": student_identifier,
                "candidate": candidate.to_dict(),
            },
        )

    def record_review_outcome(
        self,
        registration: RegisteredRepository,
        *,
        package_path: Path | str,
        review_state: str,
        cleanup_status: str,
    ) -> None:
        self._event(
            EventType.MENTOR_REVIEW_PACKAGE_GENERATED,
            registration,
            result=review_state,
            metadata={"package_path": str(package_path)},
        )
        self._event(
            EventType.REMOTE_PUSH_STATUS_RECORDED,
            registration,
            result="not_attempted",
        )
        self._event(
            EventType.MERGE_STATUS_RECORDED,
            registration,
            result="not_attempted",
        )
        self._event(
            EventType.CLEANUP_STATUS_RECORDED,
            registration,
            result=cleanup_status,
        )

    def check_scope(
        self,
        registration: RegisteredRepository,
        *,
        worktree: Path | str,
        allowed_paths: tuple[str, ...],
        base_commit: str,
        plan_digest: str,
        revision: int,
        confirmation_id: str,
        risk_level: RiskLevel = RiskLevel.LOW,
        approval_groups: frozenset[str] = frozenset(),
        maximum_changed_lines: int | None = None,
    ) -> ScopeCheck:
        if not allowed_paths:
            raise ValueError("at least one allowed path is required")
        if maximum_changed_lines is not None and maximum_changed_lines < 1:
            raise ValueError("maximum changed lines must be positive")
        worktree_path = Path(worktree).resolve()
        root = Path(run_git(worktree_path, "rev-parse", "--show-toplevel")).resolve()
        if root != worktree_path:
            raise ValueError("worktree path must resolve to its Git root")
        source_common = Path(run_git(Path(registration.path), "rev-parse", "--git-common-dir"))
        if not source_common.is_absolute():
            source_common = Path(registration.path) / source_common
        worktree_common = Path(run_git(worktree_path, "rev-parse", "--git-common-dir"))
        if not worktree_common.is_absolute():
            worktree_common = worktree_path / worktree_common
        if source_common.resolve() != worktree_common.resolve():
            raise ValueError("worktree does not belong to the registered repository")
        require_worktree_binding(
            self.event_store.read_all(),
            task_identifier=self.task_identifier,
            source_repository=registration.path,
            worktree=worktree_path,
            base_commit=base_commit,
            plan_digest=plan_digest,
            revision=revision,
            confirmation_id=confirmation_id,
        )
        base = resolve_commit(worktree_path, base_commit)
        head = resolve_commit(worktree_path, "HEAD")
        committed = {
            path
            for path in run_git(
                worktree_path,
                "diff",
                "--name-only",
                f"{base}...{head}",
                "--",
            ).splitlines()
            if path
        }
        uncommitted = {
            path
            for path in run_git(
                worktree_path,
                "diff",
                "--name-only",
                "HEAD",
                "--",
            ).splitlines()
            if path
        }
        untracked = {
            path
            for path in run_git(
                worktree_path,
                "ls-files",
                "--others",
                "--exclude-standard",
            ).splitlines()
            if path
        }
        changed = committed | uncommitted | untracked
        normalized_allowed: set[str] = set()
        for path in allowed_paths:
            candidate = Path(path)
            if candidate.is_absolute() or ".." in candidate.parts:
                raise ValueError("allowed paths must be repository-relative")
            normalized_allowed.add(candidate.as_posix().removeprefix("./"))
        unexpected = sorted(changed - normalized_allowed)
        protected = sorted(path for path in changed if _matches(path, registration.protected_paths))
        blocked_protected = protected if "Safety Code Owners" not in approval_groups else []
        runtime_paths = sorted(
            set(path for path in changed if _matches(path, registration.runtime_artifact_patterns))
            | self._runtime_artifact_paths(worktree_path, registration.runtime_artifact_patterns)
        )
        changed_lines_by_path: dict[str, int] = {}
        for comparison in (f"{base}...{head}", "HEAD"):
            for line in run_git(
                worktree_path,
                "diff",
                "--numstat",
                comparison,
                "--",
            ).splitlines():
                parts = line.split("\t", 2)
                if len(parts) < 3:
                    continue
                if any(value == "-" for value in parts[:2]):
                    changed_lines_by_path[parts[2]] = 1_000_000
                    break
                changed_lines_by_path[parts[2]] = changed_lines_by_path.get(parts[2], 0) + sum(
                    int(value) for value in parts[:2] if value.isdigit()
                )
        for path in untracked:
            candidate = (worktree_path / path).resolve()
            if not candidate.is_relative_to(worktree_path) or not candidate.is_file():
                changed_lines_by_path[path] = 1_000_000
                continue
            try:
                changed_lines_by_path[path] = changed_lines_by_path.get(path, 0) + len(
                    candidate.read_text(encoding="utf-8").splitlines()
                )
            except UnicodeDecodeError:
                changed_lines_by_path[path] = 1_000_000
        budget = registration.scope_budgets.for_risk(risk_level.label).with_total_override(
            maximum_changed_lines
        )
        line_counts = {"production": 0, "tests": 0, "docs": 0, "total": 0}
        for path, line_count in changed_lines_by_path.items():
            category = self._scope_category(path)
            line_counts[category] += line_count
            line_counts["total"] += line_count
        budget_violations = tuple(
            category
            for category in ("production", "tests", "docs", "total")
            if line_counts[category] > getattr(budget, category)
        )
        result = ScopeCheck(
            task_identifier=self.task_identifier,
            plan_digest=plan_digest,
            worktree_path=str(worktree_path),
            base_commit=base,
            head_commit=head,
            changed_paths=tuple(sorted(changed)),
            unexpected_paths=tuple(unexpected),
            protected_paths=tuple(protected),
            blocked_protected_paths=tuple(blocked_protected),
            harness_runtime_paths=tuple(runtime_paths),
            line_counts=line_counts,
            budget=budget,
            budget_violations=budget_violations,
        )
        self._event(
            EventType.SCOPE_CHECK_COMPLETED,
            registration,
            result="passed" if result.passed else "blocked",
            files=result.changed_paths,
            policy_status="passed" if result.passed else "violation",
            metadata=result.to_dict(),
            reference=head,
            repository=str(worktree_path),
            branch=run_git(worktree_path, "branch", "--show-current"),
        )
        if not result.passed:
            self._event(
                EventType.SCOPE_VIOLATION_DETECTED,
                registration,
                result="blocked",
                files=tuple(
                    sorted(
                        set(unexpected)
                        | set(result.blocked_protected_paths)
                        | set(result.harness_runtime_paths)
                    )
                ),
                policy_status="violation",
                metadata=result.to_dict(),
                repository=str(worktree_path),
                branch=run_git(worktree_path, "branch", "--show-current"),
            )
        return result

    @staticmethod
    def _scope_category(path: str) -> str:
        lowered = path.lower()
        name = Path(path).name.lower()
        if lowered.startswith("docs/") or name.endswith((".md", ".rst", ".adoc", ".txt")):
            return "docs"
        if (
            lowered.startswith(("src/test/", "tests/", "test/"))
            or "/test/" in lowered
            or name.endswith(("test.java", "test.kt", "_test.py", "_test.js"))
        ):
            return "tests"
        return "production"

    @staticmethod
    def _runtime_artifact_paths(
        worktree_path: Path,
        patterns: tuple[str, ...],
    ) -> set[str]:
        matches: set[str] = set()
        for root, directories, filenames in os.walk(worktree_path):
            current = Path(root)
            directories[:] = [name for name in directories if name != ".git"]
            for filename in filenames:
                candidate = current / filename
                relative = candidate.relative_to(worktree_path).as_posix()
                if _matches(relative, patterns):
                    matches.add(relative)
        return matches


def write_review_package(
    path: Path | str,
    *,
    task: dict[str, Any],
    repository: dict[str, Any],
    diff_summary: dict[str, Any],
    test_evidence: list[dict[str, Any]],
    learning_review: dict[str, Any],
    protected_path_evidence: dict[str, Any],
    rollback: dict[str, Any],
    recommended_code_owner_group: str,
    review_state: str,
    ai_review: dict[str, Any] | None = None,
    mentor_ai_handling: list[dict[str, Any]] | None = None,
    remote_push_status: str = "not_attempted",
    pull_request_status: str = "not_opened",
    merge_status: str = "not_attempted",
    deployment_status: str = "not_attempted",
) -> Path:
    """Write a local mentor package; never pushes, merges, or deploys."""

    allowed_states = {"ready_for_human_review", "changes_requested", "blocked", "cancelled"}
    if review_state not in allowed_states:
        raise ValueError(f"invalid mentor review state: {review_state}")
    if ai_review is not None:
        ai_result = str(ai_review.get("result", ""))
        allowed_results = {
            "pass",
            "pass_with_suggestions",
            "changes_requested",
            "escalate_to_safety_reviewer",
            "review_incomplete",
        }
        if ai_result not in allowed_results:
            raise ValueError("mentor package AI review has an invalid result")
        if review_state == "ready_for_human_review" and ai_result in {
            "changes_requested",
            "escalate_to_safety_reviewer",
            "review_incomplete",
        }:
            raise ValueError("blocking or incomplete AI review cannot be marked ready")
        unresolved_blocking = [
            finding
            for finding in ai_review.get("findings", [])
            if isinstance(finding, dict)
            and bool(finding.get("blocking"))
            and str(finding.get("state", "new")) != "resolved"
        ]
        if review_state == "ready_for_human_review" and unresolved_blocking:
            raise ValueError("unresolved blocking AI findings cannot be marked ready")
    output = Path(path)
    payload = {
        "schema_version": 2,
        "review_state": review_state,
        "task": task,
        "repository": repository,
        "diff_summary": diff_summary,
        "test_evidence": test_evidence,
        "learning_review": learning_review,
        "protected_path_evidence": protected_path_evidence,
        "rollback": rollback,
        "recommended_code_owner_group": recommended_code_owner_group,
        "ai_review": ai_review or {"status": "not_supplied"},
        "ai_review_notice": "AI review is evidence, not approval.",
        "mentor_ai_handling": mentor_ai_handling or [],
        "mentor_must_review_actual_diff": True,
        "remote_push_status": remote_push_status,
        "pull_request_status": pull_request_status,
        "merge_status": merge_status,
        "deployment_status": deployment_status,
    }
    return write_private_text(output, json.dumps(payload, indent=2, sort_keys=True) + "\n")
