"""Validation for implementation worktrees recorded by the harness."""

from __future__ import annotations

from pathlib import Path

from harness.git import run_git
from harness.monitoring import EventType, MonitoringEvent


class WorktreeBindingError(ValueError):
    """Raised when execution is not tied to the recorded isolated worktree."""


def require_worktree_binding(
    events: tuple[MonitoringEvent, ...],
    *,
    task_identifier: str,
    source_repository: Path | str,
    worktree: Path | str,
    base_commit: str,
    plan_digest: str,
    revision: int,
    confirmation_id: str,
) -> MonitoringEvent:
    """Require an exact task, plan, base, path, and branch worktree binding."""

    source = Path(source_repository).resolve()
    candidate = Path(worktree).resolve()
    root = Path(run_git(candidate, "rev-parse", "--show-toplevel")).resolve()
    if root != candidate:
        raise WorktreeBindingError("implementation path must resolve to its Git worktree root")
    branch = run_git(candidate, "branch", "--show-current")
    if not branch:
        raise WorktreeBindingError("implementation worktree must use a named branch")

    matching: list[MonitoringEvent] = []
    for event in events:
        if (
            event.event_type is not EventType.DEMO_WORKTREE_CREATED
            or event.task_identifier != task_identifier
        ):
            continue
        metadata = event.metadata
        try:
            recorded_worktree = Path(str(metadata.get("worktree_path", ""))).resolve()
            recorded_source = Path(str(metadata.get("source_repository", ""))).resolve()
        except (OSError, RuntimeError):
            continue
        if (
            recorded_worktree == candidate
            and recorded_source == source
            and event.branch == branch
            and event.commit_or_pr_reference == base_commit
            and str(metadata.get("base_commit", "")) == base_commit
            and str(metadata.get("plan_digest", "")) == plan_digest
            and str(metadata.get("revision", "")) == str(revision)
            and str(metadata.get("confirmation_id", "")) == confirmation_id
        ):
            matching.append(event)
    if not matching:
        raise WorktreeBindingError(
            "implementation path is not the isolated worktree recorded for the "
            "current task, plan revision, and confirmation"
        )
    return matching[-1]
