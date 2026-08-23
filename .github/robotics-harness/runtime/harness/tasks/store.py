"""Task artifact persistence with JSON and mentor-readable Markdown views."""

from __future__ import annotations

import json
import os
import re
from pathlib import Path

from harness.tasks.models import TaskRecord

_SAFE_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


class TaskStoreError(ValueError):
    """Raised when a task artifact cannot be located or decoded."""


class TaskStore:
    def __init__(self, directory: Path | str = "artifacts/tasks") -> None:
        self.directory = Path(directory)

    def _path(self, task_id: str, suffix: str) -> Path:
        if not _SAFE_IDENTIFIER.fullmatch(task_id):
            raise TaskStoreError(
                "task id must contain only letters, numbers, dots, underscores, and hyphens"
            )
        return self.directory / f"{task_id}{suffix}"

    def load(self, task_id: str) -> TaskRecord:
        path = self._path(task_id, ".json")
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except OSError as error:
            raise TaskStoreError(f"cannot read task {task_id}: {error}") from error
        except json.JSONDecodeError as error:
            raise TaskStoreError(f"invalid task record {path}: {error.msg}") from error
        if not isinstance(raw, dict):
            raise TaskStoreError(f"task record {path} must be a JSON object")
        record = TaskRecord.from_dict(raw)
        if not record.task_id:
            record.task_id = task_id
        return record

    def exists(self, task_id: str) -> bool:
        return self._path(task_id, ".json").is_file()

    def save(self, record: TaskRecord) -> tuple[Path, Path]:
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        json_path = self._path(record.task_id, ".json")
        markdown_path = self._path(record.task_id, ".md")
        self._write_private(
            json_path,
            json.dumps(record.to_dict(), indent=2, sort_keys=True) + "\n",
        )
        self._write_private(markdown_path, self.render_markdown(record))
        return json_path, markdown_path

    @staticmethod
    def _write_private(path: Path, content: str) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        if temporary.is_symlink():
            raise TaskStoreError("task temporary path cannot be a symbolic link")
        flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        descriptor = os.open(
            temporary,
            flags,
            0o600,
        )
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
        os.chmod(temporary, 0o600)
        temporary.replace(path)
        os.chmod(path, 0o600)

    @staticmethod
    def render_markdown(record: TaskRecord) -> str:
        lines = [
            f"# Task {record.task_id}",
            "",
            f"- State: **{record.state.value}**",
            f"- Student: `{record.student_identifier}`",
            f"- Repository: `{record.repository}`",
            f"- Risk: **{record.risk_level}**",
            f"- Required reviewer: **{record.required_reviewer_group}**",
            f"- Plan revision: `{record.revision}`",
            f"- Implementation unlocked: `{record.implementation_permitted}`",
            "",
            "## Original request",
            "",
            record.original_request,
            "",
            "## Repository inspection",
            "",
            f"- Repository unchanged: `{record.inspection.get('repository_unchanged', False)}`",
        ]
        for path in record.inspection.get("relevant_files", []):
            lines.append(f"- Relevant file: `{path}`")
        for symbol in record.inspection.get("relevant_symbols", []):
            lines.append(f"- Relevant symbol: `{symbol}`")
        if record.questions:
            lines.extend(["", "## Clarification record", ""])
            for question in record.questions:
                lines.extend(
                    [
                        f"### {question.identifier} — {question.category}",
                        "",
                        question.prompt,
                        "",
                        f"> {question.answer or '_Awaiting answer_'}",
                        "",
                    ]
                )
        if record.specification is not None:
            specification = record.specification
            lines.extend(
                [
                    "## Tightened task specification",
                    "",
                    f"- Problem: {specification.clarified_problem_statement}",
                    f"- Expected behaviour: {specification.expected_behaviour}",
                    f"- Affected subsystem: `{specification.affected_subsystem}`",
                    "- In scope:",
                    *[f"  - `{item}`" for item in specification.in_scope],
                    "- Out of scope:",
                    *[f"  - `{item}`" for item in specification.out_of_scope],
                    "- Acceptance criteria:",
                    *[f"  - {item}" for item in specification.acceptance_criteria],
                    "- Required tests:",
                    *[f"  - `{item}`" for item in specification.required_tests],
                    "- Safety constraints:",
                    *[f"  - {item}" for item in specification.safety_constraints],
                    "- Protected paths that must remain unchanged:",
                    *[f"  - `{item}`" for item in specification.protected_paths_unchanged],
                    "",
                ]
            )
        if record.plan is not None:
            plan = record.plan
            lines.extend(
                [
                    "## Implementation plan",
                    "",
                    "- Ordered implementation steps:",
                    *[
                        f"  {index}. {item}"
                        for index, item in enumerate(plan.implementation_steps, start=1)
                    ],
                    "- Files likely to be inspected:",
                    *[f"  - `{item}`" for item in plan.files_to_inspect],
                    "- Files likely to be modified:",
                    *[f"  - `{item}`" for item in plan.files_to_modify],
                    "- Planned behavioural changes:",
                    *[f"  - {item}" for item in plan.behavioural_changes],
                    "- Validation commands:",
                    *[f"  - `{item}`" for item in plan.validation_commands],
                    "- Safety checks:",
                    *[f"  - {item}" for item in plan.safety_checks],
                    f"- Planned practical verification: {plan.planned_verification_challenge}",
                    f"- Rollback: {plan.rollback_strategy}",
                    "",
                ]
            )
        if record.learning_review:
            lines.extend(["## Post-implementation learning review", ""])
            challenge = record.learning_review.get("practical_challenge", {})
            if challenge:
                lines.extend(
                    [
                        f"- Challenge: {challenge.get('challenge', '')}",
                        f"- Student response: {challenge.get('student_response', '')}",
                        f"- Passed: `{challenge.get('passed', False)}`",
                        "- Evidence:",
                    ]
                )
                lines.extend(
                    f"  - `{reference}`" for reference in challenge.get("evidence_references", [])
                )
        if record.mentor_escalation_required:
            lines.extend(
                [
                    "",
                    "## Mentor escalation",
                    "",
                    f"- Reason: {record.escalation_reason}",
                    "- Confirmed facts:",
                    *[f"  - {fact}" for fact in record.inspection.get("confirmed_facts", [])],
                    "- Unresolved questions:",
                    *[
                        f"  - {question}"
                        for question in record.inspection.get("unresolved_questions", [])
                    ],
                ]
            )
        return "\n".join(lines).rstrip() + "\n"
