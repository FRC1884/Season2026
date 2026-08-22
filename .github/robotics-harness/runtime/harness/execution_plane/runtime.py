"""External execution-plane runtime orchestration helpers."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import os
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path, PurePosixPath
from typing import Any

from harness.ai_review.diff import collect_full_diff
from harness.ai_review.github import (
    AuditTrail,
    PullRequestEvent,
    _reviewer_group,
    _safe_harness_evidence,
    _target_policy,
    _trusted_policy,
    changed_new_lines,
)
from harness.ai_review.models import ReviewContext
from harness.ai_review.provider import RecordedReviewProvider, ReviewProvider
from harness.ai_review.report import render_summary_comment, write_review_artifacts
from harness.ai_review.service import AIReviewService, PullRequestMetadata
from harness.execution_plane.manifest import build_execution_plane_manifest
from harness.git import diff_between, parse_unified_diff, resolve_commit, run_git
from harness.learning_loop import ReviewEngine, ReviewRequest
from harness.monitoring import EventStore, EventType, MonitoringEvent
from harness.private_io import write_private_text
from harness.risk import RiskClassifier
from harness.tasks import TaskService, TaskStore
from harness.tasks.models import TaskRecord, TaskState

_DEFAULT_RUNTIME_ALLOWLIST = (
    ".github/codex/outbound-data-policy.json",
    ".github/codex/prompts/full-diff-review-v1.md",
    ".github/codex/prompts/repository-review.md",
    ".github/codex/review.schema.json",
    ".github/codex/schemas/provider-review-output-v1.json",
    "config/risk_rules.yaml",
    "harness/__init__.py",
    "harness/ai_review/diff.py",
    "harness/ai_review/github.py",
    "harness/ai_review/__init__.py",
    "harness/ai_review/models.py",
    "harness/ai_review/prompt.py",
    "harness/ai_review/provider.py",
    "harness/ai_review/report.py",
    "harness/ai_review/service.py",
    "harness/execution_plane/__init__.py",
    "harness/execution_plane/manifest.py",
    "harness/execution_plane/runtime.py",
    "harness/git.py",
    "harness/learning_loop/__init__.py",
    "harness/learning_loop/reviewer.py",
    "harness/models.py",
    "harness/monitoring/__init__.py",
    "harness/monitoring/digest.py",
    "harness/monitoring/emitter.py",
    "harness/monitoring/models.py",
    "harness/monitoring/store.py",
    "harness/policy/__init__.py",
    "harness/policy/validator.py",
    "harness/private_io.py",
    "harness/repositories/__init__.py",
    "harness/repositories/binding.py",
    "harness/repositories/candidates.py",
    "harness/repositories/integration.py",
    "harness/repositories/registry.py",
    "harness/risk/__init__.py",
    "harness/risk/classifier.py",
    "harness/tasks/__init__.py",
    "harness/tasks/inspection.py",
    "harness/tasks/models.py",
    "harness/tasks/service.py",
    "harness/tasks/store.py",
)
_RUNTIME_TOP_LEVEL_CONTAMINATION_PARTS = frozenset(
    {
        ".codex",
        ".omx",
        "artifacts",
        "artifacts-private",
        "evidence",
        "monitoring",
        "temp",
        "tmp",
    }
)
_LOCAL_APPROVAL_STATE_PARTS = frozenset(
    {
        "approval-state",
        "approval-state.json",
        "approval-state.yaml",
        "approval-state.yml",
        "local-approval-state",
        "local-approval-state.json",
        "local-approval-state.yaml",
        "local-approval-state.yml",
        "local-approvals",
        "local-approvals.json",
    }
)
_POST_PR_REVIEW_CHECKS = frozenset({"agentic review"})
_AGENTIC_REVIEW_CHECK = "Agentic Review"
_REVIEW_RUNTIME_DIRECTORY = "agent-review"
_OPTIONAL_AGENT_REVIEW_ROOT = "harness/agent_review"
_OPTIONAL_AGENT_REVIEW_MODULES = (
    "harness.agent_review.runtime",
    "harness.agent_review.cli",
    "harness.agent_review",
)
_OPTIONAL_REVIEW_ADAPTERS = {
    "prepare": ("runtime_prepare_review", "prepare_review"),
    "record": ("runtime_record_review", "record_review"),
    "publish": ("runtime_publish_review", "publish_review"),
    "request_github": (
        "runtime_request_platform_review_github",
        "request_platform_review_github",
    ),
    "validate_github": ("runtime_validate_review_github", "validate_review_github"),
    "respond": ("runtime_record_review_response", "record_review_response"),
    "stale": ("runtime_mark_review_stale", "mark_review_stale"),
    "pr_create": ("runtime_create_pull_request", "create_pull_request"),
}
_REVIEW_STATE_MARKER = "<!-- robotics-harness-codex-review-state:"
_OPTIONAL_ADAPTER_MISSING = object()


@dataclass(frozen=True, slots=True)
class RuntimeEnvironment:
    target_repo: Path
    runtime_root: Path
    harness_revision: str
    manifest: dict[str, Any]
    manifest_path: Path
    task_store: TaskStore
    event_store: EventStore
    task_service: TaskService


class _ReplayRecordedProvider:
    """Replay recorded responses while binding them to the current chunk set."""

    name = RecordedReviewProvider.name

    def __init__(self, provider: RecordedReviewProvider) -> None:
        self._provider = provider

    def review(self, request: Any) -> Any:
        response = self._provider.review(request)
        payload = response.to_dict()
        payload["analysed_chunk_ids"] = list(request.chunk_ids)
        return type(response).from_dict(payload)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _resolve_runtime_root(runtime_root: Path | str) -> Path:
    return Path(runtime_root).expanduser().resolve()


def _require_external_runtime_root(
    target_repo: Path | str, runtime_root: Path | str
) -> tuple[Path, Path]:
    repo = Path(target_repo).expanduser().resolve()
    if not repo.is_dir():
        raise ValueError(f"target repository does not exist: {repo}")
    root = _resolve_runtime_root(runtime_root)
    if root == repo or root.is_relative_to(repo):
        raise ValueError("runtime_root must be outside the target repository")
    return repo, root


def _canonical_json(value: dict[str, Any]) -> str:
    return json.dumps(value, indent=2, sort_keys=True) + "\n"


def _runtime_allowlist() -> tuple[str, ...]:
    root = _repo_root() / _OPTIONAL_AGENT_REVIEW_ROOT
    optional: list[str] = []
    if root.is_dir():
        optional.extend(
            path.relative_to(_repo_root()).as_posix()
            for path in sorted(root.rglob("*.py"))
            if path.is_file() and not path.is_symlink()
        )
    return tuple(sorted(dict.fromkeys((*_DEFAULT_RUNTIME_ALLOWLIST, *optional))))


def _copy_runtime_sources(*, source_root: Path, destination_root: Path) -> None:
    if destination_root.exists():
        shutil.rmtree(destination_root)
    for relative in _runtime_allowlist():
        source = source_root / relative
        destination = destination_root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(source.read_bytes())


def _runtime_review_root(runtime_root: Path) -> Path:
    return runtime_root / _REVIEW_RUNTIME_DIRECTORY


def _safe_repository_slug(repository: str) -> str:
    return repository.replace("/", "--").replace(":", "--")


def _review_runtime_path(
    runtime_root: Path,
    *,
    kind: str,
    repository: str,
    pull_request: int,
    head_sha: str,
    suffix: str = ".json",
) -> Path:
    directory = _runtime_review_root(runtime_root) / kind
    filename = f"{_safe_repository_slug(repository)}-pr-{pull_request}-{head_sha}{suffix}"
    return directory / filename


def _write_runtime_json(path: Path, payload: dict[str, Any]) -> Path:
    return write_private_text(path, _canonical_json(payload))


def _call_optional_review_adapter(action: str, **kwargs: Any) -> Any:
    function_names = _OPTIONAL_REVIEW_ADAPTERS.get(action, ())
    for module_name in _OPTIONAL_AGENT_REVIEW_MODULES:
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError as error:
            missing = error.name or ""
            if missing == module_name or missing.startswith("harness.agent_review"):
                continue
            raise
        for function_name in function_names:
            candidate = getattr(module, function_name, None)
            if callable(candidate):
                return candidate(**kwargs)
    return _OPTIONAL_ADAPTER_MISSING


def _run_gh_api(
    *,
    path: str,
    method: str = "GET",
    payload: dict[str, Any] | None = None,
    cwd: Path | None = None,
) -> Any:
    command = ["gh", "api", path]
    if method != "GET":
        command.extend(["--method", method])
    input_text: str | None = None
    if payload is not None:
        command.extend(["--input", "-"])
        input_text = json.dumps(payload, ensure_ascii=False)
    completed = subprocess.run(
        command,
        cwd=cwd,
        input=input_text,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise ValueError(completed.stderr.strip() or completed.stdout.strip() or "gh api failed")
    stdout = completed.stdout.strip()
    if not stdout:
        return None
    return json.loads(stdout)


def _gh_actor_login(*, cwd: Path | None = None) -> str:
    value = _run_gh_api(path="user", cwd=cwd)
    if not isinstance(value, dict) or not isinstance(value.get("login"), str):
        raise ValueError("gh auth could not identify the authenticated actor")
    return str(value["login"])


def _require_github_actions_token() -> str:
    value = os.environ.get("GITHUB_TOKEN", "").strip()
    if not value:
        raise ValueError("review-validate-github requires GITHUB_TOKEN in the environment")
    return value


def _pull_request_payload(
    *,
    repository: str,
    pull_request: int,
    cwd: Path | None = None,
) -> dict[str, Any]:
    value = _run_gh_api(
        path=f"repos/{repository}/pulls/{pull_request}",
        cwd=cwd,
    )
    if not isinstance(value, dict):
        raise ValueError("pull-request lookup returned invalid JSON")
    return value


def _pull_request_state(
    *,
    repository: str,
    pull_request: int,
    cwd: Path | None = None,
) -> dict[str, Any]:
    payload = _pull_request_payload(repository=repository, pull_request=pull_request, cwd=cwd)
    base = payload.get("base", {})
    head = payload.get("head", {})
    if not isinstance(base, dict) or not isinstance(head, dict):
        raise ValueError("pull-request payload is missing base/head metadata")
    base_sha = str(base.get("sha", "")).strip().lower()
    head_sha = str(head.get("sha", "")).strip().lower()
    if not base_sha or not head_sha:
        raise ValueError("pull-request payload is missing base/head SHAs")
    return {
        "repository": repository,
        "pull_request": pull_request,
        "title": str(payload.get("title", "")),
        "body": str(payload.get("body") or ""),
        "author": str((payload.get("user") or {}).get("login", "")),
        "base_sha": base_sha,
        "head_sha": head_sha,
        "base_ref": str(base.get("ref", "")),
        "head_ref": str(head.get("ref", "")),
        "url": str(payload.get("html_url", "")),
        "draft": bool(payload.get("draft", False)),
    }


def _review_event_metadata(
    *,
    target_repo: Path,
    repository: str,
    pull_request: int,
    base_sha: str,
    head_sha: str,
    review_identity: str = "",
    requested_by_session_id: str = "",
    reviewer_session_id: str = "",
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    metadata: dict[str, Any] = {
        "repository_role": "target",
        "target_repository": str(target_repo),
        "pull_request": pull_request,
        "base_sha": base_sha,
        "head_sha": head_sha,
    }
    if review_identity:
        metadata["review_identity"] = review_identity
    if requested_by_session_id:
        metadata["requested_by_session_id"] = requested_by_session_id
    if reviewer_session_id:
        metadata["reviewer_session_id"] = reviewer_session_id
    if extra:
        metadata.update(extra)
    return metadata


def _append_review_event(
    *,
    event_store: EventStore,
    event_type: EventType,
    session_id: str,
    target_repo: Path,
    repository: str,
    branch: str,
    task_id: str,
    result: str,
    pull_request: int,
    head_sha: str,
    metadata: dict[str, Any],
    files_affected: tuple[str, ...] = (),
    reviewer_identifier: str = "",
) -> None:
    event_store.append(
        MonitoringEvent.create(
            event_type=event_type,
            session_id=session_id,
            repository=repository,
            branch=branch,
            task_identifier=task_id,
            result=result,
            reviewer_identifier=reviewer_identifier,
            commit_or_pr_reference=f"pr:{pull_request}",
            files_affected=files_affected,
            metadata={
                **metadata,
                "target_repository": str(target_repo),
                "head_sha": head_sha,
            },
        )
    )


def _decode_review_state(comment_body: str) -> dict[str, Any] | None:
    from harness.ai_review.codex import _decode_final_state

    return _decode_final_state(comment_body)


def _render_review_summary(bundle: dict[str, Any]) -> str:
    from harness.ai_review.codex import _render_summary

    return _render_summary(bundle)


def _run_import_smoke_test(*, source_root: Path, runtime_root: Path) -> dict[str, str]:
    smoke_root = runtime_root / "smoke-import"
    _copy_runtime_sources(source_root=source_root, destination_root=smoke_root)
    completed = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import harness.execution_plane.runtime as runtime;"
                "import json;"
                "print(json.dumps({'module': runtime.__name__}, sort_keys=True))"
            ),
        ],
        cwd=smoke_root,
        check=False,
        capture_output=True,
        text=True,
        env={
            **os.environ,
            "PYTHONPATH": str(smoke_root),
        },
    )
    if completed.returncode != 0:
        raise ValueError(
            "execution-plane copied-runtime import smoke test failed: "
            + (completed.stderr.strip() or completed.stdout.strip() or "unknown error")
        )
    payload = json.loads(completed.stdout.strip())
    return {
        "module": str(payload.get("module", "")),
        "root": str(smoke_root),
    }


def _runtime_event_store(runtime_root: Path) -> EventStore:
    return EventStore(runtime_root / "monitoring" / "events.jsonl")


def _runtime_task_store(runtime_root: Path) -> TaskStore:
    return TaskStore(runtime_root / "tasks")


def _runtime_review_engine(runtime_root: Path) -> ReviewEngine:
    return ReviewEngine(
        classifier=RiskClassifier(_repo_root() / "config" / "risk_rules.yaml"),
        output_directory=runtime_root / "reviews",
        event_store=_runtime_event_store(runtime_root),
    )


def _changed_files(repo: Path, *, base_ref: str, head_ref: str) -> tuple[Any, ...]:
    return parse_unified_diff(diff_between(repo, base_ref, head_ref))


def _learning_branch(repo: Path, *, head_ref: str) -> str:
    branch = run_git(repo, "branch", "--show-current")
    return branch or head_ref


def _question_payloads(questions: tuple[Any, ...]) -> list[dict[str, Any]]:
    return [asdict(question) for question in questions]


def _load_answers_map(path: Path | str) -> dict[str, str]:
    raw = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("answers file must contain a JSON object keyed by question identifier")
    return {str(key): str(value) for key, value in raw.items()}


def _task_record_for_learning(
    *,
    task_id: str,
    student_id: str,
    repository: Path,
    base_ref: str,
    head_ref: str,
    changed_files: tuple[Any, ...],
    branch: str,
    commit: str,
    risk_label: str,
    risk_matches: tuple[dict[str, str], ...],
    questions: tuple[Any, ...],
    completed: bool,
    verification_action: str = "",
    verification_result: str = "",
    evidence_references: tuple[str, ...] = (),
    answers: list[dict[str, str]] | None = None,
    answer_evaluations: list[dict[str, Any]] | None = None,
) -> TaskRecord:
    return TaskRecord(
        task_id=task_id,
        original_request=(
            "Runtime execution-plane learning review for "
            f"{repository.name} ({base_ref}...{head_ref})"
        ),
        repository=str(repository),
        student_identifier=student_id,
        session_id=f"runtime-learning-{task_id}",
        repository_role="target",
        state=(
            TaskState.AWAITING_MENTOR_REVIEW if completed else TaskState.AWAITING_LEARNING_REVIEW
        ),
        risk_level=risk_label,
        implementation={
            "changed_files": [change.path for change in changed_files],
            "base_reference": resolve_commit(repository, base_ref),
            "head_reference": commit,
            "branch": branch,
            "risk_classification": risk_label,
            "risk_matches": list(risk_matches),
        },
        learning_review={
            "questions": _question_payloads(questions),
            "answers": answers or [],
            "follow_up_questions": [],
            "answer_evaluations": answer_evaluations or [],
            "practical_challenge": {
                "challenge": verification_action,
                "student_response": verification_result,
                "evidence_references": list(evidence_references),
                "passed": completed,
            },
            "completed": completed,
        },
    )


def _prepare_learning_review(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    student_id: str,
    base_ref: str,
    head_ref: str,
) -> dict[str, Any]:
    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    engine = _runtime_review_engine(root)
    changes = _changed_files(repo, base_ref=base_ref, head_ref=head_ref)
    _sections, questions, assessment = engine.prepare(changes)
    commit = resolve_commit(repo, head_ref)
    branch = _learning_branch(repo, head_ref=head_ref)
    record = _task_record_for_learning(
        task_id=task_id,
        student_id=student_id,
        repository=repo,
        base_ref=base_ref,
        head_ref=head_ref,
        changed_files=changes,
        branch=branch,
        commit=commit,
        risk_label=assessment.risk.label,
        risk_matches=tuple(match.to_dict() for match in assessment.matches),
        questions=questions,
        completed=False,
    )
    task_json, task_markdown = _runtime_task_store(root).save(record)
    return {
        "status": "prepared",
        "task_id": task_id,
        "student_id": student_id,
        "repository": str(repo),
        "branch": branch,
        "commit": commit,
        "risk": assessment.risk.label,
        "changed_files": [change.path for change in changes],
        "questions": _question_payloads(questions),
        "task_record": str(task_json),
        "task_report": str(task_markdown),
    }


def _complete_learning_review(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    student_id: str,
    base_ref: str,
    head_ref: str,
    answers_file: Path | str,
    verification_action: str,
    verification_result: str,
    evidence: Path | str,
) -> dict[str, Any]:
    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    engine = _runtime_review_engine(root)
    changes = _changed_files(repo, base_ref=base_ref, head_ref=head_ref)
    _sections, questions, assessment = engine.prepare(changes)
    answers_map = _load_answers_map(answers_file)
    ordered_answers = tuple(answers_map[question.identifier] for question in questions)
    evidence_path = str(Path(evidence).expanduser().resolve())
    branch = _learning_branch(repo, head_ref=head_ref)
    commit = resolve_commit(repo, head_ref)
    record, record_path, report_path = engine.complete(
        ReviewRequest(
            task_identifier=task_id,
            student_identifier=student_id,
            repository=str(repo),
            branch=branch,
            commit=commit,
            changes=changes,
            answers=ordered_answers,
            verification_action=verification_action,
            verification_result=verification_result,
            evidence_references=(evidence_path,),
        )
    )
    task_record = _task_record_for_learning(
        task_id=task_id,
        student_id=student_id,
        repository=repo,
        base_ref=base_ref,
        head_ref=head_ref,
        changed_files=changes,
        branch=branch,
        commit=commit,
        risk_label=assessment.risk.label,
        risk_matches=tuple(match.to_dict() for match in assessment.matches),
        questions=record.questions,
        completed=True,
        verification_action=verification_action,
        verification_result=verification_result,
        evidence_references=(evidence_path,),
        answers=[
            {"question_id": answer.question_id, "answer": answer.answer}
            for answer in record.answers
        ],
        answer_evaluations=[asdict(evaluation) for evaluation in record.answer_evaluations],
    )
    task_json, task_markdown = _runtime_task_store(root).save(task_record)
    return {
        "status": "ok",
        "task_id": task_id,
        "review_id": record.review_id,
        "completion_state": record.completion_state,
        "risk": record.risk_classification,
        "record_path": str(record_path),
        "report_path": str(report_path),
        "task_record": str(task_json),
        "task_report": str(task_markdown),
    }


def _record_pre_pr_check_event(
    *,
    event_store: EventStore,
    event_type: EventType,
    target_repo: Path,
    task_id: str,
    status: str,
    blockers: tuple[str, ...] | list[str],
    required_checks: tuple[str, ...],
) -> None:
    event_store.append(
        MonitoringEvent.create(
            event_type=event_type,
            session_id=f"pre-pr-check-{task_id}",
            repository=str(target_repo),
            branch=run_git(target_repo, "branch", "--show-current"),
            task_identifier=task_id,
            result=status,
            status=status,
            policy_status="blocked" if status == "blocked" else "not_evaluated",
            metadata={
                "repository_role": "target",
                "target_repository": str(target_repo),
                "task_id": task_id,
                "status": status,
                "blockers": list(blockers),
                "required_checks": list(required_checks),
            },
        )
    )


def _pre_pr_check_payload(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    required_checks: tuple[str, ...],
    observed_checks: dict[str, str],
    lifecycle_state: str,
    learning_review_completed: bool,
    monitoring_chain_verified: bool,
    contamination_report: dict[str, Any],
) -> tuple[int, dict[str, Any]]:
    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    event_store = _runtime_event_store(root)
    _record_pre_pr_check_event(
        event_store=event_store,
        event_type=EventType.PRE_PR_CHECK_STARTED,
        target_repo=repo,
        task_id=task_id,
        status="started",
        blockers=(),
        required_checks=required_checks,
    )
    blockers = tuple(
        pre_pr_blockers(
            target_repo=repo,
            runtime_root=root,
            task_id=task_id,
            required_checks=required_checks,
            observed_checks=observed_checks,
            lifecycle_state=lifecycle_state,
            learning_review_completed=learning_review_completed,
            monitoring_chain_verified=monitoring_chain_verified,
            contamination_report=contamination_report,
        )
    )
    status = "ok" if not blockers else "blocked"
    _record_pre_pr_check_event(
        event_store=event_store,
        event_type=EventType.PRE_PR_CHECK_COMPLETED,
        target_repo=repo,
        task_id=task_id,
        status=status,
        blockers=blockers,
        required_checks=required_checks,
    )
    return (
        0 if not blockers else 1,
        {
            "status": status,
            "task_id": task_id,
            "blockers": list(blockers),
        },
    )


def _read_recorded_provider_responses(
    path: Path | str,
    *,
    minimum_count: int,
    target_repo: Path,
) -> ReviewProvider:
    source = Path(path).expanduser()
    if source.resolve().is_relative_to(target_repo.resolve()):
        raise ValueError("recorded provider responses must be stored outside the target repository")
    raw = json.loads(source.read_text(encoding="utf-8"))
    responses = raw.get("responses") if isinstance(raw, dict) else raw
    if not isinstance(responses, list) or not responses:
        raise ValueError("recorded provider responses must contain at least one response")
    duplicated = list(responses)
    while len(duplicated) < minimum_count:
        duplicated.append(duplicated[-1])
    return _ReplayRecordedProvider(RecordedReviewProvider(duplicated))


def prepare_runtime_environment(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    harness_revision: str,
    reviewer_authorizations: dict[str, frozenset[str]] | None = None,
) -> RuntimeEnvironment:
    """Create an external execution-plane runtime rooted outside the target checkout."""

    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    root.mkdir(parents=True, exist_ok=True)
    repo_root = _repo_root()
    manifest = build_execution_plane_manifest(
        source_root=repo_root,
        harness_revision=harness_revision,
        allowlist=_runtime_allowlist(),
    )
    manifest_path = write_private_text(
        root / "manifest" / "execution-plane.json", _canonical_json(manifest)
    )
    task_store = TaskStore(root / "tasks")
    event_store = EventStore(root / "monitoring" / "events.jsonl")
    classifier = RiskClassifier(repo_root / "config" / "risk_rules.yaml")
    task_service = TaskService(
        store=task_store,
        event_store=event_store,
        classifier=classifier,
        reviewer_authorizations=reviewer_authorizations,
    )
    return RuntimeEnvironment(
        target_repo=repo,
        runtime_root=root,
        harness_revision=harness_revision.strip().lower(),
        manifest=manifest,
        manifest_path=manifest_path,
        task_store=task_store,
        event_store=event_store,
        task_service=task_service,
    )


def _is_runtime_contamination_path(path: str) -> bool:
    normalized = PurePosixPath(path).as_posix().removeprefix("./")
    candidate = PurePosixPath(normalized)
    if candidate.parts and candidate.parts[0].casefold() in _RUNTIME_TOP_LEVEL_CONTAMINATION_PARTS:
        return True
    lowered_parts = {part.casefold() for part in candidate.parts}
    return bool(lowered_parts & _LOCAL_APPROVAL_STATE_PARTS)


def validate_product_diff(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    changed_paths: tuple[str, ...] | list[str],
) -> dict[str, Any]:
    """Reject runtime/control-plane contamination from the product diff."""

    _require_external_runtime_root(target_repo, runtime_root)
    violations: list[str] = []
    for raw_path in changed_paths:
        normalized = PurePosixPath(str(raw_path)).as_posix().removeprefix("./")
        if _is_runtime_contamination_path(normalized):
            violations.append(normalized)
    return {
        "status": "blocked" if violations else "ok",
        "violations": sorted(dict.fromkeys(violations)),
    }


def pre_pr_blockers(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    required_checks: tuple[str, ...] | list[str],
    observed_checks: dict[str, str],
    lifecycle_state: str = "",
    learning_review_completed: bool = False,
    monitoring_chain_verified: bool = False,
    contamination_report: dict[str, Any] | None = None,
) -> tuple[str, ...]:
    """Return fail-closed blockers before a PR can proceed."""

    del task_id
    _require_external_runtime_root(target_repo, runtime_root)
    blockers: list[str] = []
    if lifecycle_state != "awaiting_mentor_review":
        blockers.append("lifecycle_not_ready")
    if not learning_review_completed:
        blockers.append("learning_review_incomplete")
    if not monitoring_chain_verified:
        blockers.append("monitoring_chain_unverified")
    report = contamination_report or {"violations": ["not-validated"]}
    if report.get("violations"):
        blockers.append("product_diff_contaminated")
    for check in required_checks:
        conclusion = observed_checks.get(check)
        if conclusion is None:
            blockers.append(f"required_check_missing:{check}")
        elif str(conclusion).casefold() != "success":
            blockers.append(f"required_check_not_success:{check}")
    return tuple(blockers)


def pr_creation_blockers(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    required_checks: tuple[str, ...] | list[str],
    observed_checks: dict[str, str],
    lifecycle_state: str = "",
    learning_review_completed: bool = False,
    monitoring_chain_verified: bool = False,
    contamination_report: dict[str, Any] | None = None,
) -> tuple[str, ...]:
    """Return fail-closed blockers before a draft pull request can be created."""

    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    blockers = list(
        pre_pr_blockers(
            target_repo=repo,
            runtime_root=root,
            task_id=task_id,
            required_checks=tuple(
                check
                for check in required_checks
                if str(check).strip().casefold() not in _POST_PR_REVIEW_CHECKS
            ),
            observed_checks=observed_checks,
            lifecycle_state=lifecycle_state,
            learning_review_completed=learning_review_completed,
            monitoring_chain_verified=monitoring_chain_verified,
            contamination_report=contamination_report,
        )
    )
    if run_git(repo, "status", "--porcelain=v1", "--untracked-files=all"):
        blockers.append("local_checkout_dirty")
    return tuple(dict.fromkeys(blockers))


def run_recorded_reviewer_job(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    event: PullRequestEvent,
    provider_responses: Path | str,
    github_api: Any,
    task_identifier: str,
) -> dict[str, Any]:
    """Run a deterministic recorded review and publish evidence through a narrow adapter."""

    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    root.mkdir(parents=True, exist_ok=True)
    event_store = EventStore(root / "monitoring" / "events.jsonl")
    audit = AuditTrail(event=event, event_store=event_store, task_identifier=task_identifier)
    audit.record(
        EventType.PULL_REQUEST_DETECTED.value, pull_request=event.number, action=event.action
    )
    audit.record(EventType.AI_REVIEW_STARTED.value, pull_request=event.number)
    audit.record(EventType.AI_BASE_SHA_RECORDED.value, sha=event.base_sha, ref=event.base_ref)
    audit.record(EventType.AI_HEAD_SHA_RECORDED.value, sha=event.head_sha, ref=event.head_ref)

    diff = collect_full_diff(
        repo,
        base_sha=event.base_sha,
        head_sha=event.head_sha,
        maximum_chunk_characters=6_000,
    )
    audit.record(
        EventType.AI_FULL_DIFF_COLLECTED.value,
        sha256=diff.full_diff_sha256,
        files=len(diff.files),
        bytes=diff.full_diff_bytes,
    )
    classifier = RiskClassifier(_repo_root() / "config" / "risk_rules.yaml")
    risk = classifier.classify([])
    if diff.files:
        from harness.git import ChangedFile

        grouped: dict[str, list[str]] = {}
        for chunk in diff.chunks:
            grouped.setdefault(chunk.path, []).append(chunk.content)
        changes = tuple(
            ChangedFile(
                path=file.path,
                old_path=file.old_path,
                patch="".join(grouped.get(file.path, [])),
            )
            for file in diff.files
        )
        risk = classifier.classify(changes)

    context = ReviewContext(
        repository_policy=_trusted_policy(_repo_root()),
        target_policy=_target_policy(repo, event.base_sha),
        task_specification=(f"Recorded execution-plane AI review for task {task_identifier}."),
        confirmed_plan=(
            f"Review the complete immutable diff from {event.base_sha} to {event.head_sha} "
            "without mutating source files."
        ),
        risk_classification=risk.risk.label,
        protected_path_result=json.dumps(risk.to_dict(), sort_keys=True),
        learning_review_status="recorded",
        test_evidence={
            "github_checks_at_review_start": github_api.get_ci_status(
                event.repository, event.head_sha
            ),
            "harness": _safe_harness_evidence({"status": "recorded", "head_sha": event.head_sha}),
        },
    )
    audit.record(
        EventType.AI_REVIEW_CONTEXT_COLLECTED.value, sources="trusted base and harness evidence"
    )
    request_count = 1 if not diff.chunks else math.ceil(len(diff.chunks) / 1) + 1
    provider = _read_recorded_provider_responses(
        provider_responses,
        minimum_count=request_count,
        target_repo=repo,
    )
    service = AIReviewService(provider, chunks_per_request=1)
    review = service.review(
        PullRequestMetadata(
            number=event.number,
            repository=event.repository,
            title=event.title,
            body=event.body,
        ),
        diff,
        context,
        required_reviewer_group=_reviewer_group(risk),
    )
    audit.record(EventType.AI_REVIEW_COMPLETED.value, result=review.result.value)
    for finding in review.findings:
        audit.record(
            (
                EventType.AI_BLOCKING_FINDING_CREATED.value
                if finding.blocking
                else EventType.AI_FINDING_CREATED.value
            ),
            finding_id=finding.identity,
            severity=finding.severity.value,
            path=finding.path,
            result=finding.state.value,
        )
    review_dir = root / "reviews"
    review_json, review_markdown = write_review_artifacts(review_dir, review, diff)
    summary = render_summary_comment(review, diff)
    github_api.upsert_summary_comment(event.repository, event.number, summary)
    audit.record(EventType.AI_SUMMARY_POSTED.value, head_sha=event.head_sha)
    inline_count = github_api.post_inline_findings(
        event.repository,
        event.number,
        event.head_sha,
        review.findings,
        changed_new_lines(diff),
    )
    for _ in range(inline_count):
        audit.record(EventType.AI_INLINE_COMMENT_POSTED.value, head_sha=event.head_sha)
    github_api.create_check(event.repository, review, summary)
    audit.record(
        (
            EventType.AI_CHECK_PASSED.value
            if review.check_conclusion.value == "success"
            else EventType.AI_CHECK_FAILED.value
        ),
        head_sha=event.head_sha,
    )
    audit_path = audit.write(review_dir / f"pr-{event.number}-{event.head_sha}-audit.jsonl")
    return {
        "status": "completed",
        "pull_request": event.number,
        "review_result": review.result.value,
        "required_reviewer_group": review.required_reviewer_group,
        "review_artifacts": {
            "json": str(review_json),
            "markdown": str(review_markdown),
            "audit": str(audit_path),
        },
        "monitoring_event_types": [entry["event_type"] for entry in audit.events],
        "source_mutation_attempted": False,
    }


def _review_handoff_payload(
    *,
    runtime_root: Path,
    target_repo: Path,
    repository: str,
    pull_request: int,
    base_sha: str,
    head_sha: str,
    base_ref: str,
    head_ref: str,
    title: str,
    body: str,
    task_id: str,
    requested_by_session_id: str,
    reviewer_role: str = "Automated Reviewer",
) -> dict[str, Any]:
    review_request = f"review {repository} PR #{pull_request}"
    payload = {
        "status": "prepared",
        "repository": repository,
        "pull_request": pull_request,
        "base_sha": base_sha,
        "head_sha": head_sha,
        "base_ref": base_ref,
        "head_ref": head_ref,
        "title": title,
        "body": body,
        "task_id": task_id,
        "requested_by_session_id": requested_by_session_id,
        "reviewer_role": reviewer_role,
        "review_request": review_request,
        "machine_handoff": {
            "kind": "agentic_review_request",
            "command": "review-prepare",
            "request": review_request,
            "repository": repository,
            "pull_request": pull_request,
            "base_sha": base_sha,
            "head_sha": head_sha,
            "task_id": task_id,
            "requested_by_session_id": requested_by_session_id,
            "reviewer_role": reviewer_role,
            "target_repository": str(target_repo),
        },
    }
    handoff_path = _write_runtime_json(
        _review_runtime_path(
            runtime_root,
            kind="handoffs",
            repository=repository,
            pull_request=pull_request,
            head_sha=head_sha,
        ),
        payload,
    )
    payload["handoff_path"] = str(handoff_path)
    return payload


def prepare_review_handoff(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    repository: str,
    pull_request: int,
    task_id: str,
    requested_by_session_id: str,
    base_sha: str = "",
    head_sha: str = "",
    base_ref: str = "",
    head_ref: str = "",
    title: str = "",
    body: str = "",
    automatic: bool = False,
) -> dict[str, Any]:
    adapter = _call_optional_review_adapter(
        "prepare",
        target_repo=target_repo,
        runtime_root=runtime_root,
        repository=repository,
        pull_request=pull_request,
        task_id=task_id,
        requested_by_session_id=requested_by_session_id,
        base_sha=base_sha,
        head_sha=head_sha,
        base_ref=base_ref,
        head_ref=head_ref,
        title=title,
        body=body,
        automatic=automatic,
    )
    if adapter is not _OPTIONAL_ADAPTER_MISSING:
        return dict(adapter)
    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    state = _pull_request_state(repository=repository, pull_request=pull_request, cwd=repo)
    payload = _review_handoff_payload(
        runtime_root=root,
        target_repo=repo,
        repository=repository,
        pull_request=pull_request,
        base_sha=base_sha or str(state["base_sha"]),
        head_sha=head_sha or str(state["head_sha"]),
        base_ref=base_ref or str(state["base_ref"]),
        head_ref=head_ref or str(state["head_ref"]),
        title=title or str(state["title"]),
        body=body or str(state["body"]),
        task_id=task_id,
        requested_by_session_id=requested_by_session_id,
    )
    event_store = _runtime_event_store(root)
    metadata = _review_event_metadata(
        target_repo=repo,
        repository=repository,
        pull_request=pull_request,
        base_sha=str(payload["base_sha"]),
        head_sha=str(payload["head_sha"]),
        requested_by_session_id=requested_by_session_id,
        extra={
            "base_ref": str(payload["base_ref"]),
            "head_ref": str(payload["head_ref"]),
            "handoff_path": str(payload["handoff_path"]),
        },
    )
    _append_review_event(
        event_store=event_store,
        event_type=EventType.REVIEW_PREPARED,
        session_id=f"review-prepare-pr-{pull_request}",
        target_repo=repo,
        repository=repository,
        branch=str(payload["head_ref"]),
        task_id=task_id,
        result="prepared",
        pull_request=pull_request,
        head_sha=str(payload["head_sha"]),
        metadata=metadata,
    )
    _append_review_event(
        event_store=event_store,
        event_type=EventType.REVIEW_AGENT_REQUESTED,
        session_id=f"review-prepare-pr-{pull_request}",
        target_repo=repo,
        repository=repository,
        branch=str(payload["head_ref"]),
        task_id=task_id,
        result="requested",
        pull_request=pull_request,
        head_sha=str(payload["head_sha"]),
        metadata=metadata,
    )
    return payload


def record_review_agent_started(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    repository: str,
    pull_request: int,
    head_sha: str,
    requested_by_session_id: str,
    reviewer_session_id: str,
) -> dict[str, Any]:
    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    if reviewer_session_id == requested_by_session_id:
        raise ValueError("reviewer agent must use a different session from implementation")
    request_path = (
        root
        / "harness-evidence"
        / repository.replace("/", "--")
        / f"pr-{pull_request}"
        / "requests"
        / f"review-request-{head_sha}.json"
    )
    if not request_path.is_file():
        raise ValueError("review agent cannot start without a prepared objective request")
    context = json.loads(request_path.read_text(encoding="utf-8"))
    pull_request_context = context.get("pull_request", {})
    reference = pull_request_context.get("pull_request", {})
    base_sha = str(reference.get("base_sha", ""))
    _append_review_event(
        event_store=_runtime_event_store(root),
        event_type=EventType.REVIEW_AGENT_STARTED,
        session_id=f"review-agent-{reviewer_session_id}",
        target_repo=repo,
        repository=repository,
        branch="",
        task_id=task_id,
        result="started",
        pull_request=pull_request,
        head_sha=head_sha,
        reviewer_identifier=reviewer_session_id,
        metadata=_review_event_metadata(
            target_repo=repo,
            repository=repository,
            pull_request=pull_request,
            base_sha=base_sha,
            head_sha=head_sha,
            requested_by_session_id=requested_by_session_id,
            reviewer_session_id=reviewer_session_id,
            extra={"request_path": str(request_path)},
        ),
    )
    return {
        "status": "started",
        "repository": repository,
        "pull_request": pull_request,
        "head_sha": head_sha,
        "reviewer_role": "Automated Reviewer",
        "reviewer_session_id": reviewer_session_id,
    }


def record_review_fix_pushed(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    repository: str,
    pull_request: int,
    previous_head_sha: str,
    new_head_sha: str,
    commit_sha: str,
) -> dict[str, Any]:
    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    _append_review_event(
        event_store=_runtime_event_store(root),
        event_type=EventType.REVIEW_FIX_PUSHED,
        session_id=f"review-fix-pr-{pull_request}",
        target_repo=repo,
        repository=repository,
        branch=run_git(repo, "branch", "--show-current"),
        task_id=task_id,
        result="pushed",
        pull_request=pull_request,
        head_sha=new_head_sha,
        metadata=_review_event_metadata(
            target_repo=repo,
            repository=repository,
            pull_request=pull_request,
            base_sha="",
            head_sha=new_head_sha,
            extra={
                "previous_head_sha": previous_head_sha,
                "commit_sha": commit_sha,
            },
        ),
    )
    stale = mark_review_stale(
        target_repo=repo,
        runtime_root=root,
        task_id=task_id,
        repository=repository,
        pull_request=pull_request,
        previous_head_sha=previous_head_sha,
        new_head_sha=new_head_sha,
    )
    return {"status": "recorded", "commit_sha": commit_sha, "stale": stale}


def record_review_bundle(
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
    adapter = _call_optional_review_adapter(
        "record",
        target_repo=target_repo,
        runtime_root=runtime_root,
        task_id=task_id,
        repository=repository,
        pull_request=pull_request,
        bundle=bundle,
        requested_by_session_id=requested_by_session_id,
        reviewer_session_id=reviewer_session_id,
    )
    if adapter is not _OPTIONAL_ADAPTER_MISSING:
        return dict(adapter)
    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    source = Path(bundle).expanduser().resolve()
    if source.is_relative_to(repo):
        raise ValueError("review bundle must be stored outside the target repository")
    payload = json.loads(source.read_text(encoding="utf-8"))
    head_sha = str(payload.get("head_sha") or payload.get("review_state", {}).get("head_sha") or "")
    base_sha = str(payload.get("base_sha", ""))
    if not head_sha:
        raise ValueError("review bundle is missing head_sha")
    stored = _write_runtime_json(
        _review_runtime_path(
            root,
            kind="records",
            repository=repository,
            pull_request=pull_request,
            head_sha=head_sha,
        ),
        payload,
    )
    metadata = _review_event_metadata(
        target_repo=repo,
        repository=repository,
        pull_request=pull_request,
        base_sha=base_sha,
        head_sha=head_sha,
        review_identity=str(payload.get("review_identity", "")),
        requested_by_session_id=requested_by_session_id,
        reviewer_session_id=reviewer_session_id,
        extra={
            "record_path": str(stored),
            "review_result": str(payload.get("result", "")),
            "finding_count": len(payload.get("findings", [])),
            "complete": bool(payload.get("complete", False)),
        },
    )
    _append_review_event(
        event_store=_runtime_event_store(root),
        event_type=EventType.REVIEW_COMPLETED,
        session_id=f"review-record-pr-{pull_request}",
        target_repo=repo,
        repository=repository,
        branch=str(payload.get("review_state", {}).get("head_ref", "")),
        task_id=task_id,
        result=str(payload.get("result", "completed")),
        pull_request=pull_request,
        head_sha=head_sha,
        metadata=metadata,
        files_affected=tuple(str(item) for item in payload.get("reviewed_files", [])),
    )
    return {
        "status": "recorded",
        "repository": repository,
        "pull_request": pull_request,
        "head_sha": head_sha,
        "record_path": str(stored),
        "review_result": str(payload.get("result", "")),
    }


def request_platform_review_on_github(
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
) -> dict[str, Any]:
    adapter = _call_optional_review_adapter(
        "request_github",
        target_repo=target_repo,
        runtime_root=runtime_root,
        task_id=task_id,
        repository=repository,
        pull_request=pull_request,
        expected_head=expected_head,
        request_publisher_login=request_publisher_login,
        request_publisher_id=request_publisher_id,
        request_publisher_type=request_publisher_type,
    )
    if adapter is _OPTIONAL_ADAPTER_MISSING:
        raise ValueError("trusted Codex platform-request adapter is not installed")
    return dict(adapter)


def publish_review_bundle(
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
    trusted_request_publisher_login: str,
    trusted_request_publisher_id: int,
    trusted_request_publisher_type: str = "Bot",
    platform_wait_seconds: int = 600,
    platform_poll_seconds: int = 5,
) -> tuple[int, dict[str, Any]]:
    adapter = _call_optional_review_adapter(
        "publish",
        target_repo=target_repo,
        runtime_root=runtime_root,
        task_id=task_id,
        repository=repository,
        pull_request=pull_request,
        bundle=bundle,
        trusted_reviewer_login=trusted_reviewer_login,
        trusted_reviewer_id=trusted_reviewer_id,
        trusted_reviewer_type=trusted_reviewer_type,
        trusted_request_publisher_login=trusted_request_publisher_login,
        trusted_request_publisher_id=trusted_request_publisher_id,
        trusted_request_publisher_type=trusted_request_publisher_type,
        platform_wait_seconds=platform_wait_seconds,
        platform_poll_seconds=platform_poll_seconds,
    )
    if adapter is not _OPTIONAL_ADAPTER_MISSING:
        result = dict(adapter)
        return (0 if str(result.get("status")) == "published" else 1, result)
    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    payload = json.loads(Path(bundle).expanduser().read_text(encoding="utf-8"))
    head_sha = str(payload.get("head_sha", "")).strip().lower()
    if not head_sha:
        raise ValueError("review bundle is missing head_sha")
    state = _pull_request_state(repository=repository, pull_request=pull_request, cwd=repo)
    if str(state["head_sha"]) != head_sha:
        stale_payload = mark_review_stale(
            target_repo=repo,
            runtime_root=root,
            task_id=task_id,
            repository=repository,
            pull_request=pull_request,
            previous_head_sha=head_sha,
            new_head_sha=str(state["head_sha"]),
        )
        return 1, {
            "status": "stale",
            "repository": repository,
            "pull_request": pull_request,
            "head_sha": str(state["head_sha"]),
            "stale": stale_payload,
        }
    summary = _render_review_summary(payload)
    actor = _gh_actor_login(cwd=repo)
    comments = _run_gh_api(
        path=f"repos/{repository}/issues/{pull_request}/comments?per_page=100",
        cwd=repo,
    )
    existing_id = None
    if isinstance(comments, list):
        for comment in reversed(comments):
            if not isinstance(comment, dict):
                continue
            user = comment.get("user") or {}
            body = str(comment.get("body", ""))
            if (
                isinstance(user, dict)
                and str(user.get("login", "")) == actor
                and _REVIEW_STATE_MARKER in body
            ):
                existing_id = int(comment["id"])
                break
    if existing_id is None:
        summary_comment = _run_gh_api(
            path=f"repos/{repository}/issues/{pull_request}/comments",
            method="POST",
            payload={"body": summary},
            cwd=repo,
        )
    else:
        summary_comment = _run_gh_api(
            path=f"repos/{repository}/issues/comments/{existing_id}",
            method="PATCH",
            payload={"body": summary},
            cwd=repo,
        )
    review_comment = _run_gh_api(
        path=f"repos/{repository}/pulls/{pull_request}/reviews",
        method="POST",
        payload={
            "event": "COMMENT",
            "body": (
                "Automated Reviewer recorded the latest Agentic Review state. "
                "Human review remains required."
            ),
        },
        cwd=repo,
    )
    dispatch = _run_gh_api(
        path=f"repos/{repository}/dispatches",
        method="POST",
        payload={
            "event_type": "agentic_review_recorded",
            "client_payload": {
                "repository": repository,
                "pull_request": pull_request,
                "head_sha": head_sha,
                "task_id": task_id,
                "review_identity": str(payload.get("review_identity", "")),
                "review_result": str(payload.get("result", "")),
            },
        },
        cwd=repo,
    )
    metadata = _review_event_metadata(
        target_repo=repo,
        repository=repository,
        pull_request=pull_request,
        base_sha=str(payload.get("base_sha", "")),
        head_sha=head_sha,
        review_identity=str(payload.get("review_identity", "")),
        extra={
            "review_result": str(payload.get("result", "")),
            "complete": bool(payload.get("complete", False)),
            "summary_comment_id": int(summary_comment["id"])
            if isinstance(summary_comment, dict)
            else 0,
            "review_comment_id": int(review_comment["id"])
            if isinstance(review_comment, dict)
            else 0,
            "repository_dispatch": "agentic_review_recorded",
            "dispatched": dispatch is None,
        },
    )
    _append_review_event(
        event_store=_runtime_event_store(root),
        event_type=EventType.REVIEW_PUBLISHED,
        session_id=f"review-publish-pr-{pull_request}",
        target_repo=repo,
        repository=repository,
        branch=str(payload.get("review_state", {}).get("head_ref", state["head_ref"])),
        task_id=task_id,
        result="published",
        pull_request=pull_request,
        head_sha=head_sha,
        metadata=metadata,
    )
    return 0, {
        "status": "published",
        "repository": repository,
        "pull_request": pull_request,
        "head_sha": head_sha,
        "summary_comment_id": metadata["summary_comment_id"],
        "review_comment_id": metadata["review_comment_id"],
        "dispatch_event": "agentic_review_recorded",
    }


def validate_review_state_on_github(
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
) -> tuple[int, dict[str, Any]]:
    adapter = _call_optional_review_adapter(
        "validate_github",
        target_repo=target_repo,
        runtime_root=runtime_root,
        task_id=task_id,
        repository=repository,
        pull_request=pull_request,
        expected_head=expected_head,
        trusted_reviewer_login=trusted_reviewer_login,
        trusted_reviewer_id=trusted_reviewer_id,
        trusted_reviewer_type=trusted_reviewer_type,
        trusted_request_publisher_login=trusted_request_publisher_login,
        trusted_request_publisher_id=trusted_request_publisher_id,
        trusted_request_publisher_type=trusted_request_publisher_type,
    )
    if adapter is not _OPTIONAL_ADAPTER_MISSING:
        result = dict(adapter)
        return (0 if str(result.get("status")) == "ok" else 1, result)
    raise ValueError("trusted Codex review-state adapter is not installed")


def record_review_response(
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
    evidence: tuple[str, ...] | list[str],
    responder_role: str,
) -> dict[str, Any]:
    adapter = _call_optional_review_adapter(
        "respond",
        target_repo=target_repo,
        runtime_root=runtime_root,
        task_id=task_id,
        repository=repository,
        pull_request=pull_request,
        head_sha=head_sha,
        finding_id=finding_id,
        decision=decision,
        rationale=rationale,
        evidence=tuple(evidence),
        responder_role=responder_role,
    )
    if adapter is not _OPTIONAL_ADAPTER_MISSING:
        return dict(adapter)
    if decision.strip().casefold() == "reject":
        if not rationale.strip():
            raise ValueError("rationale is required when rejecting a finding")
        if not tuple(evidence):
            raise ValueError("evidence is required when rejecting a finding")
    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    payload = {
        "repository": repository,
        "pull_request": pull_request,
        "head_sha": head_sha,
        "finding_id": finding_id,
        "decision": decision,
        "rationale": rationale,
        "evidence": list(evidence),
        "responder_role": responder_role,
        "task_id": task_id,
    }
    stored = _write_runtime_json(
        _review_runtime_path(
            root,
            kind="responses",
            repository=repository,
            pull_request=pull_request,
            head_sha=head_sha,
        ),
        payload,
    )
    _append_review_event(
        event_store=_runtime_event_store(root),
        event_type=EventType.REVIEW_RESPONSE_ADDED,
        session_id=f"review-response-pr-{pull_request}",
        target_repo=repo,
        repository=repository,
        branch="",
        task_id=task_id,
        result=decision,
        pull_request=pull_request,
        head_sha=head_sha,
        metadata=_review_event_metadata(
            target_repo=repo,
            repository=repository,
            pull_request=pull_request,
            base_sha="",
            head_sha=head_sha,
            extra={
                "finding_id": finding_id,
                "decision": decision,
                "response_path": str(stored),
                "evidence_count": len(tuple(evidence)),
                "actor_role": responder_role,
            },
        ),
    )
    return {
        "status": "recorded",
        "repository": repository,
        "pull_request": pull_request,
        "head_sha": head_sha,
        "response_path": str(stored),
    }


def mark_review_stale(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    repository: str,
    pull_request: int,
    previous_head_sha: str,
    new_head_sha: str,
) -> dict[str, Any]:
    adapter = _call_optional_review_adapter(
        "stale",
        target_repo=target_repo,
        runtime_root=runtime_root,
        task_id=task_id,
        repository=repository,
        pull_request=pull_request,
        previous_head_sha=previous_head_sha,
        new_head_sha=new_head_sha,
    )
    if adapter is not _OPTIONAL_ADAPTER_MISSING:
        return dict(adapter)
    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    payload = {
        "status": "stale",
        "repository": repository,
        "pull_request": pull_request,
        "previous_head_sha": previous_head_sha,
        "new_head_sha": new_head_sha,
        "task_id": task_id,
        "rerun_request": f"review {repository} PR #{pull_request}",
    }
    stored = _write_runtime_json(
        _review_runtime_path(
            root,
            kind="stale",
            repository=repository,
            pull_request=pull_request,
            head_sha=new_head_sha,
        ),
        payload,
    )
    metadata = _review_event_metadata(
        target_repo=repo,
        repository=repository,
        pull_request=pull_request,
        base_sha="",
        head_sha=new_head_sha,
        extra={
            "previous_head_sha": previous_head_sha,
            "stale_path": str(stored),
        },
    )
    event_store = _runtime_event_store(root)
    _append_review_event(
        event_store=event_store,
        event_type=EventType.REVIEW_STALE,
        session_id=f"review-stale-pr-{pull_request}",
        target_repo=repo,
        repository=repository,
        branch="",
        task_id=task_id,
        result="stale",
        pull_request=pull_request,
        head_sha=new_head_sha,
        metadata=metadata,
    )
    _append_review_event(
        event_store=event_store,
        event_type=EventType.REVIEW_RERUN_REQUESTED,
        session_id=f"review-stale-pr-{pull_request}",
        target_repo=repo,
        repository=repository,
        branch="",
        task_id=task_id,
        result="requested",
        pull_request=pull_request,
        head_sha=new_head_sha,
        metadata=metadata,
    )
    return {
        **payload,
        "stale_path": str(stored),
    }


def create_pull_request_with_handoff(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    repository: str,
    title: str,
    body: str,
    base: str,
    head: str,
    requested_by_session_id: str,
    required_checks: tuple[str, ...] | list[str],
    observed_checks: dict[str, str],
    lifecycle_state: str,
    learning_review_completed: bool,
    monitoring_chain_verified: bool,
    contamination_report: dict[str, Any],
) -> tuple[int, dict[str, Any]]:
    adapter = _call_optional_review_adapter(
        "pr_create",
        target_repo=target_repo,
        runtime_root=runtime_root,
        task_id=task_id,
        repository=repository,
        title=title,
        body=body,
        base=base,
        head=head,
        requested_by_session_id=requested_by_session_id,
        required_checks=tuple(required_checks),
        observed_checks=observed_checks,
        lifecycle_state=lifecycle_state,
        learning_review_completed=learning_review_completed,
        monitoring_chain_verified=monitoring_chain_verified,
        contamination_report=contamination_report,
    )
    if adapter is not _OPTIONAL_ADAPTER_MISSING:
        result = dict(adapter)
        return (0 if str(result.get("status")) == "created" else 1, result)
    repo, root = _require_external_runtime_root(target_repo, runtime_root)
    blockers = list(
        pr_creation_blockers(
            target_repo=repo,
            runtime_root=root,
            task_id=task_id,
            required_checks=required_checks,
            observed_checks=observed_checks,
            lifecycle_state=lifecycle_state,
            learning_review_completed=learning_review_completed,
            monitoring_chain_verified=monitoring_chain_verified,
            contamination_report=contamination_report,
        )
    )
    if blockers:
        return 1, {
            "status": "blocked",
            "task_id": task_id,
            "repository": repository,
            "blockers": blockers,
        }
    created = _run_gh_api(
        path=f"repos/{repository}/pulls",
        method="POST",
        payload={
            "title": title,
            "body": body,
            "base": base,
            "head": head,
            "draft": True,
        },
        cwd=repo,
    )
    if not isinstance(created, dict):
        raise ValueError("pull-request creation returned invalid JSON")
    number = int(created["number"])
    head_sha = str((created.get("head") or {}).get("sha", "")).strip().lower()
    base_sha = str((created.get("base") or {}).get("sha", "")).strip().lower()
    head_ref = str((created.get("head") or {}).get("ref", head))
    base_ref = str((created.get("base") or {}).get("ref", base))
    if not head_sha:
        head_sha = resolve_commit(repo, "HEAD")
    metadata = _review_event_metadata(
        target_repo=repo,
        repository=repository,
        pull_request=number,
        base_sha=base_sha,
        head_sha=head_sha,
        requested_by_session_id=requested_by_session_id,
        extra={"pr_url": str(created.get("html_url", ""))},
    )
    event_store = _runtime_event_store(root)
    _append_review_event(
        event_store=event_store,
        event_type=EventType.PR_CREATED,
        session_id=f"pr-create-{task_id}",
        target_repo=repo,
        repository=repository,
        branch=head_ref,
        task_id=task_id,
        result="created",
        pull_request=number,
        head_sha=head_sha,
        metadata=metadata,
    )
    _append_review_event(
        event_store=event_store,
        event_type=EventType.PULL_REQUEST_OPENED,
        session_id=f"pr-create-{task_id}",
        target_repo=repo,
        repository=repository,
        branch=head_ref,
        task_id=task_id,
        result="opened",
        pull_request=number,
        head_sha=head_sha,
        metadata=metadata,
    )
    handoff = prepare_review_handoff(
        target_repo=repo,
        runtime_root=root,
        repository=repository,
        pull_request=number,
        task_id=task_id,
        requested_by_session_id=requested_by_session_id,
        base_sha=base_sha,
        head_sha=head_sha,
        base_ref=base_ref,
        head_ref=head_ref,
        title=title,
        body=body,
        automatic=True,
    )
    return 0, {
        "status": "created",
        "repository": repository,
        "pull_request": number,
        "url": str(created.get("html_url", "")),
        "head_sha": head_sha,
        "base_sha": base_sha,
        "handoff": handoff["machine_handoff"],
        "handoff_path": handoff["handoff_path"],
    }


def _governance_check_payload(
    *,
    target_repo: Path | str,
    runtime_root: Path | str,
    task_id: str,
    required_checks: tuple[str, ...],
    observed_checks: dict[str, str],
    lifecycle_state: str,
    learning_review_completed: bool,
    monitoring_chain_verified: bool,
    contamination_report: dict[str, Any],
) -> tuple[int, dict[str, Any]]:
    blockers = list(
        pre_pr_blockers(
            target_repo=target_repo,
            runtime_root=runtime_root,
            task_id=task_id,
            required_checks=required_checks,
            observed_checks=observed_checks,
            lifecycle_state=lifecycle_state,
            learning_review_completed=learning_review_completed,
            monitoring_chain_verified=monitoring_chain_verified,
            contamination_report=contamination_report,
        )
    )
    payload = {
        "status": "ok" if not blockers else "blocked",
        "blockers": blockers,
        "task_id": task_id,
    }
    return (0 if not blockers else 1), payload


def _learning_check_payload(
    *,
    task_store_path: Path,
    task_id: str,
) -> tuple[int, dict[str, Any]]:
    store = TaskStore(task_store_path)
    record = store.load(task_id)
    completed = bool(record.learning_review.get("completed", False))
    questions = record.learning_review.get("questions", [])
    challenge = record.learning_review.get("practical_challenge", {})
    blockers: list[str] = []
    if not completed:
        blockers.append("learning_review_incomplete")
    if not questions:
        blockers.append("missing_learning_questions")
    if not challenge.get("evidence_references"):
        blockers.append("missing_practical_verification_evidence")
    payload = {
        "status": "ok" if not blockers else "blocked",
        "task_id": task_id,
        "blockers": blockers,
        "learning_review_completed": completed,
    }
    return (0 if not blockers else 1), payload


def _parse_checks(value: str) -> dict[str, str]:
    if not value.strip():
        return {}
    parsed = json.loads(value)
    if not isinstance(parsed, dict):
        raise ValueError("checks JSON must be an object")
    return {str(key): str(item) for key, item in parsed.items()}


def _installed_harness_revision() -> str:
    version_path = _repo_root() / "version.json"
    value = json.loads(version_path.read_text(encoding="utf-8"))
    revision = value.get("harness_revision") if isinstance(value, dict) else None
    if not isinstance(revision, str) or not revision.strip():
        raise ValueError("installed runtime version does not contain a Harness revision")
    return revision.strip()


def _task_service_from_args(args: argparse.Namespace) -> TaskService:
    return prepare_runtime_environment(
        target_repo=args.target_repo,
        runtime_root=args.runtime_root,
        harness_revision=_installed_harness_revision(),
    ).task_service


def _add_task_location_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--target-repo", type=Path, required=True)
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--task-id", required=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Execution-plane runtime helpers")
    subparsers = parser.add_subparsers(dest="command", required=True)

    self_test = subparsers.add_parser("self-test")
    self_test.add_argument("--target-repo", type=Path, required=True)
    self_test.add_argument("--runtime-root", type=Path, required=True)
    self_test.add_argument("--harness-sha", required=True)

    governance = subparsers.add_parser("governance-check")
    governance.add_argument("--target-repo", type=Path, required=True)
    governance.add_argument("--runtime-root", type=Path, required=True)
    governance.add_argument("--task-id", required=True)
    governance.add_argument("--required-check", action="append", default=[])
    governance.add_argument("--observed-checks-json", default="{}")
    governance.add_argument("--lifecycle-state", default="")
    governance.add_argument("--learning-review-completed", action="store_true")
    governance.add_argument("--monitoring-chain-verified", action="store_true")
    governance.add_argument(
        "--contamination-report-json", default='{"violations":["not-validated"]}'
    )

    learning = subparsers.add_parser("learning-check")
    learning.add_argument("--runtime-root", type=Path, required=True)
    learning.add_argument("--task-id", required=True)

    learning_review = subparsers.add_parser("learning-review")
    learning_review.add_argument("--target-repo", type=Path, required=True)
    learning_review.add_argument("--runtime-root", type=Path, required=True)
    learning_review.add_argument("--task-id", required=True)
    learning_review.add_argument("--student-id", required=True)
    learning_review.add_argument("--base-ref", required=True)
    learning_review.add_argument("--head-ref", required=True)
    learning_review.add_argument("--prepare-only", action="store_true")
    learning_review.add_argument("--answers-file", type=Path)
    learning_review.add_argument("--verification-action", default="")
    learning_review.add_argument("--verification-result", default="")
    learning_review.add_argument("--evidence", type=Path)

    pre_pr = subparsers.add_parser("pre-pr-check")
    pre_pr.add_argument("--target-repo", type=Path, required=True)
    pre_pr.add_argument("--runtime-root", type=Path, required=True)
    pre_pr.add_argument("--task-id", required=True)
    pre_pr.add_argument("--required-check", action="append", default=[])
    pre_pr.add_argument("--observed-checks-json", default="{}")
    pre_pr.add_argument("--lifecycle-state", default="")
    pre_pr.add_argument("--learning-review-completed", action="store_true")
    pre_pr.add_argument("--monitoring-chain-verified", action="store_true")
    pre_pr.add_argument("--contamination-report-json", default='{"violations":["not-validated"]}')

    pr_create = subparsers.add_parser("pr-create")
    pr_create.add_argument("--target-repo", type=Path, required=True)
    pr_create.add_argument("--runtime-root", type=Path, required=True)
    pr_create.add_argument("--task-id", required=True)
    pr_create.add_argument("--repository", required=True)
    pr_create.add_argument("--title", required=True)
    pr_create.add_argument("--body", required=True)
    pr_create.add_argument("--base", required=True)
    pr_create.add_argument("--head", required=True)
    pr_create.add_argument("--requested-by-session-id", required=True)
    pr_create.add_argument("--required-check", action="append", default=[])
    pr_create.add_argument("--observed-checks-json", default="{}")
    pr_create.add_argument("--lifecycle-state", default="")
    pr_create.add_argument("--learning-review-completed", action="store_true")
    pr_create.add_argument("--monitoring-chain-verified", action="store_true")
    pr_create.add_argument(
        "--contamination-report-json", default='{"violations":["not-validated"]}'
    )

    review_prepare = subparsers.add_parser("review-prepare")
    review_prepare.add_argument("--target-repo", type=Path, required=True)
    review_prepare.add_argument("--runtime-root", type=Path, required=True)
    review_prepare.add_argument("--task-id", required=True)
    review_prepare.add_argument("--repository", required=True)
    review_prepare.add_argument("--pull-request", type=int, required=True)
    review_prepare.add_argument("--requested-by-session-id", required=True)
    review_prepare.add_argument("--base-sha", default="")
    review_prepare.add_argument("--head-sha", default="")
    review_prepare.add_argument("--base-ref", default="")
    review_prepare.add_argument("--head-ref", default="")
    review_prepare.add_argument("--title", default="")
    review_prepare.add_argument("--body", default="")

    review_record = subparsers.add_parser("review-record")
    review_record.add_argument("--target-repo", type=Path, required=True)
    review_record.add_argument("--runtime-root", type=Path, required=True)
    review_record.add_argument("--task-id", required=True)
    review_record.add_argument("--repository", required=True)
    review_record.add_argument("--pull-request", type=int, required=True)
    review_record.add_argument("--bundle", type=Path, required=True)
    review_record.add_argument("--requested-by-session-id", default="")
    review_record.add_argument("--reviewer-session-id", default="")

    review_started = subparsers.add_parser("review-agent-started")
    review_started.add_argument("--target-repo", type=Path, required=True)
    review_started.add_argument("--runtime-root", type=Path, required=True)
    review_started.add_argument("--task-id", required=True)
    review_started.add_argument("--repository", required=True)
    review_started.add_argument("--pull-request", type=int, required=True)
    review_started.add_argument("--head-sha", required=True)
    review_started.add_argument("--requested-by-session-id", required=True)
    review_started.add_argument("--reviewer-session-id", required=True)

    review_request = subparsers.add_parser("review-request-github")
    review_request.add_argument("--target-repo", type=Path, required=True)
    review_request.add_argument("--runtime-root", type=Path, required=True)
    review_request.add_argument("--task-id", required=True)
    review_request.add_argument("--repository", required=True)
    review_request.add_argument("--pull-request", type=int, required=True)
    review_request.add_argument("--expected-head", required=True)
    review_request.add_argument("--request-publisher-login", required=True)
    review_request.add_argument("--request-publisher-id", type=int, required=True)
    review_request.add_argument("--request-publisher-type", default="Bot")

    review_publish = subparsers.add_parser("review-publish")
    review_publish.add_argument("--target-repo", type=Path, required=True)
    review_publish.add_argument("--runtime-root", type=Path, required=True)
    review_publish.add_argument("--task-id", required=True)
    review_publish.add_argument("--repository", required=True)
    review_publish.add_argument("--pull-request", type=int, required=True)
    review_publish.add_argument("--bundle", type=Path, required=True)
    review_publish.add_argument("--trusted-reviewer-login", required=True)
    review_publish.add_argument("--trusted-reviewer-id", type=int, required=True)
    review_publish.add_argument("--trusted-reviewer-type", default="Bot")
    review_publish.add_argument("--trusted-request-publisher-login", required=True)
    review_publish.add_argument("--trusted-request-publisher-id", type=int, required=True)
    review_publish.add_argument("--trusted-request-publisher-type", default="Bot")
    review_publish.add_argument("--platform-wait-seconds", type=int, default=600)
    review_publish.add_argument("--platform-poll-seconds", type=int, default=5)

    review_validate = subparsers.add_parser("review-validate-github")
    review_validate.add_argument("--target-repo", type=Path, required=True)
    review_validate.add_argument("--runtime-root", type=Path, required=True)
    review_validate.add_argument("--task-id", required=True)
    review_validate.add_argument("--repository", required=True)
    review_validate.add_argument("--pull-request", type=int, required=True)
    review_validate.add_argument("--expected-head", default="")
    review_validate.add_argument("--trusted-reviewer-login", required=True)
    review_validate.add_argument("--trusted-reviewer-id", type=int, required=True)
    review_validate.add_argument("--trusted-reviewer-type", default="Bot")
    review_validate.add_argument("--trusted-request-publisher-login", required=True)
    review_validate.add_argument("--trusted-request-publisher-id", type=int, required=True)
    review_validate.add_argument("--trusted-request-publisher-type", default="Bot")

    review_respond = subparsers.add_parser("review-respond")
    review_respond.add_argument("--target-repo", type=Path, required=True)
    review_respond.add_argument("--runtime-root", type=Path, required=True)
    review_respond.add_argument("--task-id", required=True)
    review_respond.add_argument("--repository", required=True)
    review_respond.add_argument("--pull-request", type=int, required=True)
    review_respond.add_argument("--head-sha", required=True)
    review_respond.add_argument("--finding-id", required=True)
    review_respond.add_argument("--decision", required=True)
    review_respond.add_argument("--rationale", default="")
    review_respond.add_argument("--evidence", action="append", default=[])
    review_respond.add_argument(
        "--responder-role",
        default="Software Team Member",
    )

    review_stale = subparsers.add_parser("review-stale")
    review_stale.add_argument("--target-repo", type=Path, required=True)
    review_stale.add_argument("--runtime-root", type=Path, required=True)
    review_stale.add_argument("--task-id", required=True)
    review_stale.add_argument("--repository", required=True)
    review_stale.add_argument("--pull-request", type=int, required=True)
    review_stale.add_argument("--previous-head-sha", required=True)
    review_stale.add_argument("--new-head-sha", required=True)

    review_fix = subparsers.add_parser("review-fix-pushed")
    review_fix.add_argument("--target-repo", type=Path, required=True)
    review_fix.add_argument("--runtime-root", type=Path, required=True)
    review_fix.add_argument("--task-id", required=True)
    review_fix.add_argument("--repository", required=True)
    review_fix.add_argument("--pull-request", type=int, required=True)
    review_fix.add_argument("--previous-head-sha", required=True)
    review_fix.add_argument("--new-head-sha", required=True)
    review_fix.add_argument("--commit-sha", required=True)

    task_start = subparsers.add_parser("task-start")
    _add_task_location_arguments(task_start)
    task_start.add_argument("--student-id", required=True)
    task_start.add_argument("--request", required=True)
    task_start.add_argument("--planned-file", action="append", default=[])

    task_show = subparsers.add_parser("task-show")
    _add_task_location_arguments(task_show)

    task_answer = subparsers.add_parser("task-answer")
    _add_task_location_arguments(task_answer)
    task_answer.add_argument("--answer", required=True)
    task_answer.add_argument("--student-id")

    task_confirm = subparsers.add_parser("task-confirm")
    _add_task_location_arguments(task_confirm)
    task_confirm.add_argument("--student-id")

    task_check_edit = subparsers.add_parser("task-check-edit")
    _add_task_location_arguments(task_check_edit)
    task_check_edit.add_argument("--path", required=True)

    task_begin = subparsers.add_parser("task-begin")
    _add_task_location_arguments(task_begin)
    task_begin.add_argument("--file", action="append", default=[])

    args = parser.parse_args(argv)

    if args.command == "self-test":
        environment = prepare_runtime_environment(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            harness_revision=args.harness_sha,
        )
        payload = {
            "status": "ok",
            "harness_revision": environment.harness_revision,
            "runtime_root": str(environment.runtime_root),
            "manifest_sha256": environment.manifest["manifest_sha256"],
            "manifest_path": str(environment.manifest_path),
        }
        payload["import_smoke_test"] = _run_import_smoke_test(
            source_root=_repo_root(),
            runtime_root=environment.runtime_root,
        )
        print(_canonical_json(payload), end="")
        return 0

    if args.command == "governance-check":
        exit_code, payload = _governance_check_payload(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=args.task_id,
            required_checks=tuple(args.required_check),
            observed_checks=_parse_checks(args.observed_checks_json),
            lifecycle_state=args.lifecycle_state,
            learning_review_completed=bool(args.learning_review_completed),
            monitoring_chain_verified=bool(args.monitoring_chain_verified),
            contamination_report=json.loads(args.contamination_report_json),
        )
        print(_canonical_json(payload), end="")
        return exit_code

    if args.command == "learning-review":
        if bool(args.prepare_only):
            payload = _prepare_learning_review(
                target_repo=args.target_repo,
                runtime_root=args.runtime_root,
                task_id=args.task_id,
                student_id=args.student_id,
                base_ref=args.base_ref,
                head_ref=args.head_ref,
            )
            print(_canonical_json(payload), end="")
            return 0
        if args.answers_file is None:
            raise ValueError("--answers-file is required unless --prepare-only is set")
        if args.evidence is None:
            raise ValueError("--evidence is required unless --prepare-only is set")
        payload = _complete_learning_review(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=args.task_id,
            student_id=args.student_id,
            base_ref=args.base_ref,
            head_ref=args.head_ref,
            answers_file=args.answers_file,
            verification_action=str(args.verification_action),
            verification_result=str(args.verification_result),
            evidence=args.evidence,
        )
        print(_canonical_json(payload), end="")
        return 0

    if args.command == "pre-pr-check":
        exit_code, payload = _pre_pr_check_payload(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=args.task_id,
            required_checks=tuple(args.required_check),
            observed_checks=_parse_checks(args.observed_checks_json),
            lifecycle_state=str(args.lifecycle_state),
            learning_review_completed=bool(args.learning_review_completed),
            monitoring_chain_verified=bool(args.monitoring_chain_verified),
            contamination_report=json.loads(args.contamination_report_json),
        )
        print(_canonical_json(payload), end="")
        return exit_code

    if args.command == "pr-create":
        exit_code, payload = create_pull_request_with_handoff(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=str(args.task_id),
            repository=str(args.repository),
            title=str(args.title),
            body=str(args.body),
            base=str(args.base),
            head=str(args.head),
            requested_by_session_id=str(args.requested_by_session_id),
            required_checks=tuple(args.required_check),
            observed_checks=_parse_checks(args.observed_checks_json),
            lifecycle_state=str(args.lifecycle_state),
            learning_review_completed=bool(args.learning_review_completed),
            monitoring_chain_verified=bool(args.monitoring_chain_verified),
            contamination_report=json.loads(args.contamination_report_json),
        )
        print(_canonical_json(payload), end="")
        return exit_code

    if args.command == "review-prepare":
        payload = prepare_review_handoff(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            repository=str(args.repository),
            pull_request=int(args.pull_request),
            task_id=str(args.task_id),
            requested_by_session_id=str(args.requested_by_session_id),
            base_sha=str(args.base_sha),
            head_sha=str(args.head_sha),
            base_ref=str(args.base_ref),
            head_ref=str(args.head_ref),
            title=str(args.title),
            body=str(args.body),
        )
        print(_canonical_json(payload), end="")
        return 0

    if args.command == "review-record":
        payload = record_review_bundle(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=str(args.task_id),
            repository=str(args.repository),
            pull_request=int(args.pull_request),
            bundle=args.bundle,
            requested_by_session_id=str(args.requested_by_session_id),
            reviewer_session_id=str(args.reviewer_session_id),
        )
        print(_canonical_json(payload), end="")
        return 0

    if args.command == "review-agent-started":
        payload = record_review_agent_started(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=str(args.task_id),
            repository=str(args.repository),
            pull_request=int(args.pull_request),
            head_sha=str(args.head_sha),
            requested_by_session_id=str(args.requested_by_session_id),
            reviewer_session_id=str(args.reviewer_session_id),
        )
        print(_canonical_json(payload), end="")
        return 0

    if args.command == "review-request-github":
        payload = request_platform_review_on_github(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=str(args.task_id),
            repository=str(args.repository),
            pull_request=int(args.pull_request),
            expected_head=str(args.expected_head),
            request_publisher_login=str(args.request_publisher_login),
            request_publisher_id=int(args.request_publisher_id),
            request_publisher_type=str(args.request_publisher_type),
        )
        print(_canonical_json(payload), end="")
        return 0

    if args.command == "review-publish":
        exit_code, payload = publish_review_bundle(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=str(args.task_id),
            repository=str(args.repository),
            pull_request=int(args.pull_request),
            bundle=args.bundle,
            trusted_reviewer_login=str(args.trusted_reviewer_login),
            trusted_reviewer_id=int(args.trusted_reviewer_id),
            trusted_reviewer_type=str(args.trusted_reviewer_type),
            trusted_request_publisher_login=str(args.trusted_request_publisher_login),
            trusted_request_publisher_id=int(args.trusted_request_publisher_id),
            trusted_request_publisher_type=str(args.trusted_request_publisher_type),
            platform_wait_seconds=int(args.platform_wait_seconds),
            platform_poll_seconds=int(args.platform_poll_seconds),
        )
        print(_canonical_json(payload), end="")
        return exit_code

    if args.command == "review-validate-github":
        exit_code, payload = validate_review_state_on_github(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=str(args.task_id),
            repository=str(args.repository),
            pull_request=int(args.pull_request),
            expected_head=str(args.expected_head),
            trusted_reviewer_login=str(args.trusted_reviewer_login),
            trusted_reviewer_id=int(args.trusted_reviewer_id),
            trusted_reviewer_type=str(args.trusted_reviewer_type),
            trusted_request_publisher_login=str(args.trusted_request_publisher_login),
            trusted_request_publisher_id=int(args.trusted_request_publisher_id),
            trusted_request_publisher_type=str(args.trusted_request_publisher_type),
        )
        print(_canonical_json(payload), end="")
        return exit_code

    if args.command == "review-respond":
        payload = record_review_response(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=str(args.task_id),
            repository=str(args.repository),
            pull_request=int(args.pull_request),
            head_sha=str(args.head_sha),
            finding_id=str(args.finding_id),
            decision=str(args.decision),
            rationale=str(args.rationale),
            evidence=tuple(str(item) for item in args.evidence),
            responder_role=str(args.responder_role),
        )
        print(_canonical_json(payload), end="")
        return 0

    if args.command == "review-stale":
        payload = mark_review_stale(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=str(args.task_id),
            repository=str(args.repository),
            pull_request=int(args.pull_request),
            previous_head_sha=str(args.previous_head_sha),
            new_head_sha=str(args.new_head_sha),
        )
        print(_canonical_json(payload), end="")
        return 0

    if args.command == "review-fix-pushed":
        payload = record_review_fix_pushed(
            target_repo=args.target_repo,
            runtime_root=args.runtime_root,
            task_id=str(args.task_id),
            repository=str(args.repository),
            pull_request=int(args.pull_request),
            previous_head_sha=str(args.previous_head_sha),
            new_head_sha=str(args.new_head_sha),
            commit_sha=str(args.commit_sha),
        )
        print(_canonical_json(payload), end="")
        return 0

    if args.command == "task-start":
        record = _task_service_from_args(args).start(
            repository=args.target_repo,
            request=str(args.request),
            task_id=str(args.task_id),
            student_identifier=str(args.student_id),
            repository_role="target",
            planned_files=tuple(str(item) for item in args.planned_file),
        )
        print(_canonical_json(record.to_dict()), end="")
        return 0

    if args.command == "task-show":
        record = _task_service_from_args(args).show(str(args.task_id))
        print(_canonical_json(record.to_dict()), end="")
        return 0

    if args.command == "task-answer":
        record = _task_service_from_args(args).answer(
            str(args.task_id),
            str(args.answer),
            student_identifier=args.student_id,
        )
        print(_canonical_json(record.to_dict()), end="")
        return 0

    if args.command == "task-confirm":
        record = _task_service_from_args(args).confirm(
            str(args.task_id),
            student_identifier=args.student_id,
        )
        print(_canonical_json(record.to_dict()), end="")
        return 0

    if args.command == "task-check-edit":
        result = _task_service_from_args(args).check_edit(str(args.task_id), str(args.path))
        print(_canonical_json(result), end="")
        return 0 if result["allowed"] else 1

    if args.command == "task-begin":
        record = _task_service_from_args(args).begin_implementation(
            str(args.task_id),
            files=[str(item) for item in args.file] or None,
        )
        print(_canonical_json(record.to_dict()), end="")
        return 0

    exit_code, payload = _learning_check_payload(
        task_store_path=_resolve_runtime_root(args.runtime_root) / "tasks",
        task_id=args.task_id,
    )
    print(_canonical_json(payload), end="")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
