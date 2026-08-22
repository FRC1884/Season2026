"""Repository-grounded, read-only candidate scoring for the real-code demo."""

from __future__ import annotations

import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from harness.git import run_git
from harness.policy.validator import _matches
from harness.repositories.registry import RegisteredRepository

_METHOD = re.compile(
    r"(?m)^\s*(?:public|protected)\s+(?:static\s+)?"
    r"[A-Za-z0-9_<>,.?\[\] ]+\s+([a-zA-Z_][A-Za-z0-9_]*)\s*\("
)
_UNSAFE_TERMS = {
    "actuator",
    "autonomous",
    "currentlimit",
    "deploy",
    "drivetrain",
    "emergency",
    "interlock",
    "motor",
    "pid",
    "swerve",
}


@dataclass(frozen=True, slots=True)
class CandidateEvaluation:
    """A proposed test-only task; selection still requires student confirmation."""

    identifier: str
    source_file: str
    proposed_test_file: str
    relevant_symbols: tuple[str, ...]
    educational_value: int
    safety: int
    testability: int
    diff_size: int
    team_relevance: int
    workflow_coverage: int
    preliminary_risk: str
    rationale: str

    @property
    def total(self) -> int:
        return (
            self.educational_value
            + self.safety
            + self.testability
            + self.diff_size
            + self.team_relevance
            + self.workflow_coverage
        )

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["relevant_symbols"] = list(self.relevant_symbols)
        value["total"] = self.total
        return value


def _proposed_test_path(source_file: str) -> str:
    return source_file.replace("src/main/java/", "src/test/java/", 1).replace(".java", "Test.java")


def _is_safe_candidate(path: str, content: str) -> bool:
    lowered = f"{path}\n{content}".lower().replace("_", "")
    return not any(term in lowered for term in _UNSAFE_TERMS)


def evaluate_test_candidates(
    registration: RegisteredRepository,
    *,
    maximum_candidates: int = 3,
) -> tuple[CandidateEvaluation, ...]:
    """Rank meaningful test-only candidates without changing the repository."""

    if maximum_candidates < 1:
        raise ValueError("maximum_candidates must be positive")
    repository = Path(registration.path)
    tracked = tuple(run_git(repository, "ls-files").splitlines())
    tracked_set = set(tracked)
    evaluations: list[CandidateEvaluation] = []
    for path in tracked:
        if not path.startswith("src/main/java/") or not path.endswith(".java"):
            continue
        if "/util/" not in path and "/runtime/" not in path:
            continue
        if _matches(path, registration.protected_paths):
            continue
        try:
            content = (repository / path).read_text(encoding="utf-8")
        except OSError:
            continue
        if not _is_safe_candidate(path, content):
            continue
        symbols = tuple(dict.fromkeys(_METHOD.findall(content)))
        if not symbols:
            continue
        proposed_test = _proposed_test_path(path)
        line_count = len(content.splitlines())
        has_existing_test = proposed_test in tracked_set
        educational = 2 if len(symbols) >= 2 else 1
        safety = 2
        testability = 2 if " static " in f" {content} " or "record " in content else 1
        diff_size = 2 if line_count <= 200 else 1
        relevance = 2 if "/runtime/" in path else 1
        workflow = 2 if not has_existing_test else 1
        stem = Path(path).stem
        rationale = (
            f"Add focused regression coverage for {stem} using the existing Java test stack; "
            "keep production robot behaviour and protected paths unchanged."
        )
        evaluations.append(
            CandidateEvaluation(
                identifier=f"test-{stem.lower()}",
                source_file=path,
                proposed_test_file=proposed_test,
                relevant_symbols=symbols[:8],
                educational_value=educational,
                safety=safety,
                testability=testability,
                diff_size=diff_size,
                team_relevance=relevance,
                workflow_coverage=workflow,
                preliminary_risk="low",
                rationale=rationale,
            )
        )
    evaluations.sort(
        key=lambda candidate: (
            -candidate.total,
            candidate.source_file,
        )
    )
    return tuple(evaluations[:maximum_candidates])
