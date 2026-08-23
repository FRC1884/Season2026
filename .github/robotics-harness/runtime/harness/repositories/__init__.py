"""External repository registration and safe local integration controls."""

from harness.repositories.candidates import CandidateEvaluation, evaluate_test_candidates
from harness.repositories.integration import (
    ExternalDemoManager,
    ExternalPreflight,
    ScopeCheck,
    write_review_package,
)
from harness.repositories.registry import (
    DEFAULT_WPILIB_PROTECTED_PATHS,
    ExternalRepositoryInspector,
    RegisteredRepository,
    RepositoryInspection,
    RepositoryRegistry,
    RepositorySafetyError,
    RepositoryScopeBudgets,
    ScopeBudget,
    WorkspacePreparation,
    WorktreeResult,
    evaluate_protected_diff,
    policy_validator_for_registration,
)

__all__ = [
    "DEFAULT_WPILIB_PROTECTED_PATHS",
    "CandidateEvaluation",
    "ExternalDemoManager",
    "ExternalPreflight",
    "ExternalRepositoryInspector",
    "RegisteredRepository",
    "RepositoryInspection",
    "RepositoryRegistry",
    "RepositorySafetyError",
    "RepositoryScopeBudgets",
    "ScopeBudget",
    "ScopeCheck",
    "WorkspacePreparation",
    "WorktreeResult",
    "evaluate_protected_diff",
    "evaluate_test_candidates",
    "policy_validator_for_registration",
    "write_review_package",
]
