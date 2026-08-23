"""Execution-plane packaging and external runtime orchestration."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "RuntimeEnvironment",
    "build_execution_plane_manifest",
    "pre_pr_blockers",
    "prepare_runtime_environment",
    "run_recorded_reviewer_job",
    "validate_product_diff",
]


def __getattr__(name: str) -> Any:
    if name == "build_execution_plane_manifest":
        return import_module("harness.execution_plane.manifest").build_execution_plane_manifest
    if name in {
        "RuntimeEnvironment",
        "prepare_runtime_environment",
        "pre_pr_blockers",
        "run_recorded_reviewer_job",
        "validate_product_diff",
    }:
        module = import_module("harness.execution_plane.runtime")
        return getattr(module, name)
    raise AttributeError(name)
