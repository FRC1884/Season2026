"""Deterministic execution-plane manifest generation."""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path, PurePosixPath
from typing import Any

_HEX_REVISION = re.compile(r"^[0-9a-f]{40,64}$")
_SECRET_PARTS = {
    "credential",
    "credentials",
    "key",
    "keys",
    "private",
    "secret",
    "secrets",
    "token",
    "tokens",
}
_FORBIDDEN_PREFIXES = (
    ".git/",
    ".github/",
    ".omx/",
    "artifacts/",
    "docs/",
    "templates/",
    "tests/",
    "harness/fleet/",
    "harness/github_governance/",
    "harness/synchronization/",
)
_FORBIDDEN_EXACT = {
    "AGENTS.md",
    ".github/CODEOWNERS",
}
_CONTROL_PLANE_ALLOWED_PREFIXES = (".github/codex/",)
_CONTROL_PLANE_EXCEPTIONS = {
    "config/risk_rules.yaml",
}


def _canonical_json(value: dict[str, Any]) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _normalize_relative_path(path: str) -> str:
    if not isinstance(path, str) or not path.strip():
        raise ValueError("allowlist entries must be non-empty relative paths")
    candidate = PurePosixPath(path.strip())
    if candidate.is_absolute() or ".." in candidate.parts:
        raise ValueError("allowlist entries must be relative and cannot traverse directories")
    normalized = candidate.as_posix().removeprefix("./")
    if not normalized or normalized == ".":
        raise ValueError("allowlist entries must not be empty")
    return normalized


def _is_secret_like(path: str) -> bool:
    parsed = PurePosixPath(path)
    for part in parsed.parts:
        lowered = part.casefold()
        stem = PurePosixPath(part).stem.casefold()
        if lowered in _SECRET_PARTS or stem in _SECRET_PARTS:
            return True
    return False


def _is_forbidden_control_plane(path: str) -> bool:
    if path in _CONTROL_PLANE_EXCEPTIONS:
        return False
    if any(path.startswith(prefix) for prefix in _CONTROL_PLANE_ALLOWED_PREFIXES):
        return False
    if path in _FORBIDDEN_EXACT:
        return True
    return any(path.startswith(prefix) for prefix in _FORBIDDEN_PREFIXES)


def build_execution_plane_manifest(
    *,
    source_root: Path | str,
    harness_revision: str,
    allowlist: tuple[str, ...] | list[str],
) -> dict[str, Any]:
    """Build a deterministic manifest for the execution-plane runtime bytes."""

    root = Path(source_root).expanduser().resolve()
    if not root.is_dir():
        raise ValueError("source_root must be an existing directory")
    revision = harness_revision.strip().lower()
    if not _HEX_REVISION.fullmatch(revision):
        raise ValueError("harness revision must be a 40-64 character hexadecimal object ID")

    normalized = sorted({_normalize_relative_path(path) for path in allowlist})
    files: list[dict[str, Any]] = []
    for relative in normalized:
        if _is_forbidden_control_plane(relative) or _is_secret_like(relative):
            raise ValueError(f"unsafe execution-plane allowlist path: {relative}")
        path = root / relative
        if path.is_symlink():
            raise ValueError(f"execution-plane file cannot be a symbolic link: {relative}")
        if not path.is_file():
            raise ValueError(f"execution-plane allowlist path is missing: {relative}")
        resolved = path.resolve()
        if not resolved.is_relative_to(root):
            raise ValueError(f"execution-plane allowlist path escapes source root: {relative}")
        content = path.read_bytes()
        files.append(
            {
                "path": relative,
                "sha256": hashlib.sha256(content).hexdigest(),
                "size_bytes": len(content),
            }
        )

    payload: dict[str, Any] = {
        "schema_version": 1,
        "harness_revision": revision,
        "files": files,
    }
    payload["manifest_sha256"] = hashlib.sha256(_canonical_json(payload)).hexdigest()
    return payload
