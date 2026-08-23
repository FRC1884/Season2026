"""Verify managed runtime bytes before importing target execution code."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
from pathlib import Path, PurePosixPath


def _canonical(value: object) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _fail(message: str) -> int:
    print(f"runtime integrity failure: {message}", file=sys.stderr)
    return 1


def _path_uses_symlink(root: Path, candidate: Path) -> bool:
    try:
        relative = candidate.relative_to(root)
    except ValueError:
        return True
    current = root
    for part in relative.parts:
        current /= part
        if current.is_symlink():
            return True
    return False


def _expected_entrypoint_bytes() -> bytes:
    return (
        "from __future__ import annotations\n\n"
        "from harness.execution_plane.runtime import main\n\n"
        'if __name__ == "__main__":\n'
        "    raise SystemExit(main())\n"
    ).encode("utf-8")


def _validate_bootstrap_version(
    path: Path,
    *,
    root: Path,
    expected_harness_ref: str,
    manifest_sha256: str,
    payload_sha256: str,
    runtime_version: object,
) -> tuple[int, dict[str, object] | None]:
    if _path_uses_symlink(root, path) or path.is_symlink() or not path.is_file():
        return _fail("runtime version record is missing or unsafe"), None
    try:
        raw = path.read_text(encoding="utf-8")
        version = json.loads(raw)
    except (OSError, json.JSONDecodeError) as error:
        return _fail(f"runtime version record is invalid: {error}"), None
    if not isinstance(version, dict):
        return _fail("runtime version record must be an object"), None
    expected = {
        "schema_version": 1,
        "runtime_version": runtime_version,
        "harness_revision": expected_harness_ref,
        "manifest_sha256": manifest_sha256,
        "payload_sha256": payload_sha256,
    }
    if version != expected:
        return _fail("runtime version record does not match trusted metadata"), None
    if raw.encode("utf-8") != _canonical(expected):
        return _fail("runtime version record serialization is invalid"), None
    return 0, version


def _validate_bootstrap_entrypoint(path: Path, *, root: Path) -> int:
    if _path_uses_symlink(root, path) or path.is_symlink() or not path.is_file():
        return _fail("runtime entrypoint is missing or unsafe")
    if path.read_bytes() != _expected_entrypoint_bytes():
        return _fail("runtime entrypoint byte drift")
    return 0


def _validate_tree_entries(
    *,
    root: Path,
    tree_root: Path,
    allowed_files: set[str],
) -> int:
    allowed_directories = {".github/robotics-harness"}
    for relative in allowed_files:
        current = PurePosixPath(relative).parent
        while current != PurePosixPath("."):
            allowed_directories.add(current.as_posix())
            current = current.parent
    if (
        _path_uses_symlink(root, tree_root)
        or tree_root.is_symlink()
        or not tree_root.is_dir()
    ):
        return _fail("managed runtime root is missing or unsafe")
    for current_root, dirnames, filenames in os.walk(
        tree_root, topdown=True, followlinks=False
    ):
        current_path = Path(current_root)
        if _path_uses_symlink(root, current_path):
            return _fail("managed runtime root contains a symbolic link")
        current_relative = current_path.relative_to(root).as_posix()
        if current_relative not in allowed_directories:
            return _fail(f"unexpected managed runtime directory: {current_relative}")
        for name in dirnames:
            candidate = current_path / name
            relative = candidate.relative_to(root).as_posix()
            entry = candidate.lstat()
            if stat.S_ISLNK(entry.st_mode):
                return _fail(f"unexpected managed runtime symlink: {relative}")
            if not stat.S_ISDIR(entry.st_mode):
                return _fail(f"unexpected managed runtime special entry: {relative}")
            if relative not in allowed_directories:
                return _fail(f"unexpected managed runtime directory: {relative}")
        for name in filenames:
            candidate = current_path / name
            relative = candidate.relative_to(root).as_posix()
            entry = candidate.lstat()
            if stat.S_ISLNK(entry.st_mode):
                return _fail(f"unexpected managed runtime symlink: {relative}")
            if not stat.S_ISREG(entry.st_mode):
                return _fail(f"unexpected managed runtime special entry: {relative}")
            if relative not in allowed_files:
                return _fail(f"unexpected managed runtime file: {relative}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--expected-manifest-sha")
    parser.add_argument("--expected-payload-sha")
    parser.add_argument("--expected-harness-ref")
    args = parser.parse_args()
    root = args.repo.resolve()
    if not args.expected_harness_ref:
        return _fail("expected Harness ref is required")
    if bool(args.expected_manifest_sha) == bool(args.expected_payload_sha):
        return _fail("supply exactly one expected manifest or payload digest")
    manifest_path = root / ".github/robotics-harness/execution-plane-manifest.json"
    if manifest_path.is_symlink() or not manifest_path.is_file():
        return _fail("execution-plane manifest is missing or unsafe")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return _fail(f"execution-plane manifest is invalid: {error}")
    if not isinstance(manifest, dict):
        return _fail("execution-plane manifest must be an object")
    declared = str(manifest.pop("manifest_sha256", ""))
    calculated = hashlib.sha256(_canonical(manifest)).hexdigest()
    if declared != calculated:
        return _fail("execution-plane manifest digest does not match the trusted pin")
    if args.expected_manifest_sha and declared != args.expected_manifest_sha:
        return _fail("execution-plane manifest digest does not match the trusted pin")
    if str(manifest.get("harness_revision", "")) != args.expected_harness_ref:
        return _fail("execution-plane Harness revision does not match the trusted pin")
    files = manifest.get("files")
    if not isinstance(files, list) or not files:
        return _fail("execution-plane manifest has no files")
    allowed_files = {
        ".github/robotics-harness/execution-plane-manifest.json",
        ".github/robotics-harness/runtime/version.json",
        ".github/robotics-harness/runtime/entrypoint.py",
    }
    payload_files = [
        entry
        for entry in files
        if isinstance(entry, dict)
        and str(entry.get("path", ""))
        != ".github/robotics-harness/robotics-harness"
    ]
    payload = {
        "schema_version": manifest.get("schema_version"),
        "runtime_version": manifest.get("runtime_version"),
        "harness_revision": manifest.get("harness_revision"),
        "files": payload_files,
    }
    calculated_payload = hashlib.sha256(_canonical(payload)).hexdigest()
    declared_payload = str(manifest.get("payload_sha256", ""))
    if declared_payload != calculated_payload:
        return _fail("execution-plane payload digest is invalid")
    if args.expected_payload_sha and declared_payload != args.expected_payload_sha:
        return _fail("execution-plane payload digest does not match the trusted pin")
    declared_paths = {
        str(entry.get("path", "")) for entry in files if isinstance(entry, dict)
    }
    if len(declared_paths) != len(files):
        return _fail("execution-plane manifest contains duplicated or invalid paths")
    required_launchers = {
        ".github/robotics-harness/robotics-harness",
        ".github/robotics-harness/verify-runtime.py",
    }
    if not required_launchers.issubset(declared_paths):
        return _fail("execution-plane manifest omits a required launcher")
    for entry in files:
        if not isinstance(entry, dict):
            return _fail("execution-plane manifest file entry is invalid")
        relative = str(entry.get("path", ""))
        parsed = PurePosixPath(relative)
        if parsed.is_absolute() or ".." in parsed.parts:
            return _fail("execution-plane manifest contains an unsafe path")
        candidate = root / relative
        if (
            _path_uses_symlink(root, candidate)
            or candidate.is_symlink()
            or not candidate.is_file()
        ):
            return _fail(f"managed runtime file is missing or unsafe: {relative}")
        resolved = candidate.resolve()
        if not resolved.is_relative_to(root):
            return _fail(f"managed runtime file escapes the repository: {relative}")
        digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
        expected_content_digest = str(
            entry.get("content_sha256") or entry.get("source_sha256", "")
        )
        if digest != expected_content_digest:
            return _fail(f"managed runtime byte drift: {relative}")
        allowed_files.add(relative)
    tree_root = root / ".github/robotics-harness"
    tree_status = _validate_tree_entries(
        root=root,
        tree_root=tree_root,
        allowed_files=allowed_files,
    )
    if tree_status:
        return tree_status
    version_status, version = _validate_bootstrap_version(
        root / ".github/robotics-harness/runtime/version.json",
        root=root,
        expected_harness_ref=args.expected_harness_ref,
        manifest_sha256=declared,
        payload_sha256=declared_payload,
        runtime_version=manifest.get("runtime_version"),
    )
    if version_status:
        return version_status
    del version
    entrypoint_status = _validate_bootstrap_entrypoint(
        root / ".github/robotics-harness/runtime/entrypoint.py",
        root=root,
    )
    if entrypoint_status:
        return entrypoint_status
    print(json.dumps({"status": "ok", "manifest_sha256": declared}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
