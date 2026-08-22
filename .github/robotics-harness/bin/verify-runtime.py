"""Verify managed runtime bytes before importing target execution code."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path, PurePosixPath


def _canonical(value: object) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _fail(message: str) -> int:
    print(f"runtime integrity failure: {message}", file=sys.stderr)
    return 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--expected-manifest-sha", required=True)
    parser.add_argument("--expected-harness-ref", required=True)
    args = parser.parse_args()
    root = args.repo.resolve()
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
    if declared != calculated or declared != args.expected_manifest_sha:
        return _fail("execution-plane manifest digest does not match the trusted pin")
    if str(manifest.get("harness_revision", "")) != args.expected_harness_ref:
        return _fail("execution-plane Harness revision does not match the trusted pin")
    files = manifest.get("files")
    if not isinstance(files, list) or not files:
        return _fail("execution-plane manifest has no files")
    for entry in files:
        if not isinstance(entry, dict):
            return _fail("execution-plane manifest file entry is invalid")
        relative = str(entry.get("path", ""))
        parsed = PurePosixPath(relative)
        if parsed.is_absolute() or ".." in parsed.parts:
            return _fail("execution-plane manifest contains an unsafe path")
        candidate = root / relative
        if candidate.is_symlink() or not candidate.is_file():
            return _fail(f"managed runtime file is missing or unsafe: {relative}")
        resolved = candidate.resolve()
        if not resolved.is_relative_to(root):
            return _fail(f"managed runtime file escapes the repository: {relative}")
        digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
        if digest != str(entry.get("source_sha256", "")):
            return _fail(f"managed runtime byte drift: {relative}")
    version_path = root / ".github/robotics-harness/runtime/version.json"
    if version_path.is_symlink() or not version_path.is_file():
        return _fail("runtime version record is missing or unsafe")
    version = json.loads(version_path.read_text(encoding="utf-8"))
    if str(version.get("harness_revision", "")) != args.expected_harness_ref:
        return _fail("runtime version Harness revision does not match")
    if str(version.get("manifest_sha256", "")) != declared:
        return _fail("runtime version manifest digest does not match")
    print(json.dumps({"status": "ok", "manifest_sha256": declared}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
