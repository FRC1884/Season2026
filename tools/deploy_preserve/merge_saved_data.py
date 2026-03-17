#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


CONFIG_FILES = [
    "joystick-mappings.json",
    "subsystem-descriptions.json",
]


def read_json(path: Path) -> dict | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None


def metadata_timestamp(payload: dict | None) -> int:
    if not payload:
        return -1
    metadata = payload.get("metadata") or {}
    try:
        return int(metadata.get("updatedAtEpochMs", -1))
    except (TypeError, ValueError):
        return -1


def schema_version(payload: dict | None) -> str:
    if not payload:
        return ""
    value = payload.get("schemaVersion")
    return value if isinstance(value, str) else ""


def copy_tree(src: Path, dst: Path) -> None:
    if not src.exists():
        return
    dst.mkdir(parents=True, exist_ok=True)
    for child in src.iterdir():
        target = dst / child.name
        if child.is_dir():
            copy_tree(child, target)
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(child, target)


def ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def copy_with_conflict(src: Path, dst: Path, conflicts_root: Path, label: str) -> None:
    if not src.exists():
        return
    ensure_parent(dst)
    if dst.exists():
        conflict_target = conflicts_root / label / src.name
        ensure_parent(conflict_target)
        shutil.copy2(dst, conflict_target)
    shutil.copy2(src, dst)


def merge_config(local_root: Path, robot_root: Path, conflicts_root: Path) -> None:
    local_root.mkdir(parents=True, exist_ok=True)
    for name in CONFIG_FILES:
        local_path = local_root / name
        robot_path = robot_root / name
        if not local_path.exists() and robot_path.exists():
            shutil.copy2(robot_path, local_path)
            continue
        if local_path.exists() and not robot_path.exists():
            continue
        if not local_path.exists() and not robot_path.exists():
            continue

        local_json = read_json(local_path)
        robot_json = read_json(robot_path)
        local_schema = schema_version(local_json)
        robot_schema = schema_version(robot_json)

        if local_schema and robot_schema and local_schema != robot_schema:
            copy_with_conflict(local_path, conflicts_root / "config-schema-mismatch" / f"local-{name}", conflicts_root, "preserved")
            copy_with_conflict(robot_path, conflicts_root / "config-schema-mismatch" / f"robot-{name}", conflicts_root, "preserved")
            continue

        local_time = metadata_timestamp(local_json)
        robot_time = metadata_timestamp(robot_json)
        if robot_time > local_time:
            copy_with_conflict(robot_path, local_path, conflicts_root, "config-replaced")


def merge_autos(local_root: Path, robot_root: Path, conflicts_root: Path) -> None:
    local_root.mkdir(parents=True, exist_ok=True)
    if not robot_root.exists():
        return
    for robot_file in robot_root.rglob("*"):
        if robot_file.is_dir():
            continue
        relative = robot_file.relative_to(robot_root)
        local_file = local_root / relative
        ensure_parent(local_file)
        if not local_file.exists():
            shutil.copy2(robot_file, local_file)
            continue
        if local_file.read_bytes() == robot_file.read_bytes():
            continue
        if robot_file.stat().st_mtime > local_file.stat().st_mtime:
            conflict_copy = conflicts_root / "autos-replaced" / relative
            ensure_parent(conflict_copy)
            shutil.copy2(local_file, conflict_copy)
            shutil.copy2(robot_file, local_file)
        else:
            conflict_copy = conflicts_root / "autos-robot-older" / relative
            ensure_parent(conflict_copy)
            shutil.copy2(robot_file, conflict_copy)


def merge_diagnostics(local_root: Path, robot_root: Path, conflicts_root: Path) -> None:
    local_bundles = local_root / "diagnostics" / "bundles"
    robot_bundles = robot_root / "diagnostics" / "bundles"
    if not robot_bundles.exists():
        return
    for bundle in robot_bundles.rglob("*"):
        relative = bundle.relative_to(robot_bundles)
        target = local_bundles / relative
        if bundle.is_dir():
            target.mkdir(parents=True, exist_ok=True)
            continue
        ensure_parent(target)
        if not target.exists():
            shutil.copy2(bundle, target)
            continue
        if target.read_bytes() != bundle.read_bytes():
            conflict_copy = conflicts_root / "diagnostic-conflicts" / relative
            ensure_parent(conflict_copy)
            shutil.copy2(bundle, conflict_copy)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--local-data-root", required=True)
    parser.add_argument("--local-autos-root", required=True)
    parser.add_argument("--cache-root", required=True)
    args = parser.parse_args()

    local_data_root = Path(args.local_data_root).resolve()
    local_autos_root = Path(args.local_autos_root).resolve()
    cache_root = Path(args.cache_root).resolve()
    robot_data_root = cache_root / "robot" / "operatorboard-data"
    robot_autos_root = cache_root / "robot" / "pathplana-autos"
    conflicts_root = cache_root / "conflicts"
    conflicts_root.mkdir(parents=True, exist_ok=True)

    merge_config(local_data_root, robot_data_root, conflicts_root)
    merge_autos(local_autos_root, robot_autos_root, conflicts_root)
    merge_diagnostics(local_data_root, robot_data_root, conflicts_root)


if __name__ == "__main__":
    main()
