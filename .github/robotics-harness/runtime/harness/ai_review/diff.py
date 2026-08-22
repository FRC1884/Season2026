"""Complete, rename-aware Git diff collection and bounded review chunking."""

from __future__ import annotations

import hashlib
import re
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path, PurePosixPath
from typing import Any

_OBJECT_ID = re.compile(r"^[0-9a-fA-F]{40,64}$")
_GENERATED_PARTS = {"generated", "gen", "build"}
_GENERATED_SUFFIXES = (".lock", ".min.js", ".min.css", ".pb.java")


class DiffCollectionError(RuntimeError):
    """Raised when the requested full base-to-head comparison is unavailable."""


@dataclass(frozen=True, slots=True)
class ChangedFile:
    status: str
    path: str
    old_path: str | None
    additions: int | None
    deletions: int | None
    binary: bool
    generated: bool
    submodule: bool
    old_mode: str | None
    new_mode: str | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class DiffChunk:
    chunk_id: str
    path: str
    index: int
    total_for_file: int
    content: str
    content_sha256: str
    binary_metadata_only: bool
    generated: bool
    submodule: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class DiffBundle:
    repository: str
    base_sha: str
    head_sha: str
    files: tuple[ChangedFile, ...]
    chunks: tuple[DiffChunk, ...]
    full_diff_sha256: str
    full_diff_bytes: int
    additions: int
    deletions: int
    commit_messages: tuple[str, ...]
    complete: bool
    incomplete_reason: str = ""

    @property
    def changed_paths(self) -> tuple[str, ...]:
        return tuple(file.path for file in self.files)

    @property
    def analysed_paths(self) -> tuple[str, ...]:
        return tuple(dict.fromkeys(chunk.path for chunk in self.chunks))

    def to_dict(self) -> dict[str, Any]:
        return {
            "repository": self.repository,
            "base_sha": self.base_sha,
            "head_sha": self.head_sha,
            "files": [file.to_dict() for file in self.files],
            "chunks": [chunk.to_dict() for chunk in self.chunks],
            "full_diff_sha256": self.full_diff_sha256,
            "full_diff_bytes": self.full_diff_bytes,
            "additions": self.additions,
            "deletions": self.deletions,
            "commit_messages": list(self.commit_messages),
            "complete": self.complete,
            "incomplete_reason": self.incomplete_reason,
        }


def _run_git(repository: Path, *args: str, text: bool = False) -> bytes | str:
    completed = subprocess.run(
        ("git", "-c", "core.quotepath=false", *args),
        cwd=repository,
        check=False,
        capture_output=True,
    )
    if completed.returncode:
        stderr = completed.stderr.decode("utf-8", errors="replace").strip()
        raise DiffCollectionError(f"git {' '.join(args)} failed: {stderr}")
    if text:
        return completed.stdout.decode("utf-8", errors="replace")
    return completed.stdout


def _verify_commit(repository: Path, object_id: str, name: str) -> str:
    if not _OBJECT_ID.fullmatch(object_id):
        raise DiffCollectionError(f"{name} must be a full hexadecimal object ID")
    resolved = str(
        _run_git(repository, "rev-parse", "--verify", f"{object_id}^{{commit}}", text=True)
    )
    resolved = resolved.strip()
    if not resolved.startswith(object_id.lower()) and not object_id.lower().startswith(resolved):
        raise DiffCollectionError(f"{name} did not resolve to the requested commit")
    return resolved


def _verify_tree(repository: Path, object_id: str, name: str) -> str:
    if not _OBJECT_ID.fullmatch(object_id):
        raise DiffCollectionError(f"{name} must be a full hexadecimal object ID")
    resolved = str(
        _run_git(repository, "rev-parse", "--verify", f"{object_id}^{{tree}}", text=True)
    ).strip()
    if not resolved.startswith(object_id.lower()) and not object_id.lower().startswith(resolved):
        raise DiffCollectionError(f"{name} did not resolve to the requested tree")
    return resolved


def _comparison(base: str, head: str, *, direct: bool) -> tuple[str, ...]:
    return (base, head) if direct else (f"{base}...{head}",)


def _parse_name_status(raw: bytes) -> list[tuple[str, str | None, str]]:
    fields = raw.decode("utf-8", errors="replace").split("\0")
    if fields and fields[-1] == "":
        fields.pop()
    changed: list[tuple[str, str | None, str]] = []
    cursor = 0
    while cursor < len(fields):
        status = fields[cursor]
        cursor += 1
        if not status:
            raise DiffCollectionError("empty status in Git name-status output")
        if status[0] in {"R", "C"}:
            if cursor + 1 >= len(fields):
                raise DiffCollectionError("truncated Git rename/copy record")
            old_path, path = fields[cursor], fields[cursor + 1]
            cursor += 2
        else:
            if cursor >= len(fields):
                raise DiffCollectionError("truncated Git name-status record")
            old_path, path = None, fields[cursor]
            cursor += 1
        changed.append((status, old_path, path))
    return changed


def _tree_mode(repository: Path, commit: str, path: str) -> str | None:
    output = str(_run_git(repository, "ls-tree", "-z", commit, "--", path, text=True))
    if not output:
        return None
    header = output.split("\t", 1)[0]
    parts = header.split()
    return parts[0] if parts else None


def _line_counts(
    repository: Path,
    base: str,
    head: str,
    old_path: str | None,
    path: str,
    *,
    direct: bool,
) -> tuple[int | None, int | None, bool]:
    paths = (path,) if old_path is None else (old_path, path)
    output = str(
        _run_git(
            repository,
            "diff",
            "--numstat",
            *_comparison(base, head, direct=direct),
            "--",
            *paths,
            text=True,
        )
    )
    additions = 0
    deletions = 0
    found = False
    binary = False
    for line in output.splitlines():
        parts = line.split("\t", 2)
        if len(parts) < 3:
            continue
        found = True
        if parts[0] == "-" or parts[1] == "-":
            binary = True
            continue
        additions += int(parts[0])
        deletions += int(parts[1])
    if binary:
        return None, None, True
    if not found:
        return 0, 0, False
    return additions, deletions, False


def _looks_generated(path: str) -> bool:
    parsed = PurePosixPath(path)
    lowered_parts = {part.lower() for part in parsed.parts}
    lowered = path.lower()
    return bool(lowered_parts & _GENERATED_PARTS) or lowered.endswith(_GENERATED_SUFFIXES)


def _chunk_text(text: str, maximum_characters: int) -> list[str]:
    if maximum_characters < 512:
        raise ValueError("maximum chunk size must be at least 512 characters")
    if len(text) <= maximum_characters:
        return [text]
    chunks: list[str] = []
    current: list[str] = []
    current_size = 0
    for line in text.splitlines(keepends=True):
        if current and current_size + len(line) > maximum_characters:
            chunks.append("".join(current))
            current = []
            current_size = 0
        while len(line) > maximum_characters:
            if current:
                chunks.append("".join(current))
                current = []
                current_size = 0
            chunks.append(line[:maximum_characters])
            line = line[maximum_characters:]
        current.append(line)
        current_size += len(line)
    if current:
        chunks.append("".join(current))
    return chunks


def _file_patch(
    repository: Path,
    base: str,
    head: str,
    *,
    old_path: str | None,
    path: str,
    direct: bool,
) -> str:
    paths = (path,) if old_path is None else (old_path, path)
    raw = _run_git(
        repository,
        "diff",
        "--find-renames",
        "--find-copies",
        "--no-ext-diff",
        "--full-index",
        "--binary",
        *_comparison(base, head, direct=direct),
        "--",
        *paths,
    )
    assert isinstance(raw, bytes)
    return raw.decode("utf-8", errors="replace")


def collect_full_diff(
    repository: Path | str,
    *,
    base_sha: str,
    head_sha: str,
    maximum_chunk_characters: int = 32_000,
    direct_tree_comparison: bool = False,
) -> DiffBundle:
    """Collect every base-to-head file and patch without API pagination or truncation."""

    root = Path(repository).expanduser().resolve()
    if (
        not (root / ".git").exists()
        and not str(_run_git(root, "rev-parse", "--is-inside-work-tree", text=True)).strip()
        == "true"
    ):
        raise DiffCollectionError(f"{root} is not a Git worktree")
    base = (
        _verify_tree(root, base_sha, "base_sha")
        if direct_tree_comparison
        else _verify_commit(root, base_sha, "base_sha")
    )
    head = _verify_commit(root, head_sha, "head_sha")
    # A normal pull request may have branched before the current base SHA. Git's
    # three-dot comparison intentionally uses the merge base in that case.
    if not direct_tree_comparison:
        _run_git(root, "merge-base", base, head, text=True)

    status_raw = _run_git(
        root,
        "diff",
        "--name-status",
        "-z",
        "--find-renames",
        "--find-copies",
        *_comparison(base, head, direct=direct_tree_comparison),
    )
    assert isinstance(status_raw, bytes)
    statuses = _parse_name_status(status_raw)
    full_raw = _run_git(
        root,
        "diff",
        "--find-renames",
        "--find-copies",
        "--no-ext-diff",
        "--full-index",
        "--binary",
        *_comparison(base, head, direct=direct_tree_comparison),
        "--",
    )
    assert isinstance(full_raw, bytes)

    files: list[ChangedFile] = []
    chunks: list[DiffChunk] = []
    total_additions = 0
    total_deletions = 0
    for status, old_path, path in statuses:
        additions, deletions, binary = _line_counts(
            root,
            base,
            head,
            old_path,
            path,
            direct=direct_tree_comparison,
        )
        if additions is not None:
            total_additions += additions
        if deletions is not None:
            total_deletions += deletions
        old_mode = _tree_mode(root, base, old_path or path)
        new_mode = _tree_mode(root, head, path)
        submodule = old_mode == "160000" or new_mode == "160000"
        generated = _looks_generated(path)
        changed_file = ChangedFile(
            status=status,
            path=path,
            old_path=old_path,
            additions=additions,
            deletions=deletions,
            binary=binary,
            generated=generated,
            submodule=submodule,
            old_mode=old_mode,
            new_mode=new_mode,
        )
        files.append(changed_file)
        if binary:
            content_parts = [
                (
                    f"Binary file metadata: status={status}; old_path={old_path or '-'}; "
                    f"path={path}; old_mode={old_mode or '-'}; new_mode={new_mode or '-'}"
                )
            ]
        elif submodule:
            patch = _file_patch(
                root,
                base,
                head,
                old_path=old_path,
                path=path,
                direct=direct_tree_comparison,
            )
            content_parts = [
                (
                    f"Submodule change metadata: status={status}; path={path}; "
                    f"old_mode={old_mode or '-'}; new_mode={new_mode or '-'}\n"
                    f"{patch}"
                )
            ]
        else:
            patch = _file_patch(
                root,
                base,
                head,
                old_path=old_path,
                path=path,
                direct=direct_tree_comparison,
            )
            content_parts = _chunk_text(patch, maximum_chunk_characters)
        for index, content in enumerate(content_parts, start=1):
            content_digest = hashlib.sha256(content.encode("utf-8", errors="replace")).hexdigest()
            chunk_id = hashlib.sha256(f"{path}\0{index}\0{content_digest}".encode()).hexdigest()[
                :24
            ]
            chunks.append(
                DiffChunk(
                    chunk_id=chunk_id,
                    path=path,
                    index=index,
                    total_for_file=len(content_parts),
                    content=content,
                    content_sha256=content_digest,
                    binary_metadata_only=binary,
                    generated=generated,
                    submodule=submodule,
                )
            )

    expected_paths = {path for _, _, path in statuses}
    chunked_paths = {chunk.path for chunk in chunks}
    complete = expected_paths == chunked_paths
    reason = "" if complete else "one or more changed files were not represented by a review chunk"
    commits_output = (
        ""
        if direct_tree_comparison
        else str(_run_git(root, "log", "--format=%H%x09%s", f"{base}..{head}", text=True))
    )
    commits = tuple(line for line in commits_output.splitlines() if line)
    return DiffBundle(
        repository=str(root),
        base_sha=base,
        head_sha=head,
        files=tuple(files),
        chunks=tuple(chunks),
        full_diff_sha256=hashlib.sha256(full_raw).hexdigest(),
        full_diff_bytes=len(full_raw),
        additions=total_additions,
        deletions=total_deletions,
        commit_messages=commits,
        complete=complete,
        incomplete_reason=reason,
    )
