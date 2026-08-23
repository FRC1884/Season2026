"""Small, shell-free Git integration used by policy and learning-loop tools."""

from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

_DIFF_HEADER = re.compile(r"^diff --git a/(.+) b/(.+)$")
_COMMIT_HASH = re.compile(r"^[0-9a-fA-F]{40,64}$")


class GitCommandError(RuntimeError):
    """Raised when a Git command cannot produce the requested evidence."""


@dataclass(frozen=True, slots=True)
class ChangedFile:
    """One file section from a unified Git diff."""

    path: str
    patch: str
    old_path: str | None = None

    @property
    def affected_paths(self) -> tuple[str, ...]:
        if self.old_path and self.old_path != self.path:
            return (self.old_path, self.path)
        return (self.path,)


def run_git(repo: Path, *args: str) -> str:
    """Run Git with explicit arguments and return stripped standard output."""

    completed = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip() or "unknown Git error"
        raise GitCommandError(f"git {' '.join(args)} failed: {detail}")
    return completed.stdout.strip()


def diff_between(repo: Path, base_ref: str, head_ref: str) -> str:
    """Return a stable unified diff between two refs."""

    base_commit = resolve_commit(repo, base_ref)
    head_commit = resolve_commit(repo, head_ref)
    return run_git(
        repo,
        "diff",
        "--no-ext-diff",
        "--no-color",
        "--find-renames",
        "--unified=3",
        f"{base_commit}...{head_commit}",
        "--",
    )


def resolve_commit(repo: Path, reference: str) -> str:
    """Resolve an untrusted revision expression without treating it as an option."""

    if not reference.strip() or "\x00" in reference or "\n" in reference:
        raise GitCommandError("Git reference must be a nonempty single-line value")
    commit = run_git(
        repo,
        "rev-parse",
        "--verify",
        "--end-of-options",
        f"{reference}^{{commit}}",
    )
    if not _COMMIT_HASH.fullmatch(commit):
        raise GitCommandError(f"Git reference did not resolve to one commit: {reference}")
    return commit


def parse_unified_diff(diff_text: str) -> tuple[ChangedFile, ...]:
    """Parse file boundaries from a Git unified diff without interpreting hunks."""

    changes: list[ChangedFile] = []
    old_path: str | None = None
    new_path: str | None = None
    lines: list[str] = []

    def finish() -> None:
        if new_path is None:
            return
        changes.append(
            ChangedFile(path=new_path, old_path=old_path, patch="\n".join(lines).rstrip())
        )

    for line in diff_text.splitlines():
        match = _DIFF_HEADER.match(line)
        if match:
            finish()
            old_path, new_path = match.groups()
            lines = [line]
        elif new_path is not None:
            lines.append(line)
            if line.startswith("rename from "):
                old_path = line.removeprefix("rename from ")
            elif line.startswith("rename to "):
                new_path = line.removeprefix("rename to ")
            elif line == "+++ /dev/null":
                new_path = old_path
            elif line.startswith("+++ b/"):
                new_path = line.removeprefix("+++ b/")
    finish()
    return tuple(changes)
