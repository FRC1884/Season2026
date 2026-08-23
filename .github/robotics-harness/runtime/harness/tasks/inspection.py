"""Deterministic, read-only repository inspection for task clarification."""

from __future__ import annotations

import hashlib
import json
import re
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from harness.policy.validator import DEFAULT_PROTECTED_PATHS, _matches

_IGNORED_DIRECTORIES = {
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    "__pycache__",
    "artifacts",
    "build",
    "dist",
    "node_modules",
    "target",
    "venv",
    ".venv",
}
_TEXT_SUFFIXES = {
    "",
    ".c",
    ".cc",
    ".cfg",
    ".cpp",
    ".gradle",
    ".h",
    ".hpp",
    ".ini",
    ".java",
    ".json",
    ".kts",
    ".md",
    ".py",
    ".robot",
    ".sh",
    ".toml",
    ".txt",
    ".xml",
    ".yaml",
    ".yml",
}
_TOKEN_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_]{2,}")
_SYMBOL_PATTERNS = (
    re.compile(r"^\s*(?:async\s+)?def\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(", re.MULTILINE),
    re.compile(
        r"^\s*(?:public|private|protected|static|final|\s)+"
        r"[A-Za-z0-9_<>,.?\[\]]+\s+([A-Za-z_][A-Za-z0-9_]*)\s*\(",
        re.MULTILINE,
    ),
    re.compile(r"^\s*([A-Z][A-Z0-9_]{3,})\s*=", re.MULTILINE),
    re.compile(r'"([A-Za-z_][A-Za-z0-9_]{3,})"\s*:', re.MULTILINE),
)
_STOP_WORDS = {
    "about",
    "agent",
    "behaviour",
    "behavior",
    "can",
    "change",
    "code",
    "could",
    "fix",
    "for",
    "from",
    "into",
    "please",
    "repository",
    "should",
    "task",
    "that",
    "the",
    "this",
    "with",
}


@dataclass(frozen=True, slots=True)
class RepositoryInspection:
    repository_digest_before: str
    repository_digest_after: str
    git_status_before: str
    git_status_after: str
    repository_unchanged: bool
    relevant_files: tuple[str, ...]
    relevant_symbols: tuple[str, ...]
    protected_relevant_paths: tuple[str, ...]
    instruction_files: tuple[str, ...]
    validation_commands: tuple[str, ...]
    file_count_examined: int
    repository_branch: str
    repository_commit: str

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["repo_unchanged"] = self.repository_unchanged
        return value


def _candidate_files(repository: Path) -> tuple[Path, ...]:
    files: list[Path] = []
    for path in repository.rglob("*"):
        try:
            relative = path.relative_to(repository)
        except ValueError:
            continue
        if any(part in _IGNORED_DIRECTORIES for part in relative.parts):
            continue
        if not path.is_file() or path.is_symlink():
            continue
        if path.suffix.lower() not in _TEXT_SUFFIXES:
            continue
        try:
            if path.stat().st_size > 256_000:
                continue
        except OSError:
            continue
        files.append(path)
        if len(files) >= 2_000:
            break
    return tuple(sorted(files))


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return ""


def _snapshot(repository: Path, files: tuple[Path, ...]) -> str:
    digest = hashlib.sha256()
    for path in files:
        relative = path.relative_to(repository).as_posix()
        digest.update(relative.encode("utf-8"))
        try:
            digest.update(path.read_bytes())
        except OSError:
            digest.update(b"<unreadable>")
    return digest.hexdigest()


def _git_status(repository: Path) -> str:
    try:
        completed = subprocess.run(
            ["git", "status", "--porcelain=v1", "--untracked-files=all"],
            cwd=repository,
            check=True,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    return completed.stdout


def _git_value(repository: Path, *arguments: str) -> str:
    try:
        completed = subprocess.run(
            ["git", *arguments],
            cwd=repository,
            check=True,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    return completed.stdout.strip()


def _request_tokens(request: str) -> set[str]:
    tokens = {token.lower() for token in _TOKEN_PATTERN.findall(request)}
    expanded = set(tokens)
    if {"drive", "drivetrain", "alignment", "speed"} & tokens:
        expanded.update({"drive", "drivetrain", "speed", "align"})
    if {"test", "testing"} & tokens:
        expanded.update({"test", "tests"})
    return expanded - _STOP_WORDS


def _score(relative: str, content: str, tokens: set[str]) -> int:
    lowered_path = relative.lower()
    lowered_content = content.lower()
    score = sum(8 for token in tokens if token in lowered_path)
    score += sum(min(lowered_content.count(token), 6) for token in tokens)
    if relative.endswith("AGENTS.md"):
        score += 3
    if "/tests/" in f"/{relative}" or relative.startswith("tests/"):
        score += 2
    if "/safety/" in f"/{relative}":
        score += 3
    return score


def _symbols(content: str, tokens: set[str]) -> tuple[str, ...]:
    found: list[str] = []
    for pattern in _SYMBOL_PATTERNS:
        for symbol in pattern.findall(content):
            if not tokens or any(token in symbol.lower() for token in tokens):
                found.append(symbol)
    if not found:
        for pattern in _SYMBOL_PATTERNS:
            found.extend(pattern.findall(content)[:3])
    return tuple(dict.fromkeys(found))


def _validation_commands(repository: Path, relative_files: set[str]) -> tuple[str, ...]:
    managed_metadata = repository / ".github/robotics-harness.json"
    if ".github/robotics-harness.json" in relative_files:
        try:
            metadata = json.loads(managed_metadata.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            metadata = None
        if isinstance(metadata, dict) and isinstance(metadata.get("commands"), dict):
            configured = [
                str(metadata["commands"].get(name, "")).strip() for name in ("build", "test")
            ]
            configured_commands = tuple(dict.fromkeys(command for command in configured if command))
            if configured_commands:
                return configured_commands

    commands: list[str] = []
    if "gradlew" in relative_files:
        build_file = _read_text(repository / "build.gradle") + _read_text(
            repository / "build.gradle.kts"
        )
        if "spotless" in build_file.lower():
            commands.append("./gradlew spotlessCheck")
        commands.extend(("./gradlew test", "./gradlew build"))
    if "pyproject.toml" in relative_files or any(
        path.startswith("tests/") for path in relative_files
    ):
        commands.append("python -m pytest -q tests")
    if "package.json" in relative_files:
        commands.append("npm test")
    return tuple(commands or ("run the repository's documented validation command",))


def inspect_repository(repository: Path | str, request: str) -> RepositoryInspection:
    """Inspect without writing and prove the examined repository content stayed unchanged."""

    repo = Path(repository).resolve()
    if not repo.is_dir():
        raise ValueError(f"repository does not exist or is not a directory: {repo}")
    files_before = _candidate_files(repo)
    digest_before = _snapshot(repo, files_before)
    status_before = _git_status(repo)
    tokens = _request_tokens(request)

    scored: list[tuple[int, str, str]] = []
    instruction_files: list[str] = []
    for path in files_before:
        relative = path.relative_to(repo).as_posix()
        content = _read_text(path)
        if path.name == "AGENTS.md":
            instruction_files.append(relative)
        score = _score(relative, content, tokens)
        if score > 0:
            scored.append((score, relative, content))
    scored.sort(key=lambda item: (-item[0], item[1]))
    relevant = scored[:12]
    if not relevant:
        relevant = [
            (1, path.relative_to(repo).as_posix(), _read_text(path)) for path in files_before[:8]
        ]
    relevant_files = tuple(item[1] for item in relevant)
    relevant_symbols = tuple(
        dict.fromkeys(
            f"{relative}:{symbol}"
            for _score_value, relative, content in relevant
            for symbol in _symbols(content, tokens)
        )
    )[:30]
    protected = tuple(
        sorted(
            path
            for path in relevant_files
            if any(_matches(path, rule.patterns) for rule in DEFAULT_PROTECTED_PATHS)
        )
    )

    files_after = _candidate_files(repo)
    digest_after = _snapshot(repo, files_after)
    status_after = _git_status(repo)
    return RepositoryInspection(
        repository_digest_before=digest_before,
        repository_digest_after=digest_after,
        git_status_before=status_before,
        git_status_after=status_after,
        repository_unchanged=(
            digest_before == digest_after
            and status_before == status_after
            and tuple(path.relative_to(repo) for path in files_before)
            == tuple(path.relative_to(repo) for path in files_after)
        ),
        relevant_files=relevant_files,
        relevant_symbols=relevant_symbols,
        protected_relevant_paths=protected,
        instruction_files=tuple(instruction_files),
        validation_commands=_validation_commands(
            repo, {path.relative_to(repo).as_posix() for path in files_before}
        ),
        file_count_examined=len(files_before),
        repository_branch=_git_value(repo, "branch", "--show-current"),
        repository_commit=_git_value(repo, "rev-parse", "HEAD"),
    )
