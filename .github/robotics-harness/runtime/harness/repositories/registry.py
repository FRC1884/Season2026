"""Register, inspect, and isolate external local Git repositories safely."""

from __future__ import annotations

import json
import os
import re
import subprocess
from contextlib import suppress
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from harness.git import GitCommandError, parse_unified_diff, resolve_commit, run_git
from harness.policy import PolicyContext, PolicyReport, PolicyValidator
from harness.policy.validator import DEFAULT_PROTECTED_PATHS, ProtectedPathRule

DEFAULT_WPILIB_PROTECTED_PATHS = (
    "src/main/java/**/constants/**",
    "src/main/java/**/*Constants.java",
    "src/main/java/**/safety/**",
    "src/main/deploy/**",
    ".github/workflows/**",
    "build.gradle",
    "settings.gradle",
    "gradle/**",
    "vendordeps/**",
    "AGENTS.md",
)

DEFAULT_LOCAL_EXCLUDE_PATTERNS = (
    ".omx/",
    ".codex/",
    "artifacts/",
    ".pytest_cache/",
    ".mypy_cache/",
    ".ruff_cache/",
    ".coverage",
)

DEFAULT_RUNTIME_ARTIFACT_PATTERNS = (
    ".omx/**",
    ".codex/**",
    "artifacts/**",
    ".pytest_cache/**",
    ".mypy_cache/**",
    ".ruff_cache/**",
    ".coverage",
)

_WORKSPACE_EXCLUDE_START = "# robotics-agentic-development-harness:start"
_WORKSPACE_EXCLUDE_END = "# robotics-agentic-development-harness:end"

_SYMBOL_PATTERN = re.compile(
    r"^\s*(?:public|protected|private)?\s*"
    r"(?:(?:static|final|abstract|synchronized)\s+)*"
    r"(?:class|interface|enum|record|void|boolean|byte|char|short|int|long|float|double|"
    r"[A-Z][A-Za-z0-9_<>, ?.\[\]]*)\s+"
    r"([A-Za-z_][A-Za-z0-9_]*)\s*(?:\(|\{|=)",
    re.MULTILINE,
)


class RepositorySafetyError(RuntimeError):
    """Raised when an external repository operation would violate a guardrail."""


@dataclass(frozen=True, slots=True)
class ScopeBudget:
    """Maximum changed-line budgets for one task risk class."""

    production: int
    tests: int
    docs: int
    total: int

    def __post_init__(self) -> None:
        for field_name, value in (
            ("production", self.production),
            ("tests", self.tests),
            ("docs", self.docs),
            ("total", self.total),
        ):
            if value < 1:
                raise ValueError(f"scope budget {field_name} must be positive")

    def to_dict(self) -> dict[str, int]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> ScopeBudget:
        try:
            return cls(
                production=int(value["production"]),
                tests=int(value["tests"]),
                docs=int(value["docs"]),
                total=int(value["total"]),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                "scope budgets must define positive production/tests/docs/total"
            ) from error

    def with_total_override(self, total: int | None) -> ScopeBudget:
        if total is None:
            return self
        if total < 1:
            raise ValueError("scope budget total override must be positive")
        return ScopeBudget(
            production=self.production,
            tests=self.tests,
            docs=self.docs,
            total=total,
        )


@dataclass(frozen=True, slots=True)
class RepositoryScopeBudgets:
    """Risk-classified diff budgets reused by scope enforcement."""

    low: ScopeBudget
    medium: ScopeBudget
    high: ScopeBudget
    critical: ScopeBudget

    def to_dict(self) -> dict[str, dict[str, int]]:
        return {
            "low": self.low.to_dict(),
            "medium": self.medium.to_dict(),
            "high": self.high.to_dict(),
            "critical": self.critical.to_dict(),
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> RepositoryScopeBudgets:
        try:
            return cls(
                low=ScopeBudget.from_dict(_as_dict(value.get("low"), "scope_budgets.low")),
                medium=ScopeBudget.from_dict(_as_dict(value.get("medium"), "scope_budgets.medium")),
                high=ScopeBudget.from_dict(_as_dict(value.get("high"), "scope_budgets.high")),
                critical=ScopeBudget.from_dict(
                    _as_dict(value.get("critical"), "scope_budgets.critical")
                ),
            )
        except ValueError as error:
            raise ValueError(f"invalid scope budgets: {error}") from error

    def for_risk(self, risk: str) -> ScopeBudget:
        normalized = risk.strip().lower()
        if normalized == "critical":
            return self.critical
        if normalized == "high":
            return self.high
        if normalized == "medium":
            return self.medium
        return self.low


@dataclass(frozen=True, slots=True)
class WorkspacePreparation:
    """Local workspace routing and isolation state for an external repository."""

    repository_path: str
    runtime_root: str
    runtime_contract_path: str
    local_exclude_path: str
    local_exclude_patterns: tuple[str, ...]
    git_identity_name: str = ""
    git_identity_email: str = ""

    def to_dict(self) -> dict[str, object]:
        value = asdict(self)
        value["local_exclude_patterns"] = list(self.local_exclude_patterns)
        return value


def _as_dict(value: Any, location: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{location} must be an object")
    return value


def _default_scope_budgets(repository_type: str) -> RepositoryScopeBudgets:
    if repository_type == "wpilib-java":
        return RepositoryScopeBudgets(
            low=ScopeBudget(production=80, tests=200, docs=60, total=240),
            medium=ScopeBudget(production=180, tests=320, docs=100, total=440),
            high=ScopeBudget(production=320, tests=500, docs=150, total=750),
            critical=ScopeBudget(production=400, tests=650, docs=180, total=950),
        )
    return RepositoryScopeBudgets(
        low=ScopeBudget(production=60, tests=160, docs=60, total=200),
        medium=ScopeBudget(production=120, tests=220, docs=80, total=280),
        high=ScopeBudget(production=180, tests=280, docs=100, total=360),
        critical=ScopeBudget(production=220, tests=320, docs=120, total=420),
    )


@dataclass(frozen=True, slots=True)
class RegisteredRepository:
    """One user-configured external local repository."""

    name: str
    repository_id: str
    path: str
    repository_type: str
    default_branch: str
    protected_paths: tuple[str, ...]
    expected_remote: str = ""
    scope_budgets: RepositoryScopeBudgets = field(
        default_factory=lambda: _default_scope_budgets("unknown")
    )
    local_exclude_patterns: tuple[str, ...] = DEFAULT_LOCAL_EXCLUDE_PATTERNS
    runtime_artifact_patterns: tuple[str, ...] = DEFAULT_RUNTIME_ARTIFACT_PATTERNS
    git_identity_name: str = "Software Team Member"
    git_identity_email: str = "software-team-member@agentic-harness.local"

    def to_dict(self) -> dict[str, object]:
        value = asdict(self)
        value["protected_paths"] = list(self.protected_paths)
        value["scope_budgets"] = self.scope_budgets.to_dict()
        value["local_exclude_patterns"] = list(self.local_exclude_patterns)
        value["runtime_artifact_patterns"] = list(self.runtime_artifact_patterns)
        return value

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> RegisteredRepository:
        protected = value.get("protected_paths", [])
        if not isinstance(protected, list) or not all(isinstance(item, str) for item in protected):
            raise ValueError("protected_paths must be a list of strings")
        local_excludes = value.get("local_exclude_patterns", list(DEFAULT_LOCAL_EXCLUDE_PATTERNS))
        if not isinstance(local_excludes, list) or not all(
            isinstance(item, str) for item in local_excludes
        ):
            raise ValueError("local_exclude_patterns must be a list of strings")
        runtime_artifacts = value.get(
            "runtime_artifact_patterns",
            list(DEFAULT_RUNTIME_ARTIFACT_PATTERNS),
        )
        if not isinstance(runtime_artifacts, list) or not all(
            isinstance(item, str) for item in runtime_artifacts
        ):
            raise ValueError("runtime_artifact_patterns must be a list of strings")
        repository_type = str(value.get("repository_type", "unknown"))
        repository_path = str(value["path"])
        expected_remote = str(value.get("expected_remote", ""))
        repository_id = str(value.get("repository_id", "")).strip() or canonical_repository_id(
            path=repository_path,
            expected_remote=expected_remote,
        )
        return cls(
            name=str(value["name"]),
            repository_id=repository_id,
            path=repository_path,
            repository_type=repository_type,
            default_branch=str(value.get("default_branch", "main")),
            protected_paths=tuple(protected),
            expected_remote=expected_remote,
            scope_budgets=RepositoryScopeBudgets.from_dict(
                _as_dict(
                    value.get("scope_budgets", _default_scope_budgets(repository_type).to_dict()),
                    "scope_budgets",
                )
            ),
            local_exclude_patterns=tuple(local_excludes),
            runtime_artifact_patterns=tuple(runtime_artifacts),
            git_identity_name=str(value.get("git_identity_name", "Software Team Member")),
            git_identity_email=str(
                value.get(
                    "git_identity_email",
                    "software-team-member@agentic-harness.local",
                )
            ),
        )


@dataclass(frozen=True, slots=True)
class RepositoryInspection:
    """Read-only evidence gathered from a registered repository."""

    name: str
    resolved_path: str
    current_branch: str
    remote_default_branch: str
    head_commit: str
    clean: bool
    dirty_paths: tuple[str, ...]
    remotes: tuple[str, ...]
    instruction_files: tuple[str, ...]
    build_files: tuple[str, ...]
    test_files: tuple[str, ...]
    relevant_files: tuple[str, ...]
    relevant_symbols: tuple[str, ...]
    validation_commands: tuple[str, ...]
    expected_remote: str
    remote_identity_verified: bool

    def to_dict(self) -> dict[str, object]:
        value = asdict(self)
        for key in (
            "dirty_paths",
            "remotes",
            "instruction_files",
            "build_files",
            "test_files",
            "relevant_files",
            "relevant_symbols",
            "validation_commands",
        ):
            value[key] = list(value[key])
        return value


@dataclass(frozen=True, slots=True)
class WorktreeResult:
    """An isolated demo branch and worktree created from a recorded base."""

    source_repository: str
    base_ref: str
    base_commit: str
    branch: str
    worktree_path: str
    workspace_preparation: WorkspacePreparation

    def to_dict(self) -> dict[str, object]:
        value = asdict(self)
        value["workspace_preparation"] = self.workspace_preparation.to_dict()
        return value


def canonical_repository_id(*, path: Path | str, expected_remote: str = "") -> str:
    """Derive one stable identity for a repository across aliases and worktrees."""

    if expected_remote.strip():
        return f"remote:{ExternalRepositoryInspector._normalize_remote(expected_remote)}"
    resolved = Path(path).expanduser().resolve()
    try:
        common = Path(run_git(resolved, "rev-parse", "--git-common-dir"))
        if not common.is_absolute():
            common = resolved / common
        return f"local:{common.resolve()}"
    except GitCommandError:
        return f"local:{resolved}"


class RepositoryRegistry:
    """Persist external repository registrations outside versioned config."""

    def __init__(self, path: Path | str = "artifacts/repositories.json") -> None:
        self.path = Path(path)

    def _load(self) -> dict[str, RegisteredRepository]:
        if not self.path.exists():
            return {}
        try:
            raw: Any = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise RepositorySafetyError(
                f"cannot read repository registry {self.path}: {error}"
            ) from error
        if not isinstance(raw, dict) or raw.get("schema_version") != 1:
            raise RepositorySafetyError("repository registry must use schema_version 1")
        repositories = raw.get("repositories", [])
        if not isinstance(repositories, list):
            raise RepositorySafetyError("repository registry entries must be a list")
        try:
            entries = [RegisteredRepository.from_dict(item) for item in repositories]
        except (KeyError, TypeError, ValueError) as error:
            raise RepositorySafetyError(f"invalid repository registration: {error}") from error
        return {entry.name: entry for entry in entries}

    def _save(self, entries: dict[str, RegisteredRepository]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": 1,
            "repositories": [entries[name].to_dict() for name in sorted(entries)],
        }
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        descriptor = os.open(
            temporary,
            flags,
            0o600,
        )
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
            stream.flush()
        os.chmod(temporary, 0o600)
        temporary.replace(self.path)
        os.chmod(self.path, 0o600)

    def register(
        self,
        *,
        name: str,
        path: Path | str,
        repository_type: str,
        default_branch: str = "",
        protected_paths: tuple[str, ...] | None = None,
        expected_remote: str = "",
        scope_budgets: RepositoryScopeBudgets | dict[str, Any] | None = None,
        local_exclude_patterns: tuple[str, ...] | None = None,
        runtime_artifact_patterns: tuple[str, ...] | None = None,
        git_identity_name: str = "Software Team Member",
        git_identity_email: str = "software-team-member@agentic-harness.local",
    ) -> RegisteredRepository:
        normalized_name = name.strip()
        valid_name = re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", normalized_name)
        if not normalized_name or valid_name is None:
            raise RepositorySafetyError(
                "repository name must contain only letters, digits, dot, underscore, or hyphen"
            )
        resolved = Path(path).expanduser().resolve()
        if not resolved.is_dir():
            raise RepositorySafetyError(f"repository path does not exist: {resolved}")
        try:
            resolved = Path(run_git(resolved, "rev-parse", "--show-toplevel")).resolve()
        except GitCommandError as error:
            raise RepositorySafetyError(f"path is not a Git repository: {resolved}") from error

        patterns = (
            DEFAULT_WPILIB_PROTECTED_PATHS
            if protected_paths is None and repository_type == "wpilib-java"
            else protected_paths or ()
        )
        normalized_remote = expected_remote.strip()
        repository_identifier = canonical_repository_id(
            path=resolved, expected_remote=normalized_remote
        )
        selected_default_branch = (
            default_branch.strip() or ExternalRepositoryInspector.discover_default_branch(resolved)
        )
        selected_scope_budgets = (
            _default_scope_budgets(repository_type)
            if scope_budgets is None
            else (
                scope_budgets
                if isinstance(scope_budgets, RepositoryScopeBudgets)
                else RepositoryScopeBudgets.from_dict(_as_dict(scope_budgets, "scope_budgets"))
            )
        )
        registration = RegisteredRepository(
            name=normalized_name,
            repository_id=repository_identifier,
            path=str(resolved),
            repository_type=repository_type.strip() or "unknown",
            default_branch=selected_default_branch,
            protected_paths=tuple(patterns),
            expected_remote=normalized_remote,
            scope_budgets=selected_scope_budgets,
            local_exclude_patterns=tuple(local_exclude_patterns or DEFAULT_LOCAL_EXCLUDE_PATTERNS),
            runtime_artifact_patterns=tuple(
                runtime_artifact_patterns or DEFAULT_RUNTIME_ARTIFACT_PATTERNS
            ),
            git_identity_name=git_identity_name.strip() or "Software Team Member",
            git_identity_email=git_identity_email.strip()
            or "software-team-member@agentic-harness.local",
        )
        entries = self._load()
        duplicate = next(
            (
                entry
                for alias, entry in entries.items()
                if alias != registration.name and entry.repository_id == registration.repository_id
            ),
            None,
        )
        if duplicate is not None:
            raise RepositorySafetyError(
                "repository is already registered under alias "
                f"{duplicate.name!r} for canonical id {duplicate.repository_id!r}"
            )
        entries[registration.name] = registration
        self._save(entries)
        return registration

    def get(self, name: str) -> RegisteredRepository:
        try:
            return self._load()[name]
        except KeyError as error:
            raise RepositorySafetyError(f"repository is not registered: {name}") from error

    def list(self) -> tuple[RegisteredRepository, ...]:
        entries = self._load()
        return tuple(entries[name] for name in sorted(entries))

    def find_by_path(self, path: Path | str) -> RegisteredRepository | None:
        resolved = Path(path).expanduser().resolve()
        try:
            resolved = Path(run_git(resolved, "rev-parse", "--show-toplevel")).resolve()
        except GitCommandError:
            return None
        entries = self._load()
        direct = [entry for entry in entries.values() if Path(entry.path).resolve() == resolved]
        if direct:
            return sorted(direct, key=lambda entry: entry.name)[0]
        identifier = canonical_repository_id(path=resolved)
        matches = [entry for entry in entries.values() if entry.repository_id == identifier]
        if matches:
            return sorted(matches, key=lambda entry: entry.name)[0]
        remotes = tuple(line for line in run_git(resolved, "remote", "-v").splitlines() if line)
        origin_urls = [
            parts[1] for line in remotes if len(parts := line.split()) >= 2 and parts[0] == "origin"
        ]
        if origin_urls:
            remote_identifier = canonical_repository_id(
                path=resolved, expected_remote=origin_urls[0]
            )
            remote_matches = [
                entry for entry in entries.values() if entry.repository_id == remote_identifier
            ]
            if remote_matches:
                return sorted(remote_matches, key=lambda entry: entry.name)[0]
        return None


class ExternalRepositoryInspector:
    """Perform read-only discovery and confirmation-gated worktree creation."""

    @staticmethod
    def discover_default_branch(repo: Path | str) -> str:
        """Discover the remote default branch without mutating repository metadata."""
        resolved = Path(repo).expanduser().resolve()
        remote_head = subprocess.run(
            [
                "git",
                "-C",
                str(resolved),
                "symbolic-ref",
                "--quiet",
                "--short",
                "refs/remotes/origin/HEAD",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if remote_head.returncode == 0:
            value = remote_head.stdout.strip()
            if value.startswith("origin/") and len(value) > len("origin/"):
                return value.removeprefix("origin/")

        local_head = subprocess.run(
            [
                "git",
                "-C",
                str(resolved),
                "symbolic-ref",
                "--quiet",
                "--short",
                "HEAD",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if local_head.returncode == 0 and local_head.stdout.strip():
            return local_head.stdout.strip()
        raise RepositorySafetyError(
            "cannot discover the default branch from origin/HEAD or the current branch"
        )

    def inspect(
        self,
        registration: RegisteredRepository,
        *,
        request: str = "",
        maximum_files: int = 12,
    ) -> RepositoryInspection:
        repo = Path(registration.path).resolve()
        if maximum_files < 1:
            raise ValueError("maximum_files must be positive")
        top_level = Path(run_git(repo, "rev-parse", "--show-toplevel")).resolve()
        if top_level != repo:
            repo = top_level

        status_result = subprocess.run(
            [
                "git",
                "-C",
                str(repo),
                "status",
                "--porcelain=v1",
                "--untracked-files=all",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if status_result.returncode != 0:
            detail = status_result.stderr.strip() or "unknown Git status error"
            raise RepositorySafetyError(f"cannot inspect repository status: {detail}")
        status_lines = status_result.stdout.splitlines()
        dirty_paths = tuple(line[3:] if len(line) > 3 else line for line in status_lines if line)
        current_branch = run_git(repo, "branch", "--show-current")
        remote_default_branch = self._remote_default_branch(repo)
        head_commit = run_git(repo, "rev-parse", "HEAD")
        remotes = tuple(line for line in run_git(repo, "remote", "-v").splitlines() if line)
        remote_identity_verified = self._remote_matches(
            remotes,
            registration.expected_remote,
        )

        tracked = tuple(line for line in run_git(repo, "ls-files").splitlines() if line)
        instruction_files = tuple(
            path
            for path in tracked
            if path == "AGENTS.md"
            or path.endswith("/AGENTS.md")
            or path in {"README.md", "ARCHITECTURE.md", "docs/CODEX_COORDINATION.md"}
        )
        build_files = tuple(
            path
            for path in tracked
            if path
            in {
                "build.gradle",
                "build.gradle.kts",
                "settings.gradle",
                "settings.gradle.kts",
                "pom.xml",
                "pyproject.toml",
            }
            or path.startswith(".github/workflows/")
        )
        test_files = tuple(
            path
            for path in tracked
            if path.startswith(("src/test/", "tests/", "test/"))
            and Path(path).suffix in {".java", ".py", ".kt"}
        )
        relevant_files = self._relevant_files(
            repo,
            tracked,
            request=request,
            maximum_files=maximum_files,
        )
        relevant_symbols = self._symbols(repo, relevant_files)
        validation_commands = self._validation_commands(repo, tracked)
        return RepositoryInspection(
            name=registration.name,
            resolved_path=str(repo),
            current_branch=current_branch,
            remote_default_branch=remote_default_branch,
            head_commit=head_commit,
            clean=not status_lines,
            dirty_paths=dirty_paths,
            remotes=remotes,
            instruction_files=instruction_files,
            build_files=build_files,
            test_files=test_files,
            relevant_files=relevant_files,
            relevant_symbols=relevant_symbols,
            validation_commands=validation_commands,
            expected_remote=registration.expected_remote,
            remote_identity_verified=remote_identity_verified,
        )

    @staticmethod
    def _remote_default_branch(repo: Path) -> str:
        completed = subprocess.run(
            [
                "git",
                "-C",
                str(repo),
                "symbolic-ref",
                "--quiet",
                "--short",
                "refs/remotes/origin/HEAD",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            return ""
        value = completed.stdout.strip()
        return value.removeprefix("origin/") if value.startswith("origin/") else ""

    @staticmethod
    def _remote_matches(remotes: tuple[str, ...], expected: str) -> bool:
        if not expected:
            return True
        normalized_expected = ExternalRepositoryInspector._normalize_remote(expected)
        origin_urls = [
            parts[1] for line in remotes if len(parts := line.split()) >= 2 and parts[0] == "origin"
        ]
        return bool(origin_urls) and all(
            ExternalRepositoryInspector._normalize_remote(url) == normalized_expected
            for url in origin_urls
        )

    @staticmethod
    def _normalize_remote(remote: str) -> str:
        normalized = remote.strip().removesuffix("/").removesuffix(".git")
        if normalized.startswith("git@github.com:"):
            normalized = "github.com/" + normalized.removeprefix("git@github.com:")
        elif normalized.startswith("ssh://git@github.com/"):
            normalized = "github.com/" + normalized.removeprefix("ssh://git@github.com/")
        elif normalized.startswith("https://github.com/"):
            normalized = "github.com/" + normalized.removeprefix("https://github.com/")
        return normalized.lower()

    @staticmethod
    def _request_tokens(request: str) -> tuple[str, ...]:
        ignored = {
            "a",
            "an",
            "and",
            "can",
            "code",
            "fix",
            "for",
            "in",
            "make",
            "our",
            "small",
            "something",
            "the",
            "to",
            "useful",
        }
        tokens = {
            token
            for token in re.findall(r"[a-z0-9]+", request.lower())
            if len(token) >= 3 and token not in ignored
        }
        return tuple(sorted(tokens))

    def _relevant_files(
        self,
        repo: Path,
        tracked: tuple[str, ...],
        *,
        request: str,
        maximum_files: int,
    ) -> tuple[str, ...]:
        tokens = self._request_tokens(request)
        candidates: list[tuple[int, str]] = []
        for path in tracked:
            suffix = Path(path).suffix.lower()
            if suffix not in {".java", ".kt", ".py", ".js", ".json"}:
                continue
            lowered = path.lower()
            score = sum(token in lowered for token in tokens) * 3
            if "/test/" in lowered or lowered.startswith("tests/"):
                score += 1
            if score == 0 and tokens:
                try:
                    content = (repo / path).read_text(encoding="utf-8", errors="ignore")
                except OSError:
                    continue
                score = sum(token in content.lower() for token in tokens)
            if score:
                candidates.append((score, path))
        candidates.sort(key=lambda item: (-item[0], item[1]))
        if not candidates:
            candidates = [
                (0, path)
                for path in tracked
                if path.startswith(("src/main/", "src/test/", "tests/"))
                and Path(path).suffix.lower() in {".java", ".kt", ".py"}
            ]
        return tuple(path for _score, path in candidates[:maximum_files])

    @staticmethod
    def _symbols(repo: Path, paths: tuple[str, ...]) -> tuple[str, ...]:
        symbols: list[str] = []
        for path in paths:
            if Path(path).suffix not in {".java", ".kt"}:
                continue
            try:
                content = (repo / path).read_text(encoding="utf-8", errors="ignore")
            except OSError:
                continue
            for name in _SYMBOL_PATTERN.findall(content):
                reference = f"{path}:{name}"
                if reference not in symbols:
                    symbols.append(reference)
                if len(symbols) >= 30:
                    return tuple(symbols)
        return tuple(symbols)

    @staticmethod
    def _validation_commands(repo: Path, tracked: tuple[str, ...]) -> tuple[str, ...]:
        commands: list[str] = []
        if "gradlew" in tracked and "build.gradle" in tracked:
            commands.extend(("./gradlew test", "./gradlew build"))
            workflow_paths = (path for path in tracked if path.startswith(".github/workflows/"))
            for path in workflow_paths:
                try:
                    workflow = (repo / path).read_text(encoding="utf-8", errors="ignore")
                except OSError:
                    continue
                for command in re.findall(
                    r"(?m)^\s*run:\s*(\./gradlew\s+[A-Za-z0-9:_-]+)", workflow
                ):
                    if "deploy" not in command.lower() and command not in commands:
                        commands.append(command)
        elif "pyproject.toml" in tracked:
            commands.append("python -m pytest")
        return tuple(commands)

    def create_worktree(
        self,
        registration: RegisteredRepository,
        *,
        task_state: str,
        confirmation_digest: str,
        base_ref: str | None = None,
        branch_prefix: str = "demo/agentic-harness-learning-loop",
        destination_parent: Path | str | None = None,
        expected_base_commit: str = "",
    ) -> WorktreeResult:
        if task_state != "approved_for_implementation" or not confirmation_digest.strip():
            raise RepositorySafetyError(
                "worktree creation requires a confirmed approved_for_implementation task"
            )
        if not registration.expected_remote:
            raise RepositorySafetyError(
                "worktree creation requires a configured expected origin remote"
            )
        inspection = self.inspect(registration)
        if not inspection.remote_identity_verified:
            raise RepositorySafetyError(
                "target repository remotes do not match the configured expected remote"
            )
        if not inspection.clean:
            paths = ", ".join(inspection.dirty_paths[:8])
            raise RepositorySafetyError(
                f"target working tree is dirty; refusing demo worktree creation: {paths}"
            )
        if inspection.current_branch != registration.default_branch:
            raise RepositorySafetyError("target checkout is not on the configured default branch")
        if (
            inspection.remote_default_branch
            and inspection.remote_default_branch != registration.default_branch
        ):
            raise RepositorySafetyError(
                "configured default branch does not match the remote default branch"
            )
        if expected_base_commit and inspection.head_commit != expected_base_commit:
            raise RepositorySafetyError(
                "target base changed after plan confirmation; reconfirmation is required"
            )

        repo = Path(inspection.resolved_path)
        requested_base = base_ref or registration.default_branch
        try:
            base_commit = resolve_commit(repo, requested_base)
        except GitCommandError as error:
            raise RepositorySafetyError(
                f"cannot resolve verified base ref {requested_base!r}"
            ) from error
        if expected_base_commit and base_commit != expected_base_commit:
            raise RepositorySafetyError(
                "requested base does not match the plan-confirmed repository commit"
            )

        branch = self._unique_branch(repo, branch_prefix)
        parent = (
            Path(destination_parent).expanduser().resolve()
            if destination_parent is not None
            else repo.parent
        )
        if parent == repo or parent.is_relative_to(repo):
            raise RepositorySafetyError(
                "external demo worktree must be created outside the target checkout"
            )
        parent.mkdir(parents=True, exist_ok=True)
        worktree = self._unique_worktree_path(parent, f"{repo.name}-harness-demo")
        try:
            run_git(repo, "worktree", "add", str(worktree), "-b", branch, base_commit)
        except GitCommandError as error:
            raise RepositorySafetyError(f"cannot create isolated demo worktree: {error}") from error
        try:
            preparation = self.prepare_workspace(
                registration,
                workspace=worktree,
                runtime_parent=parent / ".harness-runtime",
                configure_git_identity=True,
            )
        except RepositorySafetyError:
            with suppress(GitCommandError):
                run_git(repo, "worktree", "remove", "--force", str(worktree))
            raise
        return WorktreeResult(
            source_repository=str(repo),
            base_ref=requested_base,
            base_commit=base_commit,
            branch=branch,
            worktree_path=str(worktree),
            workspace_preparation=preparation,
        )

    def prepare_workspace(
        self,
        registration: RegisteredRepository,
        *,
        workspace: Path | str,
        runtime_parent: Path | str | None = None,
        configure_git_identity: bool = False,
    ) -> WorkspacePreparation:
        root = Path(workspace).expanduser().resolve()
        try:
            root = Path(run_git(root, "rev-parse", "--show-toplevel")).resolve()
        except GitCommandError as error:
            raise RepositorySafetyError(f"workspace is not a Git repository: {root}") from error
        runtime_root = self._runtime_root(root, runtime_parent)
        local_exclude_path = Path(run_git(root, "rev-parse", "--git-path", "info/exclude"))
        if not local_exclude_path.is_absolute():
            local_exclude_path = root / local_exclude_path
        local_exclude_path = local_exclude_path.resolve()
        self._ensure_local_excludes(
            local_exclude_path,
            registration.local_exclude_patterns,
        )
        if configure_git_identity:
            self._configure_worktree_identity(root, registration)
        contract_path = runtime_root / "runtime-contract.json"
        runtime_root.mkdir(parents=True, exist_ok=True)
        contract = {
            "schema_version": 1,
            "repository_path": str(root),
            "runtime_root": str(runtime_root),
            "artifacts_root": str(runtime_root / "artifacts"),
            "omx_state_root": str(runtime_root / ".omx"),
            "local_exclude_path": str(local_exclude_path),
            "local_exclude_patterns": list(registration.local_exclude_patterns),
            "runtime_artifact_patterns": list(registration.runtime_artifact_patterns),
            "git_identity": {
                "name": registration.git_identity_name if configure_git_identity else "",
                "email": registration.git_identity_email if configure_git_identity else "",
                "scope": "worktree-local" if configure_git_identity else "not-configured",
            },
            "notes": [
                "Route harness runtime and audit artifacts outside the target repository.",
                "Do not rely on local excludes as a security boundary.",
            ],
        }
        contract_path.write_text(
            json.dumps(contract, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return WorkspacePreparation(
            repository_path=str(root),
            runtime_root=str(runtime_root),
            runtime_contract_path=str(contract_path),
            local_exclude_path=str(local_exclude_path),
            local_exclude_patterns=registration.local_exclude_patterns,
            git_identity_name=registration.git_identity_name if configure_git_identity else "",
            git_identity_email=registration.git_identity_email if configure_git_identity else "",
        )

    @staticmethod
    def _unique_branch(repo: Path, prefix: str) -> str:
        normalized = prefix.strip().strip("/")
        if not normalized:
            raise RepositorySafetyError("branch prefix is required")
        if normalized.startswith("-"):
            raise RepositorySafetyError("branch prefix is not a valid local Git branch")
        valid_ref = subprocess.run(
            [
                "git",
                "-C",
                str(repo),
                "check-ref-format",
                f"refs/heads/{normalized}",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if valid_ref.returncode != 0:
            raise RepositorySafetyError("branch prefix is not a valid local Git branch")
        for index in range(1, 1000):
            candidate = normalized if index == 1 else f"{normalized}-{index}"
            # `show-ref --exists` is not available on every supported Git version.
            result = subprocess.run(
                [
                    "git",
                    "-C",
                    str(repo),
                    "show-ref",
                    "--verify",
                    "--quiet",
                    f"refs/heads/{candidate}",
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            if result.returncode == 1:
                return candidate
            if result.returncode not in {0, 1}:
                raise RepositorySafetyError("cannot inspect existing demo branches")
        raise RepositorySafetyError("could not allocate a unique demo branch")

    @staticmethod
    def _unique_worktree_path(parent: Path, name: str) -> Path:
        for index in range(1, 1000):
            candidate = parent / (name if index == 1 else f"{name}-{index}")
            if not candidate.exists():
                return candidate
        raise RepositorySafetyError("could not allocate a unique worktree directory")

    @staticmethod
    def _runtime_root(workspace: Path, runtime_parent: Path | str | None) -> Path:
        parent = (
            Path(runtime_parent).expanduser().resolve()
            if runtime_parent is not None
            else workspace.parent / ".harness-runtime"
        )
        candidate = (parent / workspace.name).resolve()
        if candidate == workspace or candidate.is_relative_to(workspace):
            raise RepositorySafetyError(
                "external workspace runtime must remain outside the repository"
            )
        return candidate

    @staticmethod
    def _ensure_local_excludes(path: Path, patterns: tuple[str, ...]) -> None:
        existing = path.read_text(encoding="utf-8") if path.exists() else ""
        managed_block = "\n".join((_WORKSPACE_EXCLUDE_START, *patterns, _WORKSPACE_EXCLUDE_END))
        pattern = re.compile(
            rf"{re.escape(_WORKSPACE_EXCLUDE_START)}.*?{re.escape(_WORKSPACE_EXCLUDE_END)}\n?",
            re.DOTALL,
        )
        if pattern.search(existing):
            updated = pattern.sub(managed_block + "\n", existing, count=1)
        else:
            pieces = [existing.rstrip("\n")] if existing.strip() else []
            pieces.append(managed_block)
            updated = "\n".join(piece for piece in pieces if piece) + "\n"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(updated, encoding="utf-8")

    @staticmethod
    def _configure_worktree_identity(root: Path, registration: RegisteredRepository) -> None:
        try:
            run_git(root, "config", "extensions.worktreeConfig", "true")
            run_git(root, "config", "--worktree", "user.name", registration.git_identity_name)
            run_git(root, "config", "--worktree", "user.email", registration.git_identity_email)
        except GitCommandError as error:
            raise RepositorySafetyError(
                f"cannot configure worktree-local Git identity: {error}"
            ) from error


def evaluate_protected_diff(
    diff_text: str,
    protected_paths: tuple[str, ...],
    *,
    approval_groups: frozenset[str] = frozenset(),
) -> PolicyReport:
    """Evaluate a proposed diff without applying it to the target repository."""

    changes = parse_unified_diff(diff_text)
    rule = ProtectedPathRule(
        identifier="external-repository-protected",
        patterns=protected_paths,
        required_approval="Safety Code Owners",
        explanation="external robot repository protected paths require safety review",
    )
    return PolicyValidator((rule,)).validate(
        changes,
        PolicyContext(approval_groups=approval_groups),
    )


def policy_validator_for_registration(
    registration: RegisteredRepository,
) -> PolicyValidator:
    """Build the task policy that is persisted for a registered target."""

    external = ProtectedPathRule(
        identifier="external-repository-protected",
        patterns=registration.protected_paths,
        required_approval="Safety Code Owners",
        explanation="external robot repository protected paths require safety review",
    )
    return PolicyValidator((*DEFAULT_PROTECTED_PATHS, external))
