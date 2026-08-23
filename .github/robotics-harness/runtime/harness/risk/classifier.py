"""Configurable deterministic path- and content-based risk classifier."""

from __future__ import annotations

import fnmatch
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from harness.git import ChangedFile
from harness.models import RiskLevel

DEFAULT_RULES_PATH = Path(__file__).resolve().parents[2] / "config" / "risk_rules.yaml"


class RiskRulesError(ValueError):
    """Raised when the risk-rules configuration is invalid."""


@dataclass(frozen=True, slots=True)
class RiskRule:
    identifier: str
    risk: RiskLevel
    paths: tuple[str, ...]
    content_patterns: tuple[re.Pattern[str], ...]
    description: str


@dataclass(frozen=True, slots=True)
class RiskMatch:
    rule_id: str
    risk: RiskLevel
    path: str
    reason: str

    def to_dict(self) -> dict[str, str]:
        return {
            "rule_id": self.rule_id,
            "risk": self.risk.label,
            "path": self.path,
            "reason": self.reason,
        }


@dataclass(frozen=True, slots=True)
class RiskAssessment:
    risk: RiskLevel
    matches: tuple[RiskMatch, ...]
    changed_files: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "risk": self.risk.label,
            "changed_files": list(self.changed_files),
            "matches": [match.to_dict() for match in self.matches],
        }


def _strings(value: object, location: str) -> tuple[str, ...]:
    if value is None:
        return ()
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise RiskRulesError(f"{location} must be a list of strings")
    return tuple(value)


class RiskClassifier:
    """Apply the highest matching configured risk to a set of changes."""

    def __init__(self, rules_path: Path | str = DEFAULT_RULES_PATH) -> None:
        self.rules_path = Path(rules_path)
        self.default_risk, self.rules = self._load(self.rules_path)

    @staticmethod
    def _load(path: Path) -> tuple[RiskLevel, tuple[RiskRule, ...]]:
        try:
            raw: Any = json.loads(path.read_text(encoding="utf-8"))
        except OSError as error:
            raise RiskRulesError(f"cannot read risk rules at {path}: {error}") from error
        except json.JSONDecodeError as error:
            raise RiskRulesError(
                f"{path} must contain JSON-compatible YAML: {error.msg}"
            ) from error
        if not isinstance(raw, dict):
            raise RiskRulesError("risk rules must be an object")
        try:
            default = RiskLevel.parse(str(raw.get("default_risk", "low")))
        except ValueError as error:
            raise RiskRulesError(str(error)) from error
        raw_rules = raw.get("rules")
        if not isinstance(raw_rules, list):
            raise RiskRulesError("rules must be a list")

        rules: list[RiskRule] = []
        identifiers: set[str] = set()
        for index, value in enumerate(raw_rules):
            if not isinstance(value, dict):
                raise RiskRulesError(f"rules[{index}] must be an object")
            identifier = value.get("id")
            if not isinstance(identifier, str) or not identifier.strip():
                raise RiskRulesError(f"rules[{index}].id must be a non-empty string")
            if identifier in identifiers:
                raise RiskRulesError(f"duplicate rule id: {identifier}")
            identifiers.add(identifier)
            try:
                risk = RiskLevel.parse(str(value.get("risk", "")))
            except ValueError as error:
                raise RiskRulesError(f"rules[{index}]: {error}") from error
            paths = _strings(value.get("paths"), f"rules[{index}].paths")
            expressions = _strings(
                value.get("content_patterns"), f"rules[{index}].content_patterns"
            )
            if not paths and not expressions:
                raise RiskRulesError(f"rules[{index}] must define paths or content_patterns")
            try:
                compiled = tuple(
                    re.compile(expression, re.IGNORECASE) for expression in expressions
                )
            except re.error as error:
                raise RiskRulesError(f"rules[{index}] has invalid regex: {error}") from error
            rules.append(
                RiskRule(
                    identifier=identifier,
                    risk=risk,
                    paths=paths,
                    content_patterns=compiled,
                    description=str(value.get("description", identifier)),
                )
            )
        return default, tuple(rules)

    @staticmethod
    def _path_matches(path: str, pattern: str) -> bool:
        normalized = path.removeprefix("./")
        return fnmatch.fnmatchcase(normalized, pattern)

    def classify(self, changes: tuple[ChangedFile, ...] | list[ChangedFile]) -> RiskAssessment:
        matches: list[RiskMatch] = []
        highest = self.default_risk
        for change in changes:
            for rule in self.rules:
                path_hit = any(
                    self._path_matches(path, pattern)
                    for path in change.affected_paths
                    for pattern in rule.paths
                )
                content_hit = any(pattern.search(change.patch) for pattern in rule.content_patterns)
                if not (path_hit or content_hit):
                    continue
                reason_parts: list[str] = []
                if path_hit:
                    reason_parts.append("path")
                if content_hit:
                    reason_parts.append("content")
                matches.append(
                    RiskMatch(
                        rule_id=rule.identifier,
                        risk=rule.risk,
                        path=change.path,
                        reason=f"{'+'.join(reason_parts)} matched: {rule.description}",
                    )
                )
                highest = max(highest, rule.risk)
        return RiskAssessment(
            risk=highest,
            matches=tuple(matches),
            changed_files=tuple(change.path for change in changes),
        )
