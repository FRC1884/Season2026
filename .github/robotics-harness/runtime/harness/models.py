"""Shared typed models used by the harness."""

from __future__ import annotations

from enum import IntEnum


class RiskLevel(IntEnum):
    """Ordered risk classification used throughout the harness."""

    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4

    @classmethod
    def parse(cls, value: str) -> RiskLevel:
        try:
            return cls[value.strip().upper()]
        except KeyError as error:
            choices = ", ".join(level.label for level in cls)
            raise ValueError(f"unknown risk level {value!r}; expected one of: {choices}") from error

    @property
    def label(self) -> str:
        return self.name.lower()
