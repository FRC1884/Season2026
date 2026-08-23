"""Provider-neutral AI-review interface and deterministic test providers."""

from __future__ import annotations

import json
from collections.abc import Callable, Iterable
from pathlib import Path
from typing import Any, Protocol

from harness.ai_review.models import ProviderReview, ReviewRequest


class ProviderError(RuntimeError):
    """A provider could not return a valid review within the configured limits."""


class ReviewProvider(Protocol):
    """A provider reviews bounded data and returns validated structured output."""

    @property
    def name(self) -> str: ...

    def review(self, request: ReviewRequest) -> ProviderReview: ...


class FakeReviewProvider:
    """Deterministic provider for offline tests; never calls a model or network."""

    name = "fake"

    def __init__(
        self,
        responses: Iterable[dict[str, Any] | ProviderReview]
        | Callable[[ReviewRequest], dict[str, Any] | ProviderReview],
    ) -> None:
        self.requests: list[ReviewRequest] = []
        self._callback = responses if callable(responses) else None
        self._responses = iter(()) if callable(responses) else iter(responses)

    def review(self, request: ReviewRequest) -> ProviderReview:
        self.requests.append(request)
        try:
            raw = self._callback(request) if self._callback is not None else next(self._responses)
        except StopIteration as error:
            raise ProviderError("fake provider has no response for this request") from error
        if isinstance(raw, ProviderReview):
            return raw
        try:
            return ProviderReview.from_dict(raw)
        except (TypeError, ValueError) as error:
            raise ProviderError(f"fake provider returned malformed output: {error}") from error


class RecordedReviewProvider:
    """Explicit, validated external review responses for a recorded manual demo."""

    name = "recorded-external-ai"

    def __init__(self, responses: Iterable[dict[str, Any] | ProviderReview]) -> None:
        try:
            self._responses = [
                item if isinstance(item, ProviderReview) else ProviderReview.from_dict(item)
                for item in responses
            ]
        except (TypeError, ValueError) as error:
            raise ProviderError(f"recorded AI review response is malformed: {error}") from error
        if not self._responses:
            raise ProviderError("recorded AI review response file contains no responses")
        self.requests: list[ReviewRequest] = []
        self._cursor = 0

    @classmethod
    def from_path(cls, path: Path | str) -> RecordedReviewProvider:
        source = Path(path).expanduser()
        if source.is_symlink() or not source.is_file():
            raise ProviderError("recorded AI review response must be a regular file")
        if source.stat().st_size > 2_000_000:
            raise ProviderError("recorded AI review response exceeds 2,000,000 bytes")
        try:
            value = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ProviderError(f"cannot read recorded AI review response: {error}") from error
        if isinstance(value, dict):
            value = value.get("responses")
        if not isinstance(value, list):
            raise ProviderError(
                "recorded AI review file must be a list or an object with a responses list"
            )
        return cls(value)

    def review(self, request: ReviewRequest) -> ProviderReview:
        self.requests.append(request)
        if self._cursor >= len(self._responses):
            raise ProviderError("recorded AI review has no response for the current request")
        response = self._responses[self._cursor]
        self._cursor += 1
        return response
