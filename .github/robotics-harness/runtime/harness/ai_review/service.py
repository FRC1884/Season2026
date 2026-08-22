"""Orchestration for complete-diff review, synthesis, and re-review tracking."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import UTC, datetime

from harness.ai_review.diff import DiffBundle
from harness.ai_review.models import (
    AIReview,
    Confidence,
    Finding,
    FindingState,
    ReviewCategory,
    ReviewContext,
    ReviewRequest,
    ReviewResult,
    Severity,
)
from harness.ai_review.prompt import possible_prompt_injection
from harness.ai_review.provider import ProviderError, ReviewProvider


@dataclass(frozen=True, slots=True)
class PullRequestMetadata:
    number: int
    repository: str
    title: str
    body: str


class AIReviewService:
    """Review every changed-file chunk, then synthesize one immutable-head result."""

    def __init__(self, provider: ReviewProvider, *, chunks_per_request: int = 8) -> None:
        if chunks_per_request < 1:
            raise ValueError("chunks_per_request must be positive")
        self.provider = provider
        self.chunks_per_request = chunks_per_request

    @staticmethod
    def is_stale(review: AIReview, current_head_sha: str) -> bool:
        return review.head_sha != current_head_sha

    @staticmethod
    def _request(
        metadata: PullRequestMetadata,
        diff: DiffBundle,
        context: ReviewContext,
        *,
        chunk_ids: tuple[str, ...],
        contents: tuple[str, ...],
        prior_findings: tuple[Finding, ...] = (),
        synthesis: bool = False,
    ) -> ReviewRequest:
        return ReviewRequest(
            pull_request=metadata.number,
            repository=metadata.repository,
            base_sha=diff.base_sha,
            head_sha=diff.head_sha,
            title=metadata.title,
            body=metadata.body,
            commit_messages=diff.commit_messages,
            context=context,
            chunk_ids=chunk_ids,
            untrusted_diff_chunks=contents,
            prior_findings=prior_findings,
            synthesis=synthesis,
        )

    @staticmethod
    def _injection_findings(diff: DiffBundle) -> tuple[Finding, ...]:
        matches = [chunk for chunk in diff.chunks if possible_prompt_injection(chunk.content)]
        if not matches:
            return ()
        first = matches[0]
        return (
            Finding(
                severity=Severity.LOW,
                confidence=Confidence.HIGH,
                category=ReviewCategory.SECURITY,
                path=first.path,
                start_line=None,
                end_line=None,
                title="Possible prompt-injection text in pull-request content",
                explanation=(
                    "Changed content appears to instruct an AI reviewer. It was treated as "
                    "untrusted data and did not alter review rules."
                ),
                evidence=(
                    f"Detected in {len(matches)} complete-diff chunk(s); first chunk "
                    f"{first.chunk_id}."
                ),
                impact=(
                    "There is no reviewer-control impact in this run, but the text may be "
                    "misleading or represent an attempted governance bypass."
                ),
                recommendation=(
                    "Verify that the text is required application data; remove it if it was "
                    "intended to control automated review."
                ),
                blocking=False,
            ),
        )

    @staticmethod
    def _deduplicate(findings: tuple[Finding, ...]) -> tuple[Finding, ...]:
        severity_order = {
            Severity.CRITICAL: 0,
            Severity.HIGH: 1,
            Severity.MEDIUM: 2,
            Severity.LOW: 3,
        }
        by_fingerprint: dict[str, Finding] = {}
        for finding in findings:
            by_fingerprint[finding.fingerprint] = finding
        return tuple(
            sorted(
                by_fingerprint.values(),
                key=lambda item: (
                    severity_order[item.severity],
                    item.path,
                    item.start_line or 0,
                    item.title,
                ),
            )
        )

    @staticmethod
    def _result_for(findings: tuple[Finding, ...]) -> ReviewResult:
        if any(
            finding.category is ReviewCategory.ROBOTICS_SAFETY and finding.blocking
            for finding in findings
        ):
            return ReviewResult.ESCALATE_TO_SAFETY_REVIEWER
        if any(finding.blocking for finding in findings):
            return ReviewResult.CHANGES_REQUESTED
        if findings:
            return ReviewResult.PASS_WITH_SUGGESTIONS
        return ReviewResult.PASS

    @staticmethod
    def _track_findings(
        findings: tuple[Finding, ...], previous: AIReview | None
    ) -> tuple[tuple[Finding, ...], tuple[Finding, ...]]:
        if previous is None:
            return tuple(finding.with_state(FindingState.NEW) for finding in findings), ()
        previous_by_identity = {finding.identity: finding for finding in previous.findings}
        current: list[Finding] = []
        for finding in findings:
            prior = previous_by_identity.get(finding.identity)
            if prior is None:
                state = FindingState.NEW
            elif prior.fingerprint == finding.fingerprint:
                state = FindingState.STILL_PRESENT
            else:
                state = FindingState.CHANGED
            current.append(finding.with_state(state))
        current_identities = {finding.identity for finding in findings}
        resolved = tuple(
            finding.with_state(FindingState.RESOLVED)
            for finding in previous.findings
            if finding.identity not in current_identities
        )
        return tuple(current), resolved

    def _incomplete(
        self,
        metadata: PullRequestMetadata,
        diff: DiffBundle,
        context: ReviewContext,
        reason: str,
        *,
        previous: AIReview | None,
    ) -> AIReview:
        return AIReview(
            pull_request=metadata.number,
            repository=metadata.repository,
            base_sha=diff.base_sha,
            head_sha=diff.head_sha,
            reviewed_at=datetime.now(UTC),
            result=ReviewResult.REVIEW_INCOMPLETE,
            summary=f"AI review incomplete: {reason}",
            risk_level=context.risk_classification,
            findings=(),
            resolved_findings=(),
            test_evidence=context.test_evidence,
            learning_evidence={"status": context.learning_review_status},
            required_reviewer_group="Harness Administrators",
            diff_complete=False,
            analysed_file_paths=(),
            all_file_paths=diff.changed_paths,
            review_provider=self.provider.name,
            previous_head_sha="" if previous is None else previous.head_sha,
        )

    def review(
        self,
        metadata: PullRequestMetadata,
        diff: DiffBundle,
        context: ReviewContext,
        *,
        previous: AIReview | None = None,
        required_reviewer_group: str = "General Code Owners",
    ) -> AIReview:
        """Run bounded reviews plus cross-file synthesis; fail closed on any missing chunk."""

        if not diff.complete or set(diff.analysed_paths) != set(diff.changed_paths):
            return self._incomplete(
                metadata,
                diff,
                context,
                diff.incomplete_reason or "full diff unavailable",
                previous=previous,
            )
        if previous is not None and (
            previous.pull_request != metadata.number or previous.repository != metadata.repository
        ):
            raise ValueError("previous review belongs to a different pull request")

        if not diff.chunks:
            tracked, resolved = self._track_findings((), previous)
            return AIReview(
                pull_request=metadata.number,
                repository=metadata.repository,
                base_sha=diff.base_sha,
                head_sha=diff.head_sha,
                reviewed_at=datetime.now(UTC),
                result=ReviewResult.PASS,
                summary="No file changes were present in the complete base-to-head diff.",
                risk_level=context.risk_classification,
                findings=tracked,
                resolved_findings=resolved,
                test_evidence=context.test_evidence,
                learning_evidence={"status": context.learning_review_status},
                required_reviewer_group=required_reviewer_group,
                diff_complete=True,
                analysed_file_paths=diff.changed_paths,
                all_file_paths=diff.changed_paths,
                review_provider=self.provider.name,
                previous_head_sha="" if previous is None else previous.head_sha,
            )

        partial_findings: list[Finding] = []
        reviewed_ids: set[str] = set()
        try:
            for offset in range(0, len(diff.chunks), self.chunks_per_request):
                group = diff.chunks[offset : offset + self.chunks_per_request]
                request = self._request(
                    metadata,
                    diff,
                    context,
                    chunk_ids=tuple(chunk.chunk_id for chunk in group),
                    contents=tuple(chunk.content for chunk in group),
                )
                response = self.provider.review(request)
                if response.result is ReviewResult.REVIEW_INCOMPLETE:
                    raise ProviderError(f"provider reported incomplete review: {response.summary}")
                expected = set(request.chunk_ids)
                actual = set(response.analysed_chunk_ids)
                if actual != expected:
                    raise ProviderError("provider did not attest to every supplied full-diff chunk")
                reviewed_ids.update(actual)
                partial_findings.extend(response.findings)

            all_chunk_ids = tuple(chunk.chunk_id for chunk in diff.chunks)
            synthesis_request = self._request(
                metadata,
                diff,
                context,
                chunk_ids=all_chunk_ids,
                contents=tuple(
                    (
                        f"Chunk {chunk.chunk_id} manifest: path={chunk.path}; "
                        f"sha256={chunk.content_sha256}; generated={chunk.generated}; "
                        f"binary={chunk.binary_metadata_only}; submodule={chunk.submodule}"
                    )
                    for chunk in diff.chunks
                ),
                prior_findings=tuple(partial_findings),
                synthesis=True,
            )
            synthesis = self.provider.review(synthesis_request)
            if synthesis.result is ReviewResult.REVIEW_INCOMPLETE:
                raise ProviderError(f"provider reported incomplete synthesis: {synthesis.summary}")
            if set(synthesis.analysed_chunk_ids) != set(all_chunk_ids):
                raise ProviderError("provider synthesis did not account for every diff chunk")
            reviewed_ids.update(synthesis.analysed_chunk_ids)
        except ProviderError as error:
            return self._incomplete(metadata, diff, context, str(error), previous=previous)

        if reviewed_ids != {chunk.chunk_id for chunk in diff.chunks}:
            return self._incomplete(
                metadata, diff, context, "not every diff chunk was analysed", previous=previous
            )

        synthesis_identities = {finding.identity for finding in synthesis.findings}
        missing_blocking = [
            finding
            for finding in partial_findings
            if finding.blocking and finding.identity not in synthesis_identities
        ]
        if missing_blocking:
            return self._incomplete(
                metadata,
                diff,
                context,
                "synthesis omitted a blocking per-chunk finding without resolution evidence",
                previous=previous,
            )
        findings = self._deduplicate(synthesis.findings + self._injection_findings(diff))
        valid_paths = set(diff.changed_paths)
        if any(finding.path not in valid_paths for finding in findings):
            return self._incomplete(
                metadata,
                diff,
                context,
                "provider finding referenced a file outside the pull-request diff",
                previous=previous,
            )
        result = self._result_for(findings)
        tracked, resolved = self._track_findings(findings, previous)
        summary = synthesis.summary
        if result is not synthesis.result:
            summary = (
                f"{summary} The harness normalized the overall result to "
                f"{result.value} from validated finding severity."
            )
        risk = synthesis.risk_level
        if any(finding.severity is Severity.CRITICAL for finding in findings):
            risk = "critical"
        elif any(finding.severity is Severity.HIGH for finding in findings):
            risk = "high"
        reviewer_group = required_reviewer_group
        if result is ReviewResult.ESCALATE_TO_SAFETY_REVIEWER:
            reviewer_group = "Safety Code Owners"
        elif any(
            finding.severity is Severity.CRITICAL
            and finding.category in {ReviewCategory.SECURITY, ReviewCategory.GOVERNANCE}
            for finding in findings
        ):
            reviewer_group = "Harness Administrators"
        return AIReview(
            pull_request=metadata.number,
            repository=metadata.repository,
            base_sha=diff.base_sha,
            head_sha=diff.head_sha,
            reviewed_at=datetime.now(UTC),
            result=result,
            summary=summary,
            risk_level=risk,
            findings=tracked,
            resolved_findings=resolved,
            test_evidence=context.test_evidence,
            learning_evidence={"status": context.learning_review_status},
            required_reviewer_group=reviewer_group,
            diff_complete=True,
            analysed_file_paths=diff.changed_paths,
            all_file_paths=diff.changed_paths,
            review_provider=self.provider.name,
            previous_head_sha="" if previous is None else previous.head_sha,
        )
