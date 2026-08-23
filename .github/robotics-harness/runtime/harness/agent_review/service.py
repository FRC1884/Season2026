"""Pure domain services for deterministic automated-review request/state handling."""

from __future__ import annotations

import json
import re
from dataclasses import replace
from pathlib import Path

from harness.agent_review.models import (
    STATE_MARKER_PREFIX,
    STATE_MARKER_SUFFIX,
    PullRequestContext,
    PullRequestRef,
    ReviewContext,
    ReviewCycle,
    ReviewFinding,
    ReviewRequest,
    ReviewResponse,
    ReviewState,
    decode_state_marker,
    encode_state_marker,
)
from harness.private_io import write_private_text

_REPOSITORY = r"(?P<repository>[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+)"
_REQUEST_PATTERNS = (
    re.compile(
        rf"^\s*review\s+{_REPOSITORY}\s+(?:pr|pull\s+request)\s+#?(?P<identifier>\S+)\s*$",
        re.IGNORECASE,
    ),
    re.compile(
        rf"^\s*review\s+(?:pr|pull\s+request)\s+#?(?P<identifier>\S+)\s+in\s+{_REPOSITORY}\s*$",
        re.IGNORECASE,
    ),
    re.compile(
        r"^\s*review\s+(?:pr|pull\s+request)\s+#?(?P<identifier>\S+)\s*$",
        re.IGNORECASE,
    ),
)
_SEVERITY_RANK = {"critical": 0, "high": 1, "medium": 2, "low": 3}


def normalize_review_state(value: ReviewState | str) -> ReviewState:
    if isinstance(value, ReviewState):
        return value
    normalized = str(value).strip().casefold()
    for state in ReviewState:
        if normalized == state.value:
            return state
    raise ValueError(f"unsupported review state: {value!r}")


def parse_review_request(text: str, *, known_repositories: tuple[str, ...] = ()) -> ReviewRequest:
    raw = str(text).strip()
    if not raw:
        raise ValueError("review request must mention a pull request")
    repository = ""
    identifier = ""
    for pattern in _REQUEST_PATTERNS:
        match = pattern.fullmatch(raw)
        if match is None:
            continue
        repository = str(match.groupdict().get("repository", "")).strip()
        identifier = str(match.group("identifier")).strip()
        break
    if not identifier:
        raise ValueError("review request must include a pull request identifier")
    if not identifier.isdigit():
        raise ValueError("invalid pull request identifier")
    if repository:
        if known_repositories and repository not in known_repositories:
            raise ValueError("review request selected an unknown repository")
    elif len(known_repositories) == 1:
        repository = known_repositories[0]
    elif len(known_repositories) > 1:
        raise ValueError("ambiguous repository selection")
    return ReviewRequest(
        raw_text=raw,
        pull_request=PullRequestRef(repository=repository, number=int(identifier)),
    )


def build_review_context(
    *,
    request: ReviewRequest,
    pull_request: PullRequestContext,
    requested_by_session_id: str,
    trusted_policy: str = "",
    ci_status: dict[str, str] | None = None,
    prior_review_state: dict[str, object] | None = None,
) -> ReviewContext:
    resolved_request = request.with_pull_request(
        PullRequestRef(
            repository=request.repository or pull_request.pull_request.repository,
            number=request.number,
            base_sha=request.pull_request.base_sha or pull_request.pull_request.base_sha,
            head_sha=request.pull_request.head_sha or pull_request.pull_request.head_sha,
        )
    )
    diff_context = "\n".join(
        (
            f"Repository: {pull_request.pull_request.repository}",
            f"Pull request: #{pull_request.pull_request.number}",
            f"Title: {pull_request.title}",
            f"Author: {pull_request.author}",
            f"Opened at: {pull_request.opened_at.astimezone().isoformat()}",
            "Changed files:",
            *[f"- {path}" for path in pull_request.files],
            "Unified diff:",
            pull_request.diff_text.strip(),
        )
    ).strip()
    return ReviewContext(
        request=resolved_request,
        pull_request=pull_request,
        requested_by_session_id=requested_by_session_id,
        objective=(
            f"Review pull request #{pull_request.pull_request.number} for correctness, "
            "safety, and governance."
        ),
        diff_context=diff_context,
        trusted_policy=trusted_policy,
        ci_status={} if ci_status is None else dict(ci_status),
        prior_review_state=({} if prior_review_state is None else dict(prior_review_state)),
    )


def create_review_cycle(
    *, context: ReviewContext, reviewer_role: str, reviewer_session_id: str
) -> ReviewCycle:
    if reviewer_role != "Automated Reviewer":
        raise ValueError("review cycles must be created by the Automated Reviewer role")
    if reviewer_session_id == context.requested_by_session_id:
        raise ValueError("the reviewer must run in a different session from implementation")
    return ReviewCycle(
        context=context,
        reviewer_role=reviewer_role,
        reviewer_session_id=reviewer_session_id,
        state=ReviewState.RUNNING,
    )


def _finding_sort_key(finding: ReviewFinding) -> tuple[object, ...]:
    return (
        _SEVERITY_RANK.get(finding.severity.casefold(), 99),
        finding.category.casefold(),
        finding.title.casefold(),
        finding.summary.casefold(),
        finding.recommendation.casefold(),
        finding.fingerprint,
    )


def _next_finding_id(used: set[int]) -> str:
    value = 1
    while value in used:
        value += 1
    used.add(value)
    return f"AR-{value:03d}"


def _assign_stable_finding_ids(
    findings: tuple[ReviewFinding, ...], *, previous: tuple[ReviewFinding, ...] = ()
) -> tuple[ReviewFinding, ...]:
    previous_ids = {
        finding.fingerprint: finding.finding_id for finding in previous if finding.finding_id
    }
    used_numbers = {
        int(finding_id.removeprefix("AR-"))
        for finding_id in previous_ids.values()
        if finding_id.startswith("AR-")
    }
    explicit = {
        finding.finding_id
        for finding in findings
        if finding.finding_id and finding.finding_id.startswith("AR-")
    }
    used_numbers.update(int(value.removeprefix("AR-")) for value in explicit)
    assigned: dict[str, str] = {}
    historical_ids = set(previous_ids.values())
    claimed_ids: set[str] = set()
    for finding in sorted(findings, key=_finding_sort_key):
        if finding.fingerprint in previous_ids:
            assigned[finding.fingerprint] = previous_ids[finding.fingerprint]
        elif (
            finding.finding_id
            and finding.finding_id not in historical_ids
            and finding.finding_id not in claimed_ids
        ):
            assigned[finding.fingerprint] = finding.finding_id
        else:
            assigned[finding.fingerprint] = _next_finding_id(used_numbers)
        claimed_ids.add(assigned[finding.fingerprint])
    return tuple(
        finding
        if finding.finding_id == assigned[finding.fingerprint]
        else finding.with_finding_id(assigned[finding.fingerprint])
        for finding in findings
    )


def _derive_review_state(findings: tuple[ReviewFinding, ...]) -> ReviewState:
    if any(finding.blocking for finding in findings):
        return ReviewState.CHANGES_REQUESTED
    if findings:
        return ReviewState.COMMENT
    return ReviewState.PASS


def _validate_state_matches_findings(cycle: ReviewCycle) -> None:
    if cycle.state is ReviewState.PASS and cycle.findings:
        raise ValueError("a passing review cannot contain findings")
    if cycle.state is ReviewState.COMMENT and any(finding.blocking for finding in cycle.findings):
        raise ValueError("comment review state cannot contain blocking findings")
    if cycle.state is ReviewState.CHANGES_REQUESTED and not any(
        finding.blocking for finding in cycle.findings
    ):
        raise ValueError("changes_requested requires at least one blocking finding")
    if cycle.state is ReviewState.RUNNING and cycle.findings:
        raise ValueError("running review state cannot contain finalized findings")


def render_review_markdown(cycle: ReviewCycle) -> str:
    validate_review_cycle(cycle, current_head_sha=cycle.head_sha)
    categories: dict[str, list[ReviewFinding]] = {
        "robotics safety": [],
        "architecture": [],
        "testing": [],
        "governance": [],
    }
    other: list[ReviewFinding] = []
    for finding in cycle.findings:
        category = finding.category.casefold()
        if "safety" in category:
            categories["robotics safety"].append(finding)
        elif "arch" in category:
            categories["architecture"].append(finding)
        elif "test" in category:
            categories["testing"].append(finding)
        elif "govern" in category:
            categories["governance"].append(finding)
        else:
            other.append(finding)
    lines = [
        f"# Agentic Review — PR #{cycle.pull_request}",
        encode_review_state_marker(cycle),
        "",
        "## Review Metadata",
        "",
        f"- Repository: `{cycle.repository}`",
        f"- Pull Request: `#{cycle.pull_request}`",
        f"- Base: `{cycle.base_sha}`",
        f"- Head: `{cycle.head_sha}`",
        f"- Commit: `{cycle.head_sha}`",
        f"- Reviewer Role: `{cycle.reviewer_role}`",
        f"- Review Session: `{cycle.reviewer_session_id}`",
        f"- Timestamp: `{cycle.recorded_at.isoformat()}`",
        f"- Request digest: `{cycle.context.request_digest_sha256}`",
        f"- Integrity hash: `{cycle.integrity_hash}`",
        "",
        "## Executive Result",
        "",
        f"- State: `{cycle.state.value}`",
        f"- Blocking findings: `{len(cycle.blocking_findings)}`",
        "",
        "## Summary",
        "",
        f"Reviewed {len(cycle.context.pull_request.files)} changed file(s) at the exact "
        f"pull-request head and recorded {len(cycle.findings)} finding(s).",
        "",
        "## Findings",
        "",
    ]
    if cycle.findings:
        for severity in ("critical", "high", "medium", "low"):
            lines.extend((f"### {severity.upper()}", ""))
            matches = [
                finding for finding in cycle.findings if finding.severity.casefold() == severity
            ]
            if not matches:
                lines.extend(("No findings.", ""))
                continue
            for finding in matches:
                location = finding.path or "Not line-specific"
                if finding.start_line is not None:
                    location += f":{finding.start_line}"
                    if finding.end_line and finding.end_line != finding.start_line:
                        location += f"-{finding.end_line}"
                lines.extend(
                    (
                        f"#### {finding.finding_id} — {finding.title}",
                        "",
                        f"**File:** `{location}`  ",
                        f"**Category:** `{finding.category}`  ",
                        f"**Blocking:** `{'YES' if finding.blocking else 'NO'}`",
                        "",
                        "**Issue**",
                        "",
                        finding.summary,
                        "",
                        "**Why this matters**",
                        "",
                        finding.summary,
                        "",
                        "**Recommended action**",
                        "",
                        finding.recommendation,
                        "",
                        "---",
                        "",
                    )
                )
    else:
        lines.extend(("No findings were recorded for this review cycle.", ""))
    lines.extend(
        (
            "## Robot Safety Review",
            "",
            *(
                [
                    f"- {finding.finding_id}: {finding.summary}"
                    for finding in categories["robotics safety"]
                ]
                or ["- No robotics safety findings."]
            ),
            "",
            "## Architecture Review",
            "",
            *(
                [
                    f"- {finding.finding_id}: {finding.summary}"
                    for finding in categories["architecture"]
                ]
                or ["- No architecture findings."]
            ),
            "",
            "## Testing Review",
            "",
            *(
                [f"- {finding.finding_id}: {finding.summary}" for finding in categories["testing"]]
                or ["- No testing findings."]
            ),
            "",
            "## Governance Review",
            "",
            *(
                [
                    f"- {finding.finding_id}: {finding.summary}"
                    for finding in categories["governance"]
                ]
                or ["- No governance findings."]
            ),
            "",
            "## Learning / Understanding Considerations",
            "",
            "- Software Team Member responses remain part of the governed learning loop.",
            "",
            "## Positive Observations",
            "",
            "- Review state is bound to the exact repository, pull request, head SHA, "
            "request digest, and reviewer session.",
            *(
                [
                    f"- Additional finding context recorded: {finding.finding_id}"
                    for finding in other
                ]
                or []
            ),
            "",
            "## Final Recommendation",
            "",
            _final_recommendation(cycle),
            "",
        )
    )
    return "\n".join(lines)


def _final_recommendation(cycle: ReviewCycle) -> str:
    if cycle.state is ReviewState.PASS:
        return "Pass: no blocking findings were recorded for the reviewed head."
    if cycle.state is ReviewState.COMMENT:
        return "Comment: address the non-blocking findings before merge readiness."
    if cycle.state is ReviewState.CHANGES_REQUESTED:
        return "Changes requested: resolve the blocking findings and rerun review on the next head."
    if cycle.state is ReviewState.STALE:
        return "Stale: do not rely on this state after the pull request head changed."
    if cycle.state is ReviewState.FAILED:
        return "Failed: the review state is not valid for governance use."
    return "Review is still running and cannot be treated as final."


def encode_review_state_marker(cycle: ReviewCycle) -> str:
    return encode_state_marker(cycle)


def decode_review_state_marker(text: str) -> ReviewCycle | None:
    return decode_state_marker(text)


def validate_review_cycle(cycle: ReviewCycle, *, current_head_sha: str) -> ReviewCycle:
    if cycle.integrity_hash != cycle.compute_integrity_hash():
        raise ValueError("review-cycle integrity hash does not match the normalized payload")
    if cycle.reviewer_role != "Automated Reviewer":
        raise ValueError("review cycle must be authored by the Automated Reviewer role")
    if cycle.reviewer_session_id == cycle.context.requested_by_session_id:
        raise ValueError("reviewer must use a different session from implementation")
    if cycle.head_sha != current_head_sha:
        raise ValueError("review cycle is bound to a different pull request head")
    _validate_state_matches_findings(cycle)
    finding_ids = {finding.finding_id for finding in cycle.findings}
    if len(finding_ids) != len(cycle.findings):
        raise ValueError("review findings must use distinct stable AR-NNN identifiers")
    for response in cycle.responses:
        if response.responder_role != "Software Team Member":
            raise ValueError("review responses must be authored by a Software Team Member")
        if response.finding_id not in finding_ids:
            raise ValueError("review response references an unknown finding")
        if response.decision == "reject":
            if not response.rationale.strip():
                raise ValueError("rejecting a finding requires rationale")
            if not response.evidence:
                raise ValueError("rejecting a finding requires evidence")
    return cycle


def mark_review_stale(cycle: ReviewCycle, *, new_head_sha: str) -> ReviewCycle:
    if new_head_sha == cycle.head_sha:
        return cycle
    return cycle.with_state(ReviewState.STALE, stale_head_sha=new_head_sha)


def record_review_response(cycle: ReviewCycle, response: ReviewResponse) -> ReviewCycle:
    finding_ids = {finding.finding_id for finding in cycle.findings}
    if response.finding_id not in finding_ids:
        raise ValueError("review response references an unknown finding")
    if response.responder_role != "Software Team Member":
        raise ValueError("review responses must be authored by a Software Team Member")
    if response.decision == "reject" and not response.rationale.strip():
        raise ValueError("rejecting a finding requires rationale")
    if response.decision == "reject" and not response.evidence:
        raise ValueError("rejecting a finding requires evidence")
    replaced = [item for item in cycle.responses if item.finding_id != response.finding_id]
    replaced.append(response)
    replaced.sort(key=lambda item: item.finding_id)
    return cycle.with_responses(tuple(replaced))


def build_monitoring_payload(
    *, event_type: str, cycle: ReviewCycle, task_identifier: str
) -> dict[str, object]:
    validate_review_cycle(cycle, current_head_sha=cycle.head_sha)
    return {
        "event_type": str(event_type),
        "task_identifier": str(task_identifier),
        "repository": cycle.repository,
        "pull_request": cycle.pull_request,
        "base_sha": cycle.base_sha,
        "head_sha": cycle.head_sha,
        "review_state": cycle.state.value,
        "request_digest_sha256": cycle.context.request_digest_sha256,
        "integrity_hash": cycle.integrity_hash,
        "requested_by_session_id": cycle.context.requested_by_session_id,
        "reviewer_role": cycle.reviewer_role,
        "reviewer_session_id": cycle.reviewer_session_id,
        "finding_ids": [finding.finding_id for finding in cycle.findings],
        "response_count": len(cycle.responses),
    }


def create_auto_review_request(
    *,
    repository: str,
    pull_request_number: int,
    head_sha: str,
    requested_by_session_id: str,
) -> ReviewRequest:
    return ReviewRequest(
        raw_text=f"auto review {repository} PR #{pull_request_number}",
        pull_request=PullRequestRef(
            repository=repository,
            number=pull_request_number,
            head_sha=head_sha,
        ),
        requested_by_session_id=requested_by_session_id,
        automatic=True,
    )


def create_review_cycle_with_findings(
    *,
    context: ReviewContext,
    reviewer_role: str,
    reviewer_session_id: str,
    findings: tuple[ReviewFinding, ...],
    previous_cycle: ReviewCycle | None = None,
) -> ReviewCycle:
    cycle = create_review_cycle(
        context=context,
        reviewer_role=reviewer_role,
        reviewer_session_id=reviewer_session_id,
    )
    assigned = _assign_stable_finding_ids(
        findings,
        previous=() if previous_cycle is None else previous_cycle.findings,
    )
    return cycle.with_findings(assigned, overall_state=_derive_review_state(assigned))


def finalize_review_cycle(
    cycle: ReviewCycle,
    *,
    findings: tuple[ReviewFinding, ...],
    overall_state: ReviewState | str | None = None,
    previous_cycle: ReviewCycle | None = None,
) -> ReviewCycle:
    assigned = _assign_stable_finding_ids(
        tuple(findings),
        previous=() if previous_cycle is None else previous_cycle.findings,
    )
    state = (
        _derive_review_state(assigned)
        if overall_state is None
        else normalize_review_state(overall_state)
    )
    return cycle.with_findings(assigned, overall_state=state)


def require_external_evidence_root(
    *, target_repository: Path | str, evidence_root: Path | str
) -> tuple[Path, Path]:
    repository = Path(target_repository).expanduser().resolve()
    root = Path(evidence_root).expanduser().resolve()
    if root == repository or root.is_relative_to(repository):
        raise ValueError("evidence_root must be outside the target repository")
    return repository, root


def write_review_cycle_artifacts(
    *, cycle: ReviewCycle, target_repository: Path | str, evidence_root: Path | str
) -> tuple[Path, Path]:
    _, root = require_external_evidence_root(
        target_repository=target_repository,
        evidence_root=evidence_root,
    )
    stem = f"pr-{cycle.pull_request}-{cycle.head_sha}"
    json_path = write_private_text(
        root / f"{stem}.review.json",
        json.dumps(cycle.to_dict(), indent=2, sort_keys=True) + "\n",
    )
    markdown_path = write_private_text(root / f"{stem}.review.md", render_review_markdown(cycle))
    return json_path, markdown_path


def normalize_review_cycle(cycle: ReviewCycle) -> ReviewCycle:
    assigned = _assign_stable_finding_ids(cycle.findings, previous=cycle.findings)
    state = cycle.state
    if state in {ReviewState.RUNNING, ReviewState.NOT_STARTED}:
        state = _derive_review_state(assigned) if assigned else state
    return replace(cycle, findings=assigned, state=state, integrity_hash="")


__all__ = [
    "STATE_MARKER_PREFIX",
    "STATE_MARKER_SUFFIX",
    "build_monitoring_payload",
    "build_review_context",
    "create_auto_review_request",
    "create_review_cycle",
    "create_review_cycle_with_findings",
    "decode_review_state_marker",
    "encode_review_state_marker",
    "finalize_review_cycle",
    "mark_review_stale",
    "normalize_review_cycle",
    "normalize_review_state",
    "parse_review_request",
    "record_review_response",
    "render_review_markdown",
    "require_external_evidence_root",
    "validate_review_cycle",
    "write_review_cycle_artifacts",
]
