"""Human-readable AI-review summaries and private artifact persistence."""

from __future__ import annotations

import base64
import json
from pathlib import Path

from harness.ai_review.diff import DiffBundle
from harness.ai_review.models import AIReview, Finding, FindingState
from harness.private_io import write_private_text

SUMMARY_MARKER = "<!-- robotics-harness-ai-full-diff-review -->"
STATE_PREFIX = "<!-- robotics-harness-ai-review-state:"
STATE_SUFFIX = " -->"


def encode_review_state(review: AIReview) -> str:
    # The comment needs only immutable identity and current findings for re-review.
    # Large test evidence and path manifests stay in the Actions artifact.
    compact = {
        **review.to_dict(),
        "summary": "Prior immutable-head AI review state.",
        "resolved_findings": [],
        "test_evidence": {},
        "learning_evidence": {},
        "diff_complete": False,
        "analysed_file_paths": [],
        "all_file_paths": [],
    }
    raw = json.dumps(compact, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    encoded = base64.urlsafe_b64encode(raw).decode().rstrip("=")
    return f"{STATE_PREFIX}{encoded}{STATE_SUFFIX}"


def decode_review_state(comment: str) -> AIReview | None:
    start = comment.find(STATE_PREFIX)
    if start < 0:
        return None
    start += len(STATE_PREFIX)
    end = comment.find(STATE_SUFFIX, start)
    if end < 0:
        return None
    encoded = comment[start:end]
    try:
        padding = "=" * (-len(encoded) % 4)
        value = json.loads(base64.urlsafe_b64decode(encoded + padding))
        return AIReview.from_dict(value)
    except (ValueError, TypeError, json.JSONDecodeError):
        return None


def _finding_markdown(finding: Finding, index: int) -> list[str]:
    location = finding.path.replace("`", "\\`")
    if finding.start_line is not None:
        location += f":{finding.start_line}"
        if finding.end_line not in {None, finding.start_line}:
            location += f"-{finding.end_line}"
    confidence = finding.confidence.value
    state = finding.state.value.replace("_", " ")
    return [
        f"{index}. **[{finding.severity.value.title()}] {safe_markdown_text(finding.title)}**",
        f"   - Location: `{location}`",
        f"   - Confidence: {confidence}; status: {state}",
        f"   - Explanation: {safe_markdown_text(finding.explanation)}",
        f"   - Evidence: {safe_markdown_text(finding.evidence)}",
        f"   - Impact: {safe_markdown_text(finding.impact)}",
        f"   - Required action: {safe_markdown_text(finding.recommendation)}",
    ]


def safe_markdown_text(value: str) -> str:
    """Prevent provider output from injecting HTML or notification mentions."""

    return (
        value.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace("@", "@\u200b")
    )


def render_summary_comment(review: AIReview, diff: DiffBundle) -> str:
    """Render the one stable, update-in-place pull-request summary comment."""

    blocking = [finding for finding in review.findings if finding.blocking]
    non_blocking = [finding for finding in review.findings if not finding.blocking]
    lines = [
        SUMMARY_MARKER,
        encode_review_state(review),
        "## Codex PR Review",
        "",
        f"**Result:** `{review.result.value}`  ",
        f"**Risk:** `{review.risk_level}`  ",
        f"**Reviewed:** `{review.base_sha}...{review.head_sha}`  ",
        (
            f"**Files:** {len(diff.files)} changed "
            f"(`+{diff.additions}` / `-{diff.deletions}`; "
            f"{diff.full_diff_bytes} diff bytes)"
        ),
        "",
    ]
    if blocking:
        lines.extend(("### Blocking Findings", ""))
        for index, finding in enumerate(blocking, start=1):
            lines.extend(_finding_markdown(finding, index))
        lines.append("")
    if non_blocking:
        lines.extend(("### Non-Blocking Findings", ""))
        for index, finding in enumerate(non_blocking, start=1):
            lines.extend(_finding_markdown(finding, index))
        lines.append("")
    if review.resolved_findings:
        lines.extend(("### Resolved Since the Previous Review", ""))
        for index, finding in enumerate(review.resolved_findings, start=1):
            resolved = finding.with_state(FindingState.RESOLVED)
            lines.extend(_finding_markdown(resolved, index))
        lines.append("")
    lines.extend(
        (
            "### Validation Evidence",
            "",
            f"- Test evidence: {json.dumps(review.test_evidence, sort_keys=True)}",
            f"- Learning evidence: {json.dumps(review.learning_evidence, sort_keys=True)}",
            f"- Complete diff analysed: {'yes' if review.diff_complete else 'no'}",
            f"- Required reviewer group: {review.required_reviewer_group}",
            "",
            "### Human Review Required",
            "",
            "**Codex review is evidence, not approval.** This check cannot approve, merge, "
            "or complete the student learning review. The human reviewer or reviewers "
            "eligible under current repository policy must inspect the actual diff and evidence.",
        )
    )
    rendered = "\n".join(lines) + "\n"
    if len(rendered.encode("utf-8")) > 60_000:
        raise ValueError("AI review summary exceeds GitHub's safe comment size")
    return rendered


def render_artifact_report(review: AIReview, diff: DiffBundle) -> str:
    return (
        "# Codex PR Review Artifact\n\n"
        f"- Pull request: {review.pull_request}\n"
        f"- Repository: `{review.repository}`\n"
        f"- Base: `{review.base_sha}`\n"
        f"- Head: `{review.head_sha}`\n"
        f"- Full diff SHA-256: `{diff.full_diff_sha256}`\n"
        f"- Provider: `{review.review_provider}`\n\n"
        + render_summary_comment(review, diff).replace(SUMMARY_MARKER + "\n", "", 1)
    )


def write_review_artifacts(
    directory: Path | str, review: AIReview, diff: DiffBundle
) -> tuple[Path, Path]:
    """Write owner-only JSON and Markdown artifacts keyed by PR and immutable head."""

    root = Path(directory).expanduser().resolve()
    stem = f"pr-{review.pull_request}-{review.head_sha}"
    json_path = write_private_text(
        root / f"{stem}.json",
        json.dumps(
            {
                **review.to_dict(),
                "diff": diff.to_dict(),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
    )
    markdown_path = write_private_text(root / f"{stem}.md", render_artifact_report(review, diff))
    return json_path, markdown_path
