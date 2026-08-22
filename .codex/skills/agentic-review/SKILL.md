---
name: "agentic-review"
description: "Use for on-demand pull-request review requests such as `review PR #7` and for the active-session automatic review handoff that follows a Harness-created pull request."
metadata:
  short-description: "Fresh Automated Reviewer handoff for governed PR review"
---

# Agentic Review

Use this skill when either of these is true:

- The user asks for a governed review with phrases such as `review PR #7`, `review pull request 7`, or a repository-qualified equivalent.
- The Harness creates a pull request while the current Codex session is still active and emits the post-`pr_created` review handoff.

## Required behavior

1. Prepare one objective review request from live PR or repository state.
2. Start a fresh Codex agent using `.codex/agents/automated-reviewer.toml`.
3. Immediately record `review-agent-started` with the spawned agent thread/session identifier; it must differ from the requesting Software Team Member session.
4. Give that reviewer only trusted policy, immutable review metadata, diff or tracked-tree context, tests, CI, and prior review state when present.
5. Do not pass implementation conclusions, fix summaries, or "what changed" narratives from the implementer as reviewer instructions.
6. Require strict normalized output matching `.github/codex/schemas/provider-review-output-v1.json`, including exact repository/PR/base/head/request digest, `reviewer_role`, reviewer session, and `markdown_report`.
7. Record and publish review artifacts outside the product diff. Recording must reject output without the matching reviewer-agent start event.
8. If the PR head changes, record `review-fix-pushed`, mark the older review stale, and rerun a fresh Automated Reviewer cycle.

## Boundaries

- The Automated Reviewer is evidence-only and read-only.
- It never approves, merges, edits, deploys, or completes student learning steps.
- Public generated runtime distribution includes this skill and the reviewer agent without `OPENAI_API_KEY` or governance-token material.
- Missing, malformed, stale, or incomplete bundles must fail as `review_incomplete`, not as a silent pass.

## Output expectations

- Findings use stable `AR-NNN` identifiers.
- `markdown_report` is the full Markdown source for the human-readable review.
- The Markdown report covers metadata, result, severity-grouped findings, robotics safety, architecture, testing, governance, learning considerations, positives, and the final recommendation.
