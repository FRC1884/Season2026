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
6. Require strict normalized output matching `.github/robotics-harness/runtime/.github/codex/schemas/provider-review-output-v1.json`, including exact repository/PR/base/head/request digest, `reviewer_role`, reviewer session, and `markdown_report`.
7. Record review artifacts outside the product diff, publish the human-readable Markdown marker, and request `@codex review` with the exact-head platform request marker. Recording must reject output without the matching reviewer-agent start event.
8. Run `review-publish` with `--trusted-reviewer-login "chatgpt-codex-connector[bot]" --trusted-reviewer-id 199175422 --trusted-reviewer-type "Bot" --trusted-request-publisher-login "github-actions[bot]" --trusted-request-publisher-id 41898282 --trusted-request-publisher-type "Bot" --platform-wait-seconds 600`. It dispatches the trusted default-branch job that creates an immutable bot-authored exact-head request, waits for the configured Codex identity to publish findings, a request-comment `+1`, or an unedited clean-result comment carrying the matching reviewed-commit prefix, and only then dispatches deterministic validation. Repository-wide reactions are not review authority. A prior-head request receives a bounded wait before a distinct current-head request is created; expiry never treats the old request as current. Request-bound `eyes` is not an indefinite retry lock. Retry from the active session if the outer wait times out. The initiating dispatch sender is never reviewer authentication.
9. If the PR head changes, record `review-fix-pushed`, mark the older review stale, and rerun both a fresh Automated Reviewer cycle and the trusted platform review for the new head.

## Boundaries

- The Automated Reviewer is evidence-only and read-only.
- It never approves, merges, edits, deploys, or completes student learning steps.
- Public generated runtime distribution includes this skill and the reviewer agent without `OPENAI_API_KEY` or governance-token material.
- Missing, malformed, stale, or incomplete bundles must fail as `review_incomplete`, not as a silent pass.
- The local integrity hash and session lifecycle are readable orchestration evidence, not required-check authentication. Agent-context independence is enforced by the Harness, while the required check derives solely from the pinned Codex GitHub identity's exact-head review result.

## Output expectations

- Findings use stable `AR-NNN` identifiers.
- `markdown_report` is the full Markdown source for the human-readable review.
- The Markdown report covers metadata, result, severity-grouped findings, robotics safety, architecture, testing, governance, learning considerations, positives, and the final recommendation.
