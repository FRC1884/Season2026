# AGENTS.md — Managed Robotics Repository

This repository is governed by the central harness distribution for
`FRC1884/Season2026`.

- Harness source: `FRC1884/robotics-agentic-development-harness@843fc3fba16ec56c4a666fa4501d1f80e4583d46`
- Harness policy version: `frc1884-bootstrap-v1`
- Protected default branch: `main`

Software Team Members must use the repository lifecycle exposed by the Harness.
Human approval remains explicit. The automated reviewer is evidence, not merge
authority.

## Required controls

- Read the repository and task scope before editing.
- Confirm the current plan before implementation.
- Keep work on a task branch.
- Do not bypass protected paths, Code Owners, CI, or mentor approval.
- Do not deploy robot code from automation.

## Required pull-request checks

- `Phase 0 Validation`
- `Agentic Review`

## Human reviewer groups

- Mentor / Code Owner: `@FRC1884/mentors`
- Programming lead: `@FRC1884/software-lead`
- Approved alumni: `@FRC1884/approved-alumni-reviewers`

## Baseline validation commands

```text
Build: ./gradlew build --no-daemon --console=plain
Test:  ./gradlew test --no-daemon --console=plain
```

External GitHub branch protection, team membership, and required-check
activation must be adopted separately. Repository files alone do not activate
those hosted controls.
