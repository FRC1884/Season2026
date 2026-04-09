# GriffinSim Migration Plan

## Goal

Move this robot-consumer repository from ad hoc in-process simulation helpers toward a deterministic
simulation execution layer that is explicit about frames, time, and replay.

## Phase 0: Audit and Contract Baseline

- Publish the audit report, gap analysis, and migration plan.
- Add versioned sim I/O contracts and a stable binary codec.
- Introduce a manual simulation time source so replay logic can run without wall-clock coupling.

Risk mitigation:
- Keep all existing runtime wiring intact.
- Add deterministic surfaces in parallel packages rather than rewriting live robot code first.

## Phase 1: Deterministic Replay Foundation

- Refactor shot/replay utilities to support injected time sources.
- Add deterministic replay traces, hashing, and diffing.
- Add a Gradle-verifiable replay command and unit tests that prove repeated runs are identical.

Risk mitigation:
- Preserve the current AdvantageScope logging behavior by keeping wall-clock defaults in
  production-facing constructors.

## Phase 2: Local Control-Host Structure

- Add a lockstep-friendly execution layer that can step local simulation surfaces with explicit
  `stepId` and `simTimeNanos`.
- Route visualization to snapshots instead of direct mutable state where practical.
- Keep the current Maple/GriffinSim adapters behind a compatibility boundary.

Risk mitigation:
- Do not rewrite `RobotContainer` or subsystem IO ownership until deterministic contracts are
  already verified in tests.

## Phase 3: Engine/Bridge Boundary Replacement

- Replace placeholder bridge code.
- Add a real transport boundary around the new frame contracts.
- If the GriffinSim engine source is pulled into this workspace later, implement native bridge
  loading and queue discipline against the new contracts instead of ad hoc adapters.

Risk mitigation:
- Refuse to depend on placeholder or undocumented bridge behavior once the authoritative bridge
  path exists.

## Phase 4: Full Lockstep Integration

- Add a real control-host mode that can drive robot simulation in fixed steps.
- Capture deterministic traces for end-to-end scenarios.
- Promote replay verification to a required check before claiming completion.

Risk mitigation:
- Keep interactive real-time sim available as a separate mode for driver workflow and debugging.

## Stop Conditions

Stop and fix immediately if any of the following break:

- repeated deterministic runs produce different hashes
- visualization mutates authoritative state
- wall-clock time leaks back into deterministic replay surfaces
- external dependencies force behavior that cannot be reproduced locally
