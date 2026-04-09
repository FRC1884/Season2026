# Original GriffinSim vs Rebuilt System

## Important Boundary

This repository is not the original GriffinSim engine repository. It is the 2026 robot-code
consumer that integrates GriffinSim and MapleSim through external dependencies.

## Original State in This Repo

- In-process WPILib simulation loop
- Maple/GriffinSim adapter wiring in `RobotContainer`
- AdvantageScope visualization and shot-review outputs
- No local authoritative frame schema
- No deterministic replay proof
- No repo-local audit/migration artifacts

## Rebuilt Additions in This Repo

- Audit report, gap analysis, and migration plan
- Versioned deterministic frame contracts
- Stable binary codec for frame serialization and comparison
- Manual time source abstraction for fixed-step replay
- Replay trace hashing, diffing, and verification
- Deterministic replay test coverage and a Gradle entry point

## What Is Still Outside This Repo

- The actual GriffinSim engine implementation
- The native HALSIM extension source
- A separate physics-engine process
- External IPC transport between robot host and engine process

## Practical Interpretation

The rebuilt system in `season2026` is now a deterministic consumer-side execution layer and proof
surface. It is the correct place to verify contracts, time control, and replay behavior for this
robot program. Full engine replacement still requires the GriffinSim source repository or a
monorepo migration that brings that code into this workspace.
