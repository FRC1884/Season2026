# FRC 1884 • 2026 REBUILT Robot Code

> The official software stack for Team 1884 Griffins’ 2026 build season (REBUILT). Everything that hits the field, from drivetrain controls to the operator dashboard, lives here.

## 2026 Season Highlights

- **Operator Board UI** - Lightweight tablet web app served by the robot for state requests and at-a-glance telemetry.
- **Drive + Turn SysId** - Separate drivetrain and steer characterization routines so pit-side retuning does not require code edits.
- **State-first subsystems** - AdvantageKit logging, IO abstraction, and command-based verbs are used throughout the robot stack.
- **Simulation runtime** - The active SIM path now uses repo-local deterministic drivetrain physics, terrain response, replay logs, and a socket-backed authoritative engine.

## Start Here

- Main architecture map: [Architecture][arch]
- Match/practice validation checklist: [Checklist][checklist]
- Swerve calibration and odometry flow: [Swerve][swerve-doc]
- Vision pose acceptance and consumers: [Vision][vision-doc]
- Persistence and deploy-preserve behavior: [Persistence][persistence-doc]
- Simulation design docs: [Sim Arch][sim-arch], [Sim Review][sim-review], [CAD Pipeline][sim-cad]

## SIM Architecture And Invariants

- There is exactly one authoritative world state: the local deterministic physics world in active SIM.
- No secondary or parallel truth sources are allowed for robot, terrain, field, or gamepiece state.
- Actuator outputs flow only through staged SIM IO into the authoritative world step.
- SIM IO must never read raw world state directly; it must read staged deterministic sensor outputs.
- All sensor outputs must come from seeded, latency-aware deterministic pipelines.
- All randomness must be seeded and reproducible.
- All ordering must be stable for bodies, contacts, articulated constraints, sensor releases, and replay records.
- Replay must include all authoritative state required for deterministic diffing.
- Predictive and visual overlays must never feed back into world stepping, sensors, or replay truth.

## Maintenance Checklist

- Required validation before merging:
  - `./gradlew test verifyDeterministicShotReplay`
  - targeted physics tests
  - targeted sensor and vision tests
  - targeted replay/transport tests
- Verify before merging:
  - determinism hash did not change unexpectedly
  - no new truth path bypasses the authoritative world
  - no SIM IO reads raw world state directly
  - no snapshot fields needed for replay diffing were omitted
- High-risk files:
  - `simulation/engine/PhysicsWorld.java`
  - `simulation/engine/CollisionSystem.java`
  - `simulation/engine/SequentialImpulseSolver.java`
  - `simulation/engine/ArticulatedModuleAssembly.java`
  - `simulation/sensors/LocalSwerveSensorModel.java`
  - `simulation/contracts/FrameBinaryCodec.java`
  - `simulation/replay/PhysicsSnapshotFactory.java`

## Repository Guide

| Area | Primary paths | What lives there | Short docs |
| --- | --- | --- | --- |
| Robot entry + wiring | `src/main/java/org/Griffins1884/frc2026` | `Main`, `Robot`, `RobotContainer`, global config, top-level runtime wiring. | [Architecture][arch], [Checklist][checklist] |
| Drive / odometry / calibration | `src/main/java/org/Griffins1884/frc2026/subsystems/swerve`<br>`src/main/java/org/Griffins1884/frc2026/commands` | Swerve IO, estimator, gyro failover, drive commands, SysId hooks, zeroing/calibration workflows. | [Swerve][swerve-doc], [Performance][performance-doc], [Checklist][checklist] |
| Vision | `src/main/java/org/Griffins1884/frc2026/subsystems/vision`<br>`src/main/deploy/apriltags`<br>`src/main/deploy/tagfields` | AprilTag pipelines, pose filtering, camera configs, field/tag assets. | [Vision][vision-doc], [Northstar/Limelight][northstar-doc] |
| Superstructure + mechanisms | `src/main/java/org/Griffins1884/frc2026/subsystems`<br>`src/main/java/org/Griffins1884/frc2026/mechanisms` | Coordinated state machine behavior, reusable mechanism definitions, roller/arm/turret composition. | [Mechanisms][mechanism-doc], [Phase Report][phase-doc] |
| Operator board + persistence | `src/main/deploy/operatorboard`<br>`operatorboard-data`<br>`src/main/java/org/Griffins1884/frc2026/subsystems/objectivetracker` | Served dashboard assets, persisted requests/state, runtime diagnostics, web-backed operator workflows. | [Persistence][persistence-doc], [NT Audit][nt-doc] |
| Autonomous + path tooling | `src/main/deploy/pathplanner`<br>`src/main/deploy/choreo`<br>`tools/pathplana_contract` | PathPlanner/Choreo assets, auto definitions, PathPlanA contracts and schema tooling. | [PathPlanA Integration][pathplana-int], [PathPlanA Contract][pathplana-contract] |
| Simulation | `src/main/java/org/Griffins1884/frc2026/simulation`<br>`docs/simulation` | Local deterministic physics, lockstep/replay tooling, visualization, terrain model, and sim review flow. | [Sim Arch][sim-arch], [Sim Review][sim-review], [CAD Pipeline][sim-cad], [Field CAD][field-cad-doc] |
| Deploy assets + AdvantageScope | `src/main/deploy/advantagescope`<br>`src/main/deploy/music` | Layouts, robot assets, visualization support files, and deploy-time resources used by tools and demos. | [Architecture][arch], [Performance][performance-doc] |
| Tests + verification | `src/test/java/org/Griffins1884/frc2026`<br>`config/logging` | Unit/integration coverage for control logic plus logging policy checks wired into Gradle. | [Checklist][checklist], [Performance][performance-doc] |
| Vendor deps + support tools | `vendordeps`<br>`tools/deploy_preserve`<br>`tools/northstar` | Pinned vendor libraries, deploy-preserve scripts, and camera bring-up tooling. | [Persistence][persistence-doc], [Northstar/Limelight][northstar-doc], [Northstar Tooling][northstar-tool-doc] |

## Top-Level Layout

| Path | Purpose |
| --- | --- |
| `src/main/java/org/Griffins1884/frc2026` | Main robot codebase: commands, subsystems, simulation, utilities, runtime wiring. |
| `src/test/java/org/Griffins1884/frc2026` | Tests for drive, vision, ballistics, and regression-prone subsystem logic. |
| `src/main/deploy` | Deploy-time assets including Operator Board UI, paths, AprilTags, and AdvantageScope data. |
| `operatorboard-data` | Local persistent dashboard/runtime data mirrored with the roboRIO. |
| `docs` | Design notes, operating procedures, simulation docs, audits, and season reports. |
| `tools` | Small support scripts and schemas for deploy-preserve, Northstar, and PathPlanA workflows. |
| `vendordeps` | Version-pinned vendor JSON files for the robot stack. |

## Docs Index

- Drive and odometry: [Swerve][swerve-doc]
- Vision and cameras: [Vision][vision-doc], [Northstar/Limelight][northstar-doc]
- Operator board and persistence: [Persistence][persistence-doc], [NT Audit][nt-doc]
- Autonomous contracts: [PathPlanA Integration][pathplana-int], [PathPlanA Contract][pathplana-contract]
- Mechanism planning: [Mechanisms][mechanism-doc], [Phase Report][phase-doc]
- Simulation: [Sim Arch][sim-arch], [Sim Review][sim-review], [CAD Pipeline][sim-cad], [Field CAD][field-cad-doc]
- Performance and validation: [Performance][performance-doc], [Checklist][checklist]

## Getting Started

1. **Tools** - Install WPILib 2026 beta, JDK 17, and AdvantageScope.
2. **Clone** - `git clone https://github.com/frc1884/season2026.git`
3. **Build** - `./gradlew build`
4. **Sim** - `./gradlew simulateJava`
5. **Deterministic replay verification** - `./gradlew verifyDeterministicShotReplay`
6. **Headless engine server** - `./gradlew runDeterministicShotEngineServer`
7. **Replay diff** - `./gradlew replayDiff --args="<left-log> <right-log>"`
8. **Deploy** - `./gradlew deploy`
9. **Deploy + preserve saved data** - `./gradlew deployPreserve`
10. **Operator Board UI** - Browse to `http://<roboRIO>:5805`
11. **Remote tablets** - Append `?ntHost=<robot-ip>` and optionally `&ntPort=<port>` when hosting the UI elsewhere

## Simulation Truth Path

1. Commanded module targets are staged through `ModuleIOSim`.
2. `RobotContainer.simulationPeriodic()` advances the authoritative local physics world.
3. Articulated module bodies, wheel-ground forces, field collisions, and gamepieces are solved in one fixed-step world.
4. Deterministic sensor models stage gyro/module outputs with seeded noise and latency before SIM IO reads them.
5. Simulated vision samples the same world with fixed cadence, deterministic latency, frustum gating, and occlusion checks.
6. Replay snapshots and server/tooling snapshots are serialized from the same authoritative world state.

## Authoritative vs Predictive

- Authoritative:
  - articulated robot world state
  - field and gamepiece collision state
  - staged sensor outputs
  - replay snapshots
- Predictive only:
  - shot arc overlays
  - impact marker overlays
  - targeting preview overlays

## Persistent Data

- Local saved dashboard data lives in `operatorboard-data/`.
- roboRIO runtime saved data lives in `/home/lvuser/operatorboard-data`.
- Deploy-seeded defaults live in `src/main/deploy/operatorboard/default-data/`.
- PathPlanA autos are preserved across deploys through the `deployPreserve` workflow.
- The operator board exposes subsystem/state descriptions and diagnostic bundle export endpoints.

See [Persistence][persistence-doc] for the full file inventory, sync rules, and conflict policy.

## Tech Stack

There are way too many people to thank and the list gets longer virtually every day, but we'd like to extend a massive thank you to every FRC team for publicizing their 2023-24 season repositories as inspiration for this project. Without that gracious professionalism, this project would not exist in its current form.

Libraries and vendors used here:

- Team 6328 Mechanical Advantage (AdvantageKit, AdvantageScope, URCL)
- CTR Electronics (Phoenix)
- PhotonVision (PhotonVision, PhotonLib)
- REV Robotics (REVLib)
- Studica (NavX2)
- WPILib Suite (WPILib and the FRC software ecosystem)

And to every team sharing 2024-2026 codebases: you continue to make FRC better. If you reuse anything here, let us know; we like contributing upstream.

*(Updated for the 2026 build season)*

[arch]: ARCHITECTURE.md
[checklist]: docs/ROBOT_FUNCTIONALITY_CHECKLIST.md
[swerve-doc]: docs/SWERVE_ODOMETRY_AND_CALIBRATION.md
[vision-doc]: docs/VISION_POSE_USAGE.md
[persistence-doc]: docs/PERSISTENCE_ARCHITECTURE.md
[northstar-doc]: docs/NORTHSTAR_LIMELIGHT_SETUP.md
[pathplana-int]: docs/PATHPLANA_INTEGRATION.md
[pathplana-contract]: docs/PATHPLANA_CONTRACT.md
[nt-doc]: docs/NETWORKTABLES_AUDIT.md
[performance-doc]: docs/PERFORMANCE_VALIDATION.md
[mechanism-doc]: docs/MECHANISM_ARCHITECTURE_PLAN.md
[phase-doc]: docs/PHASE_COMPLETION_REPORT.md
[sim-arch]: docs/simulation/architecture.md
[sim-cad]: docs/simulation/cad-pipeline.md
[sim-review]: docs/simulation/review-workflow.md
[field-cad-doc]: docs/simulation/field-cad/README.md
[northstar-tool-doc]: tools/northstar/README.md
