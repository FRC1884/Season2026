# FRC 1884 • 2026 REBUILT Robot Code

> The official software stack for Team 1884 Griffins’ 2026 build season (REBUILT). Everything that hits the field, from drivetrain controls to the operator dashboard, lives here.

## 2026 Season Highlights

- **Operator Board UI** - Lightweight tablet web app served by the robot for state requests and at-a-glance telemetry.
- **Drive + Turn SysId** - Separate drivetrain and steer characterization routines so pit-side retuning does not require code edits.
- **State-first subsystems** - AdvantageKit logging, IO abstraction, and command-based verbs are used throughout the robot stack.
- **Simulation readiness** - GriffinSim and maple-sim support terrain-aware simulation before the full robot is wired.

## Start Here

- Main architecture map: [Architecture][arch]
- Match/practice validation checklist: [Checklist][checklist]
- Swerve calibration and odometry flow: [Swerve][swerve-doc]
- Vision pose acceptance and consumers: [Vision][vision-doc]
- Persistence and deploy-preserve behavior: [Persistence][persistence-doc]
- Simulation design docs: [Sim Arch][sim-arch], [Sim Review][sim-review], [CAD Pipeline][sim-cad]

## Repository Guide

| Area | Primary paths | What lives there | Short docs |
| --- | --- | --- | --- |
| Robot entry + wiring | `src/main/java/org/Griffins1884/frc2026` | `Main`, `Robot`, `RobotContainer`, global config, top-level runtime wiring. | [Architecture][arch], [Checklist][checklist] |
| Drive / odometry / calibration | `src/main/java/org/Griffins1884/frc2026/subsystems/swerve`<br>`src/main/java/org/Griffins1884/frc2026/commands` | Swerve IO, estimator, gyro failover, drive commands, SysId hooks, zeroing/calibration workflows. | [Swerve][swerve-doc], [Performance][performance-doc], [Checklist][checklist] |
| Vision | `src/main/java/org/Griffins1884/frc2026/subsystems/vision`<br>`src/main/deploy/apriltags`<br>`src/main/deploy/tagfields` | AprilTag pipelines, pose filtering, camera configs, field/tag assets. | [Vision][vision-doc], [Northstar/Limelight][northstar-doc] |
| Superstructure + mechanisms | `src/main/java/org/Griffins1884/frc2026/subsystems`<br>`src/main/java/org/Griffins1884/frc2026/mechanisms` | Coordinated state machine behavior, reusable mechanism definitions, roller/arm/turret composition. | [Mechanisms][mechanism-doc], [Phase Report][phase-doc] |
| Operator board + persistence | `src/main/deploy/operatorboard`<br>`operatorboard-data`<br>`src/main/java/org/Griffins1884/frc2026/subsystems/objectivetracker` | Served dashboard assets, persisted requests/state, runtime diagnostics, web-backed operator workflows. | [Persistence][persistence-doc], [NT Audit][nt-doc] |
| Autonomous + path tooling | `src/main/deploy/pathplanner`<br>`src/main/deploy/choreo`<br>`tools/pathplana_contract` | PathPlanner/Choreo assets, auto definitions, PathPlanA contracts and schema tooling. | [PathPlanA Integration][pathplana-int], [PathPlanA Contract][pathplana-contract] |
| Simulation | `src/main/java/org/Griffins1884/frc2026/simulation`<br>`docs/simulation` | GriffinSim/maple adapters, replay hooks, visualization, field contact models, sim review flow. | [Sim Arch][sim-arch], [Sim Review][sim-review], [CAD Pipeline][sim-cad], [Field CAD][field-cad-doc] |
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
5. **Deploy** - `./gradlew deploy`
6. **Deploy + preserve saved data** - `./gradlew deployPreserve`
7. **Operator Board UI** - Browse to `http://<roboRIO>:5805`
8. **Remote tablets** - Append `?ntHost=<robot-ip>` and optionally `&ntPort=<port>` when hosting the UI elsewhere

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
- Team 1884 Griffins (GriffinSim)
- Team 5516 Iron Maple (maple-sim)
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
