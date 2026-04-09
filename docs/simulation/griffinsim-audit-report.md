# GriffinSim Audit Report

## Scope

This audit classifies the current `season2026` repository against the required GriffinSim
reconstruction architecture. The current repository is a robot program that consumes GriffinSim,
not the GriffinSim engine repository itself.

## Classification

| Path | Classification | Reasoning |
| --- | --- | --- |
| `README.md` | `MODIFY` | Correctly describes the repo as robot code, but it does not state the boundary between this consumer repo and the missing GriffinSim engine/runtime. |
| `build.gradle` | `MODIFY` | Adds the GriffinSim native HALSIM extension as an external dependency, but does not define local engine, bridge, replay, or verification modules. |
| `vendordeps/GriffinSim.json` | `KEEP` | Correctly pins the external GriffinSim/runtime dependency surface. It is useful as an integration contract while local deterministic scaffolding is added. |
| `src/main/java/org/Griffins1884/frc2026/Robot.java` | `MODIFY` | Uses the standard in-process WPILib sim loop and wall-clock logging. No lockstep scheduler or deterministic replay boundary exists. |
| `src/main/java/org/Griffins1884/frc2026/RobotContainer.java` | `MODIFY` | Simulation wiring is tightly coupled to Maple/GriffinSim adapters and singleton arena state. The code is usable, but not yet separated into authoritative control/physics/render boundaries. |
| `src/main/java/org/Griffins1884/frc2026/simulation/maple/MapleArenaSetup.java` | `KEEP` | Useful arena-selection shim for the current consumer repo. It can stay as a compatibility adapter while deeper deterministic control is introduced elsewhere. |
| `src/main/java/org/Griffins1884/frc2026/simulation/maple/MapleArenaAdapter.java` | `MODIFY` | Hides the Maple singleton, but still exposes direct in-process stepping only. No explicit scheduling or replay metadata is attached to steps. |
| `src/main/java/org/Griffins1884/frc2026/simulation/maple/MapleProjectileBridge.java` | `DELETE` | Pure placeholder. It provides no bridge semantics, no world ownership, and no determinism guarantees. |
| `src/main/java/org/Griffins1884/frc2026/simulation/shooter/*` | `MODIFY` | Useful deterministic-friendly math and projectile state logic already exist, but the package is framed as shot-only visualization rather than a replayable simulation subsystem. |
| `src/main/java/org/Griffins1884/frc2026/simulation/replay/ShotReviewEvents.java` | `MODIFY` | Useful event surface, but it uses wall-clock time by default and only records AdvantageKit outputs, not canonical replay data. |
| `src/main/java/org/Griffins1884/frc2026/simulation/visualization/*` | `KEEP` | Good renderer-facing output layer. It should remain non-authoritative and subscribe to deterministic state rather than own state transitions. |
| `docs/simulation/architecture.md` | `MODIFY` | Explicitly documents a shot-only visualization workflow, which is below the target GriffinSim architecture. |
| `src/test/java/org/Griffins1884/frc2026/simulation/shooter/*` | `KEEP` | Existing deterministic-friendly unit coverage for local projectile math is valuable and should be extended, not removed. |
| `src/test/java/org/Griffins1884/frc2026/UnitTests.java` | `MODIFY` | Contains commented lockstep test ideas, which are useful references, but they are not active verification. |
| Root-level repo layout | `MODIFY` | No `contracts`, `control-host`, `bridge`, `engine`, or replay verification structure existed before this reconstruction pass. |

## Determinism Findings

- The runtime currently mixes deterministic math with wall-clock timestamps such as
  `Timer.getFPGATimestamp()`.
- Simulation stepping is still single-process and singleton-driven.
- There was no canonical frame contract or stable binary encoding for sim I/O before this pass.
- There was no replay verifier capable of proving repeated runs produced identical frame streams.

## Threading Findings

- No local HALSIM callback implementation exists in this repo; the bridge is external.
- The repo therefore cannot prove non-blocking callback behavior locally.
- Rendering and visualization are correctly kept separate from robot actuation logic, but state
  ownership still lives inside the in-process robot runtime.

## Summary

This repo had reusable robot-consumer simulation pieces, but it had not completed the GriffinSim
execution-layer setup described in the target specification. The work in this reconstruction pass
focuses on adding the missing audit trail, deterministic contracts, local replay verification, and
time-source abstraction without breaking the existing robot program.
