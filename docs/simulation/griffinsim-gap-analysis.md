# GriffinSim Gap Analysis

## Current vs Required

| Area | Current State | Required State | Severity |
| --- | --- | --- | --- |
| Repository identity | Robot program consuming GriffinSim and MapleSim | GriffinSim engine/control/runtime repo or a clearly separated local execution layer | `BLOCKER` |
| Control host | Standard WPILib `Robot` process only | Explicit control host with real-time and lockstep stepping modes | `BLOCKER` |
| HALSIM bridge | External vendordep native extension only | Local, documented, deterministic bridge contract and non-blocking queue boundary | `BLOCKER` |
| IPC | No local IPC contract or versioned schema | Versioned binary frame schema for actuator, sensor, and world state transport | `BLOCKER` |
| Physics authority | In-process Maple/GriffinSim adapter wiring | Authoritative physics boundary with explicit frame timestamps and step ids | `MAJOR` |
| Replay verification | AdvantageKit replay plus visualization markers | Deterministic replay traces and a comparer proving identical outputs across runs | `BLOCKER` |
| Time source | Wall-clock timestamps in simulation/replay helpers | Deterministic time source abstraction with fixed-step control | `MAJOR` |
| Rendering boundary | AdvantageScope outputs exist | Renderer must remain subscriber-only and consume authoritative snapshots | `MINOR` |
| Documentation | General sim docs only | Audit report, gap analysis, migration plan, and rebuilt-system note | `BLOCKER` |
| Tests | Unit tests for robot logic and projectile math | Deterministic replay tests for the reconstructed sim surfaces | `MAJOR` |

## Concrete Missing Pieces Before This Pass

1. No local `ActuatorFrame`, `SensorFrame`, or `WorldSnapshot` contracts.
2. No stable binary codec for deterministic frame comparison.
3. No manual simulation time source for stepping replayable runs.
4. No replay trace hashing or diffing utility.
5. No CLI or Gradle entry point for deterministic replay verification.
6. No repo-local audit/migration documents required by the target specification.

## Residual Gaps After This Pass

The current reconstruction work can close the contract, documentation, and local replay-proof gaps
inside this repo. It cannot fully replace the external GriffinSim engine or its native HALSIM
extension from within `season2026` alone. That remains an architectural boundary and should be
treated as phase-2 work unless the engine repo is moved into this workspace.
