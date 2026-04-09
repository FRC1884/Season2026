# Simulation Architecture

## Scope

This repo now uses a rebuilt local simulation runtime:

- deterministic drivetrain physics in-process for the active WPILib SIM mode
- fixed-step replay contracts and replay logs
- a socket-backed authoritative shot engine for process-separated verification
- AdvantageScope visualization on top of the same local terrain/physics state

CAD assets remain optional and are not required for the current workflow.

## SIM Architecture And Invariants

- Exactly one authoritative world exists in active SIM.
- No secondary or parallel truth sources are permitted.
- Actuator staging enters the simulation only through SIM IO and the authoritative world step.
- SIM IO must never read raw world state directly.
- Sensor outputs must come from staged deterministic pipelines with seeded noise and explicit latency.
- Bodies, contacts, articulated constraints, sensor emissions, and replay records must all use stable ordering.
- Replay must serialize all authoritative state required for deterministic diffing.
- Predictive and visual overlays must never feed back into world stepping, sensors, or replay truth.

## Runtime Packages

- `org.Griffins1884.frc2026.util.ballistics`
  - pure shot-model math and configurable geometry
- `org.Griffins1884.frc2026.simulation.shooter`
  - field-space shot solving, projectile state, projectile manager, release detection
- `org.Griffins1884.frc2026.simulation.visualization`
  - robot component pose publishing, shot-arc publishing, active projectile publishing
- `org.Griffins1884.frc2026.simulation.replay`
  - shot-review markers, deterministic replay traces, diffing, and scenario runners
- `org.Griffins1884.frc2026.simulation.maple`
  - local terrain surface model retained under the historical package path
- `org.Griffins1884.frc2026.simulation.contracts`
  - versioned actuator/sensor/world frame contracts and binary codec
- `org.Griffins1884.frc2026.simulation.runtime`
  - bounded queues and replay log read/write support
- `org.Griffins1884.frc2026.simulation.transport`
  - socket transport and engine server for process-separated deterministic execution
- `org.Griffins1884.frc2026.simulation.engine`
  - authoritative fixed-step shot engine
- `org.Griffins1884.frc2026.simulation.physics`
  - authoritative articulated chassis/module world, 3D terrain response, bounded field interaction,
    and unified gamepiece stepping

## Logged Outputs

- `FieldSimulation/ShotReleasePose`
- `FieldSimulation/RobotPose3d`
- `FieldSimulation/TurretComponentPose3d`
- `FieldSimulation/ShooterPivotComponentPose3d`
- `FieldSimulation/ShooterExitPose3d`
- `FieldSimulation/RobotComponentPoses`
- `FieldSimulation/PredictedShotArc`
- `FieldSimulation/PredictedImpactPose`
- `FieldSimulation/ActiveProjectiles`
- `FieldSimulation/ShotMarkers`
- `ShotReview/*`

## Current Behavior

- The active `SIM` robot mode uses an articulated deterministic physics world.
- The chassis is dynamic and each swerve pod is its own rigid body coupled by deterministic joint
  assemblies.
- Wheel-ground interaction is resolved through per-wheel contact patches attached to the module pod
  bodies.
- Gamepieces live in the same world and are stepped on the same fixed-timestep clock.
- Field bounds and major collision structures are authoritative static bodies supplied by the field
  model.
- Deterministic verification can run without the interactive GUI because replay snapshots are
  emitted from the same world used by active SIM and the transport/server lane.

## Authoritative Truth Path

1. `Module` and `SwerveSubsystem` stage actuator targets into `ModuleIOSim`.
2. `RobotContainer.simulationPeriodic()` advances `LocalSwervePhysicsSimulation`.
3. `LocalSwervePhysicsSimulation` steps:
   - articulated module joints
   - wheel-ground forces
   - collision broadphase/narrowphase
   - sequential impulse solve
   - unified gamepiece bodies
4. Raw world state is staged through deterministic sensor pipelines before SIM IO reads it.
5. Vision SIM samples the same world with fixed cadence, latency, frustum gating, and occlusion.
6. `PhysicsSnapshotFactory` serializes the authoritative world for replay and tooling.

## Field Authority

- Analytic terrain remains for bump pitch/roll/height sampling.
- Static collision bodies define the authoritative bounded field volume:
  - floor
  - walls
  - bump structures
  - major blocking geometry represented as deterministic compound primitives
- For this repo scope, compound primitives are sufficient because the active interactions are
  drivetrain traversal, wall scraping, bump traversal, and robot/gamepiece collisions against large
  field structures. No current active interaction requires fine mesh contact to preserve behavior.

## Sensor Pipeline

- Gyro and module sensors are sampled from the authoritative world, then staged through
  `LocalSwerveSensorModel`.
- Noise is seeded and deterministic.
- Release order is latency-queue driven and stable.
- SIM IO reads staged outputs only.

## Vision SIM

- Camera poses come from the authoritative world and camera extrinsics.
- Visibility requires:
  - frustum acceptance
  - frame cadence
  - deterministic pipeline latency
  - simple occlusion rejection against authoritative world AABBs
- Measurement noise scales with range and off-axis viewing angle.
- Limitations:
  - occlusion is box/AABB based
  - no rasterized optical pipeline
  - intended to model deterministic measurement behavior, not photorealistic imagery

## Determinism Guarantees

- Fixed timestep only.
- Stable ordering for bodies, contacts, constraints, sensors, and replay records.
- Seeded randomness only.
- Replay serialization includes rigid bodies, contacts, wheel contact telemetry, and world
  projectile state needed for deterministic diffing.

## Maintenance Checklist

- Run before merge:
  - `./gradlew test verifyDeterministicShotReplay`
  - targeted engine tests
  - targeted sensor and vision tests
  - targeted replay and transport tests
- Re-check whenever touching:
  - `simulation/engine/PhysicsWorld.java`
  - `simulation/engine/CollisionSystem.java`
  - `simulation/engine/SequentialImpulseSolver.java`
  - `simulation/engine/ArticulatedModuleAssembly.java`
  - `simulation/sensors/LocalSwerveSensorModel.java`
  - `simulation/contracts/FrameBinaryCodec.java`
  - `simulation/replay/PhysicsSnapshotFactory.java`
