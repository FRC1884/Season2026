# GriffinSim I/O Contract v1

## Purpose

This document defines the versioned local frame contract used by the deterministic replay surfaces
in `season2026`.

## Version

- Schema id: `griffinsim-io-v1`
- Version number: `1`

## Frames

### `ActuatorFrame`

Fields:

- `simTimeNanos`
- `stepId`
- `robotPose`
- `fieldVelocityMetersPerSecond`
- `turretYawRadians`
- `pivotMotorRotations`
- `shooterRpm`
- `shooterArmed`

### `SensorFrame`

Fields:

- `simTimeNanos`
- `stepId`
- `predictionAvailable`
- `predictionFeasible`
- `shotReleased`
- `activeProjectileCount`
- `projectileSpawnCount`
- `closestApproachErrorMeters`
- `timeOfFlightSeconds`

### `WorldSnapshot`

Fields:

- `simTimeNanos`
- `stepId`
- `predictionAvailable`
- `predictionFeasible`
- `releasePose`
- `impactPose`
- `activeProjectilePoses`

## Binary Encoding

Each encoded frame starts with:

1. `uint16` schema version
2. `uint8` frame type

Frame types:

- `1` = actuator
- `2` = sensor
- `3` = world snapshot

The canonical codec implementation lives in:

- `org.Griffins1884.frc2026.simulation.contracts.FrameBinaryCodec`

## Determinism Rules

- `simTimeNanos` and `stepId` must be monotonic.
- Deterministic runs must not depend on wall-clock time.
- The same scenario, step count, and seedless inputs must produce identical encoded byte streams.
