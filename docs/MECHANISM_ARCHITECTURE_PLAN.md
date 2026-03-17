# Mechanism Architecture Plan

## Scope

This document defines the Phase 0 and early Phase 1 target for the non-drivetrain, non-vision
mechanism redesign in this repo.

Excluded for this plan:

- drivetrain
- vision

Included in the current repo inventory:

- `IntakeSubsystem`
- `IndexerSubsystem`
- `ShooterSubsystem`
- `IntakePivotSubsystem`
- `ShooterPivotSubsystem`
- `TurretSubsystem`
- `ToothRolloutSubsystem`
- `SpindexerSubsystem`

Requested future mechanisms now present as disabled framework-ready subsystems in this repo:

- intake tooth gear roll-out mechanism
- spindexer / indexer turntable

Those two are scaffolded on top of the same framework and remain disabled by default until real
hardware wiring and controls are ready.

## Current State

The repo already has a partial generic mechanism layer, but it is fragmented by family:

- `generic/rollers`
- `generic/arms`
- `generic/turrets`
- `generic/elevators`

That layer provides useful behavior, but it has three structural problems:

1. Mechanism definition, hardware config, runtime control, and telemetry are mixed together.
2. Each family has its own IO contract and logging shape.
3. Mode/debug behavior is mostly global through `GlobalConstants` instead of profile-driven and
   subsystem-selective.

The redesign should unify those concerns without forcing an immediate rewrite of every vendor IO
class.

## Repo-Specific Target Architecture

The new base package is:

- `org.Griffins1884.frc2026.mechanisms`

The initial foundation in that package is responsible for:

- declaring a mechanism in a vendor-agnostic way
- validating config before mechanism creation
- standardizing telemetry shape
- standardizing health/fault reporting contracts
- providing runtime mode hooks that later dashboard/config code can drive

The first-pass class contract is:

- `MechanismDefinition`
  - repo-wide declarative definition for one mechanism
  - owns mechanism type, motor group config, closed-loop config, telemetry policy, simulation
    config, and vendor extensions
- `MechanismIO`
  - common low-level interface for mechanism-capable hardware adapters
  - does not replace existing family IO immediately; it becomes the target contract for new and
    migrated adapters
- `MechanismTelemetry`
  - normalized telemetry snapshot model
  - defines the baseline signal categories for logging and NetworkTables publishing
- `MechanismHealth`
  - common health state for operator/debug workflows

Mode/debug control is owned by:

- `org.Griffins1884.frc2026.runtime.RuntimeModeProfile`
- `org.Griffins1884.frc2026.runtime.RuntimeModeManager`

Those classes define:

- global COMP vs DEBUG profile
- subsystem-specific debug enablement
- signal-level log/publish policy
- tuning enablement policy

## Required Config Fields

Every migrated mechanism definition must be able to specify:

- mechanism key
- display name
- mechanism type
- motor controller type
- CAN bus
- motor count
- CAN IDs
- inversion per motor
- follower mode per motor
- current limits
- neutral mode
- voltage compensation
- open-loop and closed-loop ramp rates
- gear ratio
- feedback sensor type
- soft limits
- PID values
- feedforward values
- motion constraints
- max voltage
- telemetry policy
- fault policy
- simulation parameters
- Kraken-only extension settings when applicable

Config cleanup rules for this repo:

- the mechanism catalog is the source of truth for runtime capability declarations
- config declarations should use capability sets and shared helpers instead of raw boolean flags
- duplicate config constants should be collapsed where the meaning is identical
- constants should be named consistently and grouped by behavior instead of vendor history
- legacy PathPlanner-era or one-off setup leftovers should be removed when no longer used

## Required Telemetry Baseline

Every migrated mechanism should be able to report the same baseline categories:

- identity
- connection state
- faults and warnings
- temperature
- voltage
- current
- position
- velocity
- applied output
- target state
- closed-loop error
- limit switch state
- sensor state
- overall health
- config snapshot

The old family-specific input structs can remain during migration, but each migrated subsystem
should be able to map into the new telemetry shape.

## Runtime Mode Policy

The new mode model is:

- `COMP`
  - only competition-safe telemetry categories
  - no implicit tuning UI
  - minimal NetworkTables publishing
- `DEBUG`
  - all telemetry categories enabled
  - tuning allowed
  - richer diagnostics and inspection
- subsystem debug
  - keeps the global profile in `COMP`
  - enables expanded telemetry for a named subsystem only

Initial subsystem keys for this repo:

- `intake`
- `indexer`
- `shooter`
- `intakePivot`
- `shooterPivot`
- `turret`

## Migration Order

The migration order should match repo complexity instead of the original generic list.

First wave:

1. `IntakeSubsystem`
2. `IndexerSubsystem`
3. `ShooterSubsystem`

Second wave:

1. `IntakePivotSubsystem`
2. `ShooterPivotSubsystem`
3. `TurretSubsystem`

Future wave after those are stable:

1. tooth gear roll-out mechanism
2. spindexer

That future wave is now satisfied at the framework level through disabled placeholder-ready
subsystems so the mechanism list from the original phase plan is represented in code.

## Implementation Notes

The migration strategy should be incremental:

1. Introduce the new `mechanisms/` and `runtime/` foundations.
2. Route logging/tuning policy through the new runtime profile layer.
3. Migrate simple roller systems onto the new definition/config contract first.
4. Migrate pivot/turret systems once the config and telemetry model proves out.
5. Remove legacy family-specific setup utilities only after all target mechanisms are moved.

The old `generic/rollers`, `generic/arms`, and `generic/turrets` packages should be treated as
legacy compatibility layers during the transition, not as the final design target.

Shooter-specific cleanup policy:

- the shooter uses one velocity PID/feedforward profile
- no high-slot vs low-slot gain switching should remain in the migrated shooter path
- recovery behavior can add compensation on top of that one profile, but it should not swap
  controller gain sets at runtime
