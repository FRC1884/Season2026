# Performance Validation

## Scope

This validation closes the software-side performance and stability work for the mechanism/runtime/
dashboard phases in this repo. It focuses on NetworkTables and debug/logging pressure because that
was the highest-risk crash source during the rebuild.

## Current Publish Cadence

- OperatorBoard robot telemetry publish period while enabled:
  - `0.05 s` (`20 Hz`)
- OperatorBoard robot telemetry publish period while disabled:
  - `0.10 s` (`10 Hz`)
- Main operator-board browser subscription period:
  - `0.05 s` (`20 Hz`)
- Runtime-config default subscription period:
  - `0.10 s` (`10 Hz`)
- Runtime-config full topic viewer:
  - opt-in only
- Game-piece Limelight forced NT flush path:
  - `10 Hz` rate-limited

## Stability / Throughput Safeguards

- Heavy OperatorBoard JSON/string topics are delta-gated and are not republished when unchanged.
- Auto quick-run validation is on-demand only and does not execute continuously in the background.
- The runtime-config page no longer subscribes to the full OperatorBoard namespace unless the live
  topic viewer is explicitly enabled.
- The main operator board now subscribes at the same rate the robot publishes instead of polling at
  a higher rate.
- Migrated mechanism debug logs are runtime-profile gated so COMP mode avoids their high-rate
  signal spam.
- High-rate per-loop diagnostics in `AutoAlignToPoseCommand` and `TurretCommands` now follow the
  runtime debug profile instead of the old global logging flag.

## Verification Results

Validated in this workspace with:

- `node --check src/main/deploy/operatorboard/index.js`
- `node --check src/main/deploy/operatorboard/runtime-config.js`
- `./gradlew compileJava`
- `./gradlew check`

## Expected Outcome

- Less NT churn from unchanged dashboard state.
- Lower dashboard-side subscription pressure.
- Reduced risk of vision-loop flush bursts overwhelming NT batching.
- Reduced COMP-mode logger pressure from migrated mechanisms and aiming commands.

## Remaining Real-World Validation

Software validation is complete. Remaining work is on-robot verification:

- confirm NT stability with the dashboard open during full-system testing
- watch sampled NT byte rate and hot-topic ranking under aggressive debug use
- confirm CAN/NT alerts remain quiet during match-like operation
