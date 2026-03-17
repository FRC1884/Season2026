# NetworkTables Audit

## Scope

This audit covers direct NetworkTables usage and dashboard traffic patterns after the OperatorBoard
runtime/debug rebuild. It does not attempt to fully quantify AdvantageKit log traffic, but it
calls out logger-heavy areas that still contribute to dashboard load in debug mode.

## Highest-Risk Hotspots

1. `OperatorBoardTracker`
   File: `src/main/java/org/Griffins1884/frc2026/subsystems/objectivetracker/OperatorBoardTracker.java`
   Risk: high-frequency JSON/string topics published every telemetry cycle.
   Notes:
   - Queue state, runtime profile, system checks, auto checks, NT diagnostics, mechanism status,
     and action trace were all being reserialized at 20 Hz while enabled.
   - This is the clearest direct NT traffic hotspot because it mixes frequency with larger payloads.

2. `OperatorBoardIOServer`
   File: `src/main/java/org/Griffins1884/frc2026/subsystems/objectivetracker/OperatorBoardIOServer.java`
   Risk: broad publisher surface.
   Notes:
   - A large set of output publishers exists for the operator board.
   - Publisher registration is not leaking, but a wide topic surface still makes change gating
     valuable.

3. `GamePieceVisionIOLimelight`
   File: `src/main/java/org/Griffins1884/frc2026/subsystems/vision/GamePieceVisionIOLimelight.java`
   Risk: forced NT flush in the vision loop.
   Notes:
   - Per-loop `NetworkTableInstance.flush()` amplifies traffic and can defeat normal batching.

4. `operatorboard/index.js`
   File: `src/main/deploy/operatorboard/index.js`
   Risk: wide subscription surface plus JSON parsing in the browser.
   Notes:
   - The main board subscribes to many topics and parses multiple JSON payloads continuously.
   - This is more of a dashboard load issue than a robot crash source, but it still affects stale
     data behavior.

5. `operatorboard/runtime-config.js`
   File: `src/main/deploy/operatorboard/runtime-config.js`
   Risk: full namespace subscription for the topic viewer.
   Notes:
   - Useful for diagnostics, but expensive if left open during active operation.

## Implemented Mitigations

- Added sampled OperatorBoard NT publish-rate and byte-rate diagnostics to the robot-side
  diagnostics payload.
- Added per-topic hot-topic ranking for the heaviest sampled OperatorBoard outputs.
- Delta-gated the heavy string/JSON OperatorBoard topics so identical payloads are no longer
  republished every telemetry cycle.
- Rate-limited the game-piece Limelight NT flush path to 10 Hz instead of every loop.
- Surfaced NT output-rate warnings in the operator board alerts strip.
- Changed `runtime-config.js` so the full OperatorBoard namespace viewer is opt-in instead of
  always-on; the page now keeps only runtime-profile topics subscribed by default.
- Reduced the main operator-board subscription period to match the robot publish cadence instead of
  subscribing faster than the robot updates.
- Kept auto quick-run validation on-demand so the queue dry-run logic does not add steady-state NT
  traffic.
- Moved high-rate `AutoAlignToPoseCommand` and `TurretCommands` debug logging onto the runtime
  profile gate so COMP mode stays leaner.

## Remaining Follow-Up

- Remaining follow-up is now hardware validation, not additional required software mitigation.
- If future queue/debug payloads grow materially, reconsider splitting some JSON blobs into
  smaller structured topics.
