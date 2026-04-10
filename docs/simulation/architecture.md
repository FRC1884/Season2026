# Simulation Architecture

## Scope

This repo now treats shot-only visualization as the primary workflow:

- robot-side ballistic solving and projectile prediction
- visualization and replay in AdvantageScope

MapleSim remains the drivetrain and future arena/game-piece layer. CAD assets are optional and are
not required for the current shot-review workflow.

## Runtime Packages

- `org.Griffins1884.frc2026.util.ballistics`
  - pure shot-model math and configurable geometry
- `org.Griffins1884.frc2026.simulation.shooter`
  - field-space shot solving, projectile state, projectile manager, release detection
- `org.Griffins1884.frc2026.simulation.visualization`
  - robot component pose publishing, shot-arc publishing, active projectile publishing
- `org.Griffins1884.frc2026.simulation.replay`
  - shot-review markers for AdvantageScope timelines and video sync
- `org.Griffins1884.frc2026.simulation.maple`
  - Season2026-owned rebuilt field contact geometry, robot profile, Maple arena setup, and terrain helpers
- `org.Griffins1884.frc2026.simulation.drive`
  - Season2026-owned drivetrain simulation facade around the current backing simulator
- `org.Griffins1884.frc2026.simulation.validation`
  - unattended `simulateJava` teleop validation, matched bump-vs-flat validation, per-phase summaries, and trace export

## Migration Boundary

Season2026 now owns the season-specific simulation surfaces:

- `Season2026FieldContactModel`
  - rebuilt-2026 hub, bump, trench, tower, terrain, marker, and planar-contact geometry
  - validation flat-terrain override used by matched bump-vs-flat runs
- `Season2026RobotProfile`
  - chassis footprint and mass properties used by terrain-aware drivetrain simulation
- `Season2026MapleArena`
  - Maple field-border, hub, trench, and tower obstacle setup
- `Season2026DriveSimulation`
  - Season2026-facing facade for drivetrain sim construction, gyro data, module authority, terrain/contact snapshots, support diagnostics, and validation telemetry

The remaining legacy-backed pieces are intentionally isolated:

- `Season2026DriveSimulation` still delegates internally to the current legacy-backed drivetrain implementation.
- Generic `org.griffins1884.sim3d` drivetrain/data contracts are still used internally where the current simulator requires them.
- Gradle still substitutes `org.griffins1884:GriffinSim` to `:compat:legacy-griffinsim`.
- The HALSIM extension is still loaded from the GriffinSim vendordep/runtime path.

Future migration should keep Season2026 callers on these owned boundaries and replace the internals behind them rather than reintroducing legacy imports at call sites.

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

## Sim Validation Path

- Entry command: `./gradlew simulateJava --console=plain`
- Output trace: `build/sim-validation/simulatejava-trace.json`
- Runtime behavior:
  - uses the real Season2026 `simulateJava` path
  - injects deterministic teleop inputs through the existing sim driver-controller mapping
  - records machine-readable samples plus phase audit lines to the console
  - includes flat-floor motion validation plus matched bump-vs-flat drivetrain validation

## Scripted Teleop Validation Sequence

- `idle`
  - zero input, validates stationary stability
- `forward`
  - forward stick command, validates clean forward translation
- `settle_after_forward`
  - zero input, lets the robot coast and settle before the next phase
- `rotate`
  - pure rotation command, validates in-place turning
- `settle_after_rotate`
  - zero input, removes residual rotational carryover
- `strafe`
  - pure lateral command, validates planar lateral response
- `stop`
  - zero input, validates final settle-to-zero behavior

## Phase-Level Checks

- `idle`
  - displacement, heading drift, final linear speed, and final angular speed must stay below limits
- `forward`
  - `+x` displacement must dominate while lateral drift and heading change stay bounded
- `rotate`
  - heading change must dominate while translation stays small
- `strafe`
  - lateral displacement must dominate while `x` drift and heading change stay bounded
- `stop`
  - final linear/angular speed must settle below threshold

## Current Known Limitations

- stop validation checks final settling, not active brake-hold behavior
- startup loop-overrun/logging noise can still appear before the validation phases begin
- the injector is intentionally tied to the current Season2026 sim driver mapping on controller port `0`
- `FieldSimulation/TurretComponentPose3d` can still spin / fail to visually settle; validated shot physics uses the commanded shot target and remains scoring-valid
- drivetrain physics is still legacy-backed internally behind `Season2026DriveSimulation`

## Aggressive Bump Validation

- Rerun command:
  - `./gradlew bumpJumpRegression --console=plain`
- Trace artifact:
  - `build/sim-validation/simulatejava-trace.json`
- Run variants:
  - `medium_speed`
    - deterministic supported-climb check over the bump centerline
  - `high_speed`
    - longer run-up and full command to stress high-speed bump traversal
  - `max_practical`
    - longest practical straight-line run-up through the same bump lane
- Matched flat-ground controls:
  - `high_speed_flat_control`
  - `max_practical_flat_control`
- Matching rules:
  - same start pose
  - same heading
  - same command profile
  - same phase timing
  - terrain flattened only for the control run
- Compared metrics:
  - forward speed
  - forward acceleration
  - translational kinetic energy
  - commanded drive volts
  - drive-authority scale
  - desired vs actual module speed
  - terrain/support state and normal-force summaries
- Comparison windows:
  - bump entry vs flat entry at the same phase-relative offset
  - bump crest vs flat crest-equivalent time
  - bump coast start vs flat coast start
  - later coast-settle samples at the same phase-relative offset
- Accepted result:
  - the bump traversal is physically believable relative to the matched flat-ground control baseline
  - the bump path is not faster than flat at aligned windows
  - drive authority is reduced on the bump rather than boosted
  - post-bump coast remains close to the flat control
  - no remaining evidence of fake braking or fake boost from the bump logic itself
- Regression coverage:
  - `bumpJumpRegression` still validates the scripted bump variants
  - it now also asserts longitudinal sanity envelopes against the matched flat controls so future changes cannot reintroduce implausible bump-specific speed loss or speed gain

## Turning-Over-Bump Validation

- Rerun command:
  - `./gradlew turningBumpRegression --console=plain`
- Purpose:
  - validates turning and diagonal traversal over the bump against matched flat-ground controls
  - checks speed, heading, yaw-rate, module-speed, and drive-authority envelopes at aligned windows
- Accepted result:
  - bump variants interact with bump terrain
  - flat controls do not interact with bump terrain
  - turning behavior remains close to the matched flat-control baseline without fake terrain-specific boost

## Shot Validation

- Rerun command:
  - `./gradlew shotValidation --console=plain`
- Trace artifact:
  - `build/shot-validation/shot-validation-trace.json`
- Purpose:
  - runs unattended shots through the real Season2026 `simulateJavaRelease` path
  - uses the real superstructure/feed release chain and projectile manager
  - validates predicted release state against observed projectile spawn state
  - validates predicted arc against observed live projectile arc
  - validates hub geometry: clear top/cone region first, then descend into the bottom/goal region
- Accepted result:
  - `COMPLETE_PHYSICALLY_BELIEVABLE_AND_SCORING_VALID`

## Current Behavior

- Hub shots use the configured shot model and publish a predicted field-space arc.
- A simulated projectile is spawned on the rising edge of a feed-ready shot command.
- Active projectiles are advanced with the same gravity/drag config as the shot model.
- A shot-only AdvantageScope view can use release, target, impact, arc, and projectile outputs
  without any CAD assets.
- Ferrying still uses the legacy aiming path for commanded setpoints, but the visualization stack
  is structured so ferry trajectories can be added later without reworking the publishers.

## Next Steps

1. Use `FieldSimulation/PredictedShotArc`, `FieldSimulation/ShotReleasePose`,
   `FieldSimulation/PredictedImpactPose`, `FieldSimulation/TargetPose3d`, and
   `FieldSimulation/ActiveProjectiles` as the default review set.
2. Add real robot CAD GLB assets later if you want robot rendering in the same layout.
3. Replace the no-op Maple projectile bridge with actual arena entities once the game-piece API is
   selected.
