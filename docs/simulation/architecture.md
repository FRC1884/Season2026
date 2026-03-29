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
  - adapter layer reserved for future arena/game-piece bridging

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
