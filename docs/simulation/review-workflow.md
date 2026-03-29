# Shot Review Workflow

## Inputs

- robot log with `FieldSimulation/*` and `ShotReview/*` outputs
- a fixed-camera practice video recorded during the same session

## AdvantageScope Setup

1. Open the log.
2. Load the review layout from `src/main/deploy/advantagescope/layouts/shot-only-review.json`.
3. Attach the matching video file in the video tab.
4. Align the video with the `ShotReview/LastReleaseTimestampSec` markers and mechanism telemetry.

## Panels To Use

- 3D field showing robot component poses and projectile trajectories
- 3D field showing:
  - `FieldSimulation/PredictedShotArc`
  - `FieldSimulation/ShotReleasePose`
  - `FieldSimulation/PredictedImpactPose`
  - `FieldSimulation/TargetPose3d`
  - `FieldSimulation/ActiveProjectiles`
- video panel for the real shot
- line graphs for:
  - shooter RPM
  - shooter pivot position
  - turret angle
  - shot prediction error
  - projectile count

## Review Questions

- Did the release pose line up with the real shot frame?
- Did the predicted arc match the real apex and arrival timing?
- Did the miss direction match the real miss direction?
- Was the release event early or late relative to feeder motion?
