# CAD Pipeline

## Goal

Publish a stable AdvantageScope robot asset pipeline without coupling CAD files to the physics
 implementation.

## Expected Output

- `src/main/deploy/advantagescope/robot-assets/robot.glb`
- optional articulated component models such as:
  - `turret.glb`
  - `shooter-pivot.glb`
- `src/main/deploy/advantagescope/robot-assets/config.json`

## Recommended Workflow

1. Export the robot assembly from CAD as STEP.
2. Convert the assembly to GLB with CAD Assistant or an equivalent tool.
3. Export articulated pieces as separate GLBs with the same origin discipline.
4. Update `config.json` so the published component poses in AdvantageScope line up with the CAD
   asset origins.

## Coordinate Rules

- Robot frame: `+X` forward, `+Y` left, `+Z` up
- Turret yaw is relative to robot forward
- Shooter pitch comes from the shot-model launch angle
- The shooter exit point is defined by `ShotModelConfig`

## Validation

- Zeroed robot pose renders on the field in the expected orientation.
- Turret yaw lines up with the logged turret angle.
- Shooter pivot animation lines up with the logged pivot angle.
- The logged shooter exit pose visually matches the muzzle in the asset.
