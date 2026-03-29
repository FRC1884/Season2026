Place AdvantageScope robot CAD assets here if you want robot rendering in AdvantageScope.

These assets are optional. The default shot-review workflow only needs the logged shot outputs and
does not depend on any robot mesh files.

Expected files:

- `robot.glb`
- optional articulated meshes such as `turret.glb` and `shooter-pivot.glb`
- `config.json`

The runtime publishes these component poses:

- `FieldSimulation/RobotPose3d`
- `FieldSimulation/TurretComponentPose3d`
- `FieldSimulation/ShooterPivotComponentPose3d`
- `FieldSimulation/ShooterExitPose3d`

Keep the asset origin aligned to the robot coordinate frame:

- `+X` forward
- `+Y` left
- `+Z` up
