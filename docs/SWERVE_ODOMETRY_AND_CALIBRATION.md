# Swerve Odometry and Calibration

This document is the team-facing protocol for keeping drivetrain odometry accurate on the real robot.

## What Is Using Odometry

- The shared robot pose comes from `SwerveSubsystem` through `SwerveDrivePoseEstimator`.
- Auto alignment, field-relative driving, turret aiming, and superstructure shot logic all read `drive.getPose()`.
- AprilTag vision feeds corrections into the estimator. Odometry is still the main source between vision updates.

Relevant code:

- [`SwerveSubsystem.java`](../src/main/java/org/Griffins1884/frc2026/subsystems/swerve/SwerveSubsystem.java)
- [`Module.java`](../src/main/java/org/Griffins1884/frc2026/subsystems/swerve/Module.java)
- [`ModuleIOFullKraken.java`](../src/main/java/org/Griffins1884/frc2026/subsystems/swerve/ModuleIOFullKraken.java)
- [`DriveCommands.java`](../src/main/java/org/Griffins1884/frc2026/commands/DriveCommands.java)

## Why Odometry Drifts

Odometry can drift even when the code is working correctly because it is estimating motion from wheel rotation and gyro heading.

Main causes on this robot:

- Wheel radius error. If the effective wheel radius on carpet is not the same as the configured radius, distance accumulates incorrectly.
- Pod zero error. If a module thinks "straight ahead" is slightly rotated, every wheel delta is projected in the wrong direction.
- Wheel slip. Hard acceleration, defense, carpet variation, and scrub all make encoder travel differ from real travel.
- Heading error. A small gyro heading error creates XY drift over distance.
- Steer-drive coupling. On MK5n modules, rotating the pod can induce apparent drive motion in the encoder. This has to be compensated.

## What Was Added

The robot now supports:

- MK5n steer-drive coupling compensation in odometry.
- Persistent saved wheel radius in WPILib `Preferences`.
- Persistent saved zero trims for each module in WPILib `Preferences`.
- Dashboard commands to capture or clear module zero trims without editing code.
- Dashboard commands to measure and save wheel radius without editing code.

The saved values survive code restart and reboot.

## Dashboard Chooser

The commands live in the `Characterization/Diagnostics` chooser published by `RobotContainer`.

Look for entries such as:

- `Drive | Wheel Radius Characterization + Save`
- `Drive | Clear Saved Wheel Radius`
- `Drive | Capture FL Zero Offset`
- `Drive | Capture FR Zero Offset`
- `Drive | Capture BL Zero Offset`
- `Drive | Capture BR Zero Offset`
- `Drive | Clear FL Zero Offset`
- `Drive | Clear FR Zero Offset`
- `Drive | Clear BL Zero Offset`
- `Drive | Clear BR Zero Offset`

Use whatever dashboard you normally use for command choosers.

## Pod Re-Zero Procedure

This is the practical field-safe procedure for re-zeroing pods without changing code.

### Before You Start

- Put the robot on blocks or on a cart so the wheels can rotate safely.
- Disable the robot before physically straightening any wheel.
- Work on one pod at a time.

### How Each Pod Should Be Pointed

The wheel tread should be straight forward relative to the robot frame.

- `FL`: wheel parallel to the left frame rail, pointing toward the robot front.
- `FR`: wheel parallel to the right frame rail, pointing toward the robot front.
- `BL`: wheel parallel to the left frame rail, pointing toward the robot front.
- `BR`: wheel parallel to the right frame rail, pointing toward the robot front.

The important part is that the wheel is aligned with the robot's front-back direction, not sideways.

### Step-By-Step

1. Disable the robot.
2. If that pod may already have a bad saved trim, run the matching `Clear <pod> Zero Offset` command first.
3. Physically point the pod straight ahead.
4. Run the matching `Capture <pod> Zero Offset` command.
5. Repeat for the remaining pods if needed.
6. Restart robot code after finishing all captures.
7. Drive a short straight line and confirm the robot tracks cleanly.

### Important Note

This workflow is intended for trimming a real pod to the correct straight-ahead position. A wheel visually looks the same at `0°` and `180°`, so if a module is completely wrong mechanically, use your current known-good setup as the starting point and make the smallest correction needed.

## Wheel Radius Calibration Procedure

Run this after wheel changes, tread wear, major drivetrain service, or whenever straight-line odometry scale looks wrong.

1. Put the robot on carpet with space around it.
2. Select `Drive | Wheel Radius Characterization + Save`.
3. Let the robot spin in place as the command runs.
4. When it completes or is cancelled, check the logs for the measured radius.
5. If the command was the `+ Save` version, the measured radius is saved automatically.
6. Reboot robot code once before validating the new value.

Use `Drive | Clear Saved Wheel Radius` if you want to go back to the nominal configured value.

## Validation Protocol

After any pod-zero or wheel-radius change:

1. Zero odometry normally.
2. Drive forward 3-5 meters.
3. Stop and confirm the pose heading and translation look reasonable.
4. Rotate 180 degrees and drive back to the original start point.
5. Check whether the estimated pose returns close to the start.
6. Repeat with a diagonal drive and a short auto path if needed.

If drift is still large after correct zeroing and wheel-radius save, investigate:

- bad wheel slip
- damaged module hardware
- gyro heading quality
- overly conservative vision acceptance

## Real-Robot Notes For This Hardware

This robot uses:

- SDS MK5n
- Kraken X60 drive motors
- Kraken X44 turn motors
- molded spike grip wheels
- option 2 drive gearing

The code is currently configured around that hardware, including:

- `KRAKEN_DRIVE_GEAR_RATIO = 6.03`
- `KRAKEN_ROTATOR_GEAR_RATIO = 287.0 / 11.0`
- steer-drive coupling compensation ratio `4.5`

If hardware changes, re-check those assumptions before trusting odometry.
