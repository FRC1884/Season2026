# Vision Pose Usage

This document explains how AprilTag vision is currently used for robot pose estimation and how that pose is consumed elsewhere in the codebase.

## High-Level Flow

The pose pipeline is:

`AprilTag cameras -> Vision subsystem -> SwerveSubsystem pose estimator -> drive.getPose() consumers`

Relevant code:

- [`Vision.java`](../src/main/java/org/Griffins1884/frc2026/subsystems/vision/Vision.java)
- [`AprilTagVisionIOLimelight.java`](../src/main/java/org/Griffins1884/frc2026/subsystems/vision/AprilTagVisionIOLimelight.java)
- [`SwerveSubsystem.java`](../src/main/java/org/Griffins1884/frc2026/subsystems/swerve/SwerveSubsystem.java)
- [`RobotContainer.java`](../src/main/java/org/Griffins1884/frc2026/RobotContainer.java)

## What The Cameras Do

Each AprilTag camera:

- updates Limelight inputs
- reads MegaTag2 pose estimates
- reports tag IDs, timestamp, quality, and pose
- provides the data to the `Vision` subsystem

The game-piece targeting Limelight is separate from field-pose estimation.

## How Vision Is Accepted

The current AprilTag pose policy is intentionally WildStang-like in camera selection, but still uses the better WPILib estimator handoff.

### Current Behavior

- All configured AprilTag cameras are polled.
- Invalid estimates are rejected first.
- Only one camera is selected as the best camera each cycle.
- The best camera is chosen primarily by closest visible fiducial distance.
- A reference-style camera FOM is compared against an odometry FOM.
- If vision wins, that measurement is sent into `SwerveDrivePoseEstimator.addVisionMeasurement(...)`.

### Important Detail

This is not a hard pose snap. The robot still uses the estimator path in `SwerveSubsystem`.

That means:

- odometry remains the continuous backbone
- vision corrects the estimator
- one camera is used at a time
- multiple cameras are not fused together in the final selection stage

## Vision Rejection Rules

Vision can be rejected for several reasons, including:

- disconnected camera
- no valid MegaTag pose
- pose outside field bounds
- stale timestamp
- out-of-order timestamp
- low quality on single-tag MegaTag2
- excessive yaw rate
- excessive residual or outlier detection
- reset suppression after odometry reset
- odometry being preferred by the FOM gate

## Odometry Reset Interaction

After an odometry reset, vision is temporarily suppressed for a short window before being accepted again.

This is important because it prevents stale pre-reset frames from pulling the estimator to a bad pose immediately after a reset.

## What Uses The Fused Pose

The shared robot pose comes from `drive.getPose()` in `SwerveSubsystem`.

Major consumers include:

- field-relative drive commands
- auto alignment commands
- turret aiming
- superstructure shot solution logic

Examples:

- [`DriveCommands.java`](../src/main/java/org/Griffins1884/frc2026/commands/DriveCommands.java)
- [`AutoAlignToPoseCommand.java`](../src/main/java/org/Griffins1884/frc2026/commands/AutoAlignToPoseCommand.java)
- [`TurretCommands.java`](../src/main/java/org/Griffins1884/frc2026/commands/TurretCommands.java)
- [`Superstructure.java`](../src/main/java/org/Griffins1884/frc2026/subsystems/Superstructure.java)

## What Does Not Use This Pose

The game-piece alignment path is separate and does not use the AprilTag field-pose estimator.

That path is for direct targeting data such as `tx` and `tv`, not for fused field localization.

## Practical Notes

- If odometry drift appears large, vision acceptance frequency matters.
- If vision is too conservative, drift can remain visible longer before correction.
- If pods are not zeroed or wheel radius is off, vision may look inconsistent even when the real issue is drivetrain calibration.

For drivetrain calibration and drift-control procedures, see [`SWERVE_ODOMETRY_AND_CALIBRATION.md`](./SWERVE_ODOMETRY_AND_CALIBRATION.md).
