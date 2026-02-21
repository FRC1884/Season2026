// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.Griffins1884.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.subsystems.vision.Vision;
import org.Griffins1884.frc2026.util.RobotLogging;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), AlignConstants.ALIGN_MANUAL_DEADBAND.get());
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static void joystickDrive(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Apply rotation deadband
    double omega =
        MathUtil.applyDeadband(
            omegaSupplier.getAsDouble(), AlignConstants.ALIGN_MANUAL_DEADBAND.get());

    // Square rotation value for more precise control
    omega = Math.copySign(omega * omega, omega);

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec());
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      SwerveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            SwerveConstants.ROTATION_CONSTANTS.kP,
            0.0,
            SwerveConstants.ROTATION_CONSTANTS.kD,
            new TrapezoidProfile.Constraints(
                AlignConstants.ALIGN_MAX_ANGULAR_SPEED.get(),
                AlignConstants.ALIGN_MAX_ANGULAR_ACCELERATION.get()));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  private static ChassisSpeeds clampChassisSpeeds(ChassisSpeeds speeds) {
    double maxComponent =
        Math.max(
            1.0,
            Math.max(
                Math.abs(speeds.vxMetersPerSecond)
                    / AlignConstants.ALIGN_MAX_TRANSLATIONAL_SPEED.get(),
                Math.abs(speeds.vyMetersPerSecond)
                    / AlignConstants.ALIGN_MAX_TRANSLATIONAL_SPEED.get()));
    double vx = speeds.vxMetersPerSecond / maxComponent;
    double vy = speeds.vyMetersPerSecond / maxComponent;
    double omega =
        MathUtil.clamp(
            speeds.omegaRadiansPerSecond,
            -AlignConstants.ALIGN_MAX_ANGULAR_SPEED.get(),
            AlignConstants.ALIGN_MAX_ANGULAR_SPEED.get());
    return new ChassisSpeeds(vx, vy, omega);
  }

  public static Command alignToClimbCommand(SwerveSubsystem drive, Vision vision) {
    return Commands.defer(
        () -> {
          // Prefer alliance (when known). If the DS has not provided alliance yet (sim/offline),
          // fall back to picking the nearer tower in the blue-origin field coordinate system.
          boolean isBlue =
              DriverStation.getAlliance().isPresent()
                  ? DriverStation.getAlliance().get() == Alliance.Blue
                  : drive.getPose().getX() < GlobalConstants.FieldConstants.fieldLength / 2.0;

          Translation2d target =
              isBlue
                  ? GlobalConstants.FieldConstants.Tower.centerPoint
                  : GlobalConstants.FieldConstants.Tower.oppCenterPoint;
          int tagId = isBlue ? 31 : 15;

          Rotation2d rotation =
              GlobalConstants.FieldConstants.defaultAprilTagType
                  .getLayout()
                  .getTagPose(tagId)
                  .map(tagPose -> tagPose.getRotation().toRotation2d())
                  // Sensible fallback: point toward the tower along +/-X.
                  .orElse(isBlue ? new Rotation2d() : new Rotation2d(Math.PI));

          Logger.recordOutput("Autonomy/AlignTargetClimb", new Pose2d(target, rotation));
          return new AutoAlignToPoseCommand(drive, new Pose2d(target, rotation))
              .beforeStarting(() -> vision.setExclusiveTagId(tagId))
              .finallyDo(vision::clearExclusiveTagId);
        },
        Set.of(drive));
  }

  public static Command alignToClimbHolonomicCommand(SwerveSubsystem drive, Vision vision) {
    return Commands.defer(
        () -> {
          // Prefer alliance (when known). If the DS has not provided alliance yet (sim/offline),
          // fall back to picking the nearer tower in the blue-origin field coordinate system.
          boolean isBlue =
              DriverStation.getAlliance().isPresent()
                  ? DriverStation.getAlliance().get() == Alliance.Blue
                  : drive.getPose().getX() < GlobalConstants.FieldConstants.fieldLength / 2.0;

          Translation2d target =
              isBlue
                  ? GlobalConstants.FieldConstants.Tower.centerPoint
                  : GlobalConstants.FieldConstants.Tower.oppCenterPoint;

          // Tower wall tags: use the same end-of-field tags that define the tower centerpoint Y.
          int tagId = isBlue ? 31 : 15;

          Rotation2d rotation =
              GlobalConstants.FieldConstants.defaultAprilTagType
                  .getLayout()
                  .getTagPose(tagId)
                  .map(tagPose -> tagPose.getRotation().toRotation2d())
                  // Sensible fallback: point toward the tower along +/-X.
                  .orElse(isBlue ? new Rotation2d() : new Rotation2d(Math.PI));

          Logger.recordOutput("Autonomy/AlignTargetClimb", new Pose2d(target, rotation));
          return new AutoAlignToPoseHolonomicCommand(drive, new Pose2d(target, rotation))
              .beforeStarting(() -> vision.setExclusiveTagId(tagId))
              .finallyDo(vision::clearExclusiveTagId);
        },
        Set.of(drive));
  }

  public static Command alignToAfterCollectStartCommand(SwerveSubsystem drive) {
    return Commands.defer(
        () -> {
          Pose2d target = AlignConstants.getAfterCollectStartPose();
          Logger.recordOutput("Autonomy/AlignTargetAfterCollectStart", target);
          return new AutoAlignToPoseCommand(drive, target);
        },
        Set.of(drive));
  }

  public static Command alignToAfterBumpStartCommand(SwerveSubsystem drive) {
    return Commands.defer(
        () -> {
          Pose2d target = AlignConstants.getAfterOverBumpStartPose();
          Logger.recordOutput("Autonomy/AlignTargetAfterBumpStart", target);
          return new AutoAlignToPoseCommand(drive, target);
        },
        Set.of(drive));
  }

  public static Command alignToAfterSecondBumpCommand(SwerveSubsystem drive) {
    return Commands.defer(
        () -> {
          Pose2d target = AlignConstants.getAfterSecondBumpStartPose();
          Logger.recordOutput("Autonomy/AlignTargetAfterSecondBumpStart", target);
          return new AutoAlignToPoseCommand(drive, target);
        },
        Set.of(drive));
  }

  public static Command alignToAfterBumpToNeutralCommand(SwerveSubsystem drive) {
    return Commands.defer(
        () -> {
          Pose2d target = AlignConstants.getAfterBumpToNeutralStartPose();
          Logger.recordOutput("Autonomy/AlignTargetAfterBumpToNeutralStart", target);
          return new AutoAlignToPoseCommand(drive, target);
        },
        Set.of(drive));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(SwerveSubsystem drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(AlignConstants.FF_START_DELAY.get()),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * AlignConstants.FF_RAMP_RATE.get();
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  RobotLogging.debug("********** Drive FF Characterization Results **********");
                  RobotLogging.debug("\tkS: " + formatter.format(kS));
                  RobotLogging.debug("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(SwerveSubsystem drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(AlignConstants.WHEEL_RADIUS_RAMP_RATE.get());
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(AlignConstants.WHEEL_RADIUS_MAX_VELOCITY.get());
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * SwerveConstants.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      RobotLogging.debug(
                          "********** Wheel Radius Characterization Results **********");
                      RobotLogging.debug(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      RobotLogging.debug(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      RobotLogging.debug(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
