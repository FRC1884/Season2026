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

package frc.robot.commands;

import static frc.robot.GlobalConstants.FieldMap.*;
import static frc.robot.commands.AlignConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
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
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.RotationalAllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {

  @Setter @Getter public static AlignGains alignGains = DEFAULT_ALIGN_GAINS;

  private DriveCommands() {}

  // --- Alignment context & telemetry helpers (used by autonomy helpers) ---
  private static volatile String alignContext = "";
  private static volatile String alignName = "";

  public static void setAlignContext(String context, String name) {
    alignContext = context == null ? "" : context;
    alignName = name == null ? "" : name;
    try {
      Logger.recordOutput("Autonomy/AlignContext", alignContext);
      Logger.recordOutput("Autonomy/AlignName", alignName);
    } catch (Throwable ignored) {
    }
  }

  public static void clearAlignTelemetry() {
    alignContext = "";
    alignName = "";
    try {
      Logger.recordOutput("Autonomy/AlignContext", "");
      Logger.recordOutput("Autonomy/AlignName", "");
    } catch (Throwable ignored) {
    }
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), ALIGN_MANUAL_DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

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
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), ALIGN_MANUAL_DEADBAND);

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

  public static void robotRelativeChassisSpeedDrive(SwerveSubsystem drive, ChassisSpeeds speeds) {
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(speeds);
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
                ALIGN_MAX_ANGULAR_SPEED, ALIGN_MAX_ANGULAR_ACCELERATION));
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
                Math.abs(speeds.vxMetersPerSecond) / ALIGN_MAX_TRANSLATIONAL_SPEED,
                Math.abs(speeds.vyMetersPerSecond) / ALIGN_MAX_TRANSLATIONAL_SPEED));
    double vx = speeds.vxMetersPerSecond / maxComponent;
    double vy = speeds.vyMetersPerSecond / maxComponent;
    double omega =
        MathUtil.clamp(
            speeds.omegaRadiansPerSecond, -ALIGN_MAX_ANGULAR_SPEED, ALIGN_MAX_ANGULAR_SPEED);
    return new ChassisSpeeds(vx, vy, omega);
  }

  private static Supplier<Pose2d> reefBranchTargetPose(
      Supplier<Pose2d> faceSupplier,
      BooleanSupplier leftAlignSupplier,
      double forwardOffset,
      double lateralOffset,
      double tangentBuffer) {
    return () -> {
      Pose2d facePose = faceSupplier.get();
      double sideSign = leftAlignSupplier.getAsBoolean() ? 1.0 : -1.0;
      double totalLateral = lateralOffset * sideSign + tangentBuffer * sideSign;
      Translation2d branchTransform =
          new Translation2d(-forwardOffset, totalLateral).rotateBy(facePose.getRotation());
      return new Pose2d(facePose.getTranslation().plus(branchTransform), facePose.getRotation());
    };
  }

  /** code for reef alignment */
  public static Command leftAlignToReefCommandTeleop(SwerveSubsystem drive) {
    return alignToReefCommandTeleop(drive, () -> true, () -> 0);
  }

  public static Command rightAlignToReefCommandTeleop(SwerveSubsystem drive) {
    return alignToReefCommandTeleop(drive, () -> false, () -> 0);
  }

  /** helper methods for alignment */

  // align to target face
  public static Command alignToReefCommandAuto(
      SwerveSubsystem drive, BooleanSupplier leftInput, Supplier<Integer> targetReefFace) {
    return Commands.defer(
        () -> {
          // find the coordinates of the selected face
          Supplier<Pose2d> targetFace;
          targetFace =
              switch (targetReefFace.get()) {
                case 1 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_1.getPose();
                case 2 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_2.getPose();
                case 3 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_3.getPose();
                case 4 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_4.getPose();
                case 5 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_5.getPose();
                case 6 -> () -> GlobalConstants.FieldMap.Coordinates.REEF_6.getPose();
                default -> findClosestReefFace(drive);
              };

          double xOffset =
              GlobalConstants.AlignOffsets.BUMPER_TO_CENTER_OFFSET
                  + GlobalConstants.AlignOffsets.REEF_TO_BUMPER_OFFSET;

          BooleanSupplier leftAlign = isFieldRelativeLeftAlign(targetFace, leftInput);
          Supplier<Pose2d> target =
              reefBranchTargetPose(
                  targetFace,
                  leftAlign,
                  xOffset,
                  GlobalConstants.AlignOffsets.REEF_TO_BRANCH_OFFSET,
                  REEF_TANGENT_BUFFER_METERS);

          return new AutoAlignToPoseCommand(drive, target.get());
        },
        Set.of(drive));
  }

  // align to target face
  public static Command alignToReefCommandTeleop(
      SwerveSubsystem drive, BooleanSupplier leftInput, Supplier<Integer> targetReefFace) {
    return Commands.defer(
        () -> {
          // find the coordinates of the selected face
          Supplier<Pose2d> targetFace;
          targetFace =
              switch (targetReefFace.get()) {
                case 1 -> () -> Coordinates.REEF_1.getPose();
                case 2 -> () -> Coordinates.REEF_2.getPose();
                case 3 -> () -> Coordinates.REEF_3.getPose();
                case 4 -> () -> Coordinates.REEF_4.getPose();
                case 5 -> () -> Coordinates.REEF_5.getPose();
                case 6 -> () -> Coordinates.REEF_6.getPose();
                default -> findClosestReefFace(drive);
              };

          double xOffset =
              GlobalConstants.AlignOffsets.BUMPER_TO_CENTER_OFFSET
                  + GlobalConstants.AlignOffsets.REEF_TO_BUMPER_OFFSET;

          BooleanSupplier leftAlign = isFieldRelativeLeftAlign(targetFace, leftInput);
          Supplier<Pose2d> target =
              reefBranchTargetPose(
                  targetFace,
                  leftAlign,
                  xOffset,
                  GlobalConstants.AlignOffsets.REEF_TO_BRANCH_OFFSET,
                  0.0);
          Logger.recordOutput("Autonomy/AlignTargetReef", target.get());
          return new AutoAlignToPoseCommand(drive, target.get());
        },
        Set.of(drive));
  }

  private static BooleanSupplier isFieldRelativeLeftAlign(
      Supplier<Pose2d> targetReefFace, BooleanSupplier leftInput) {
    boolean facingDriver =
        RotationalAllianceFlipUtil.apply(targetReefFace.get()).getRotation().getRadians()
                >= Math.PI / 2
            && RotationalAllianceFlipUtil.apply(targetReefFace.get()).getRotation().getRadians()
                <= 3 * Math.PI / 2;

    Logger.recordOutput("Targets/Facing Driver", facingDriver);

    return () -> facingDriver ? !leftInput.getAsBoolean() : leftInput.getAsBoolean();
  }

  public static Command pathfindThenAlignCommand(SwerveSubsystem drive, Pose2d target, String name) {
      return DriveCommands.pathfindThenAlignCommand(drive, target, name);
  }

  // returns the nearest face of the reef
  public static Supplier<Pose2d> findClosestReefFace(SwerveSubsystem drive) {
    double reef1 =
        drive.getPose().getTranslation().getDistance(Coordinates.REEF_1.getPose().getTranslation());
    double reef2 =
        drive.getPose().getTranslation().getDistance(Coordinates.REEF_2.getPose().getTranslation());
    double reef3 =
        drive.getPose().getTranslation().getDistance(Coordinates.REEF_3.getPose().getTranslation());
    double reef4 =
        drive.getPose().getTranslation().getDistance(Coordinates.REEF_4.getPose().getTranslation());
    double reef5 =
        drive.getPose().getTranslation().getDistance(Coordinates.REEF_5.getPose().getTranslation());
    double reef6 =
        drive.getPose().getTranslation().getDistance(Coordinates.REEF_6.getPose().getTranslation());

    double closestFace =
        Math.min(Math.min(Math.min(reef1, reef2), Math.min(reef3, reef4)), Math.min(reef5, reef6));
    if (reef1 == closestFace) return () -> Coordinates.REEF_1.getPose();
    if (reef2 == closestFace) return () -> Coordinates.REEF_2.getPose();
    if (reef3 == closestFace) return () -> Coordinates.REEF_3.getPose();
    if (reef4 == closestFace) return () -> Coordinates.REEF_4.getPose();
    if (reef5 == closestFace) return () -> Coordinates.REEF_5.getPose();
    return () -> Coordinates.REEF_6.getPose();
  }
  /**
   * Command to align to the nearest coral station
   *
   * @param drive
   * @return
   */
  public static Command alignToNearestCoralStationCommand(
      SwerveSubsystem drive) {
    return Commands.defer(
        () -> {
          Supplier<Boolean> alignLeft = () -> chooseLeftCoralStation(drive);
          Supplier<Pose2d> target =
              () -> {
                boolean leftStation = alignLeft.get();
                Pose2d tagPose =
                    leftStation
                        ? Coordinates.LEFT_CORAL_STATION.getPose()
                        : Coordinates.RIGHT_CORAL_STATION.getPose();
                double sideSign = leftStation ? -1.0 : 1.0;
                return coralStationTargetPose(tagPose, sideSign);
              };

          Logger.recordOutput("Autonomy/AlignToNearestCoralStation", target.get());
          return new AutoAlignToPoseCommand(drive, target.get());
        },
        Set.of(drive));
  }

  /** helper methods for alignment */

  // returns the boolean of the nearest coral station
  private static boolean chooseLeftCoralStation(SwerveSubsystem drive) {
    double leftDistance =
        drive
            .getPose()
            .getTranslation()
            .getDistance(Coordinates.LEFT_CORAL_STATION.getPose().getTranslation());
    double rightDistance =
        drive
            .getPose()
            .getTranslation()
            .getDistance(Coordinates.RIGHT_CORAL_STATION.getPose().getTranslation());
    return leftDistance <= rightDistance;
  }

  private static Pose2d coralStationTargetPose(Pose2d tagPose, double sideSign) {
    Rotation2d approachHeading = tagPose.getRotation().plus(Rotation2d.k180deg);
    double forwardOffset =
        GlobalConstants.AlignOffsets.SOURCE_TO_TAG_STANDOFF
            - GlobalConstants.AlignOffsets.BUMPER_TO_CENTER_OFFSET;
    double lateralOffset = GlobalConstants.AlignOffsets.SIDE_TO_SIDE_OFFSET_AUTO * sideSign;
    Translation2d offsetVector =
        new Translation2d(forwardOffset, lateralOffset).rotateBy(approachHeading);
    return new Pose2d(tagPose.getTranslation().plus(offsetVector), approachHeading);
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
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
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
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(SwerveSubsystem drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
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
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
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
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
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
