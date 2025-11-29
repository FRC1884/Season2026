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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.RotationalAllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  // driving
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_MAX_VELOCITY = 6.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  // characterization
  private static final double FF_START_DELAY = 0.5; // Secs
  private static final double FF_RAMP_RATE = 0.2; // Volts/Sec

  private static final double ALIGN_TRANSLATION_KP_DEFAULT = 4.5;
  private static final double ALIGN_TRANSLATION_KD_DEFAULT = 1;
  private static final double ALIGN_ROTATION_KP_DEFAULT = 6.0;
  private static final double ALIGN_ROTATION_KD_DEFAULT = 0.0;
  private static final TrapezoidProfile.Constraints ALIGN_ROTATION_CONSTRAINTS =
      new TrapezoidProfile.Constraints(Math.toRadians(540.0), Math.toRadians(900.0));
  private static final double ALIGN_MAX_TRANSLATIONAL_SPEED = 5; // ~5 m/s
  private static final double ALIGN_MAX_ANGULAR_SPEED = Math.toRadians(450.0);
  private static final double ALIGN_TRANSLATION_SLEW_RATE = 4.0; // m/s^2
  private static final double ALIGN_ROTATION_SLEW_RATE = Math.toRadians(720.0); // rad/s^2
  private static final double ALIGN_MANUAL_MAX_SPEED = 5;
  private static final double ALIGN_TRANSLATION_FF_GAIN_DEFAULT =
      3.0; // feedforward m/s per meter of error
  private static final double ALIGN_TRANSLATION_FF_DEADBAND = 0.02; // meters
  private static final double CENTRAL_REEF_RADIUS_METERS = 1.7;
  private static final double CENTRAL_REEF_MARGIN_METERS = 0.4;
  private static final double ALIGN_BYPASS_TIMEOUT_SECS = 1.8;
  private static final double REEF_TANGENT_BUFFER_METERS = 0.0;
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final AlignGains DEFAULT_ALIGN_GAINS =
      new AlignGains(
          ALIGN_TRANSLATION_KP_DEFAULT,
          ALIGN_TRANSLATION_KD_DEFAULT,
          ALIGN_ROTATION_KP_DEFAULT,
          ALIGN_ROTATION_KD_DEFAULT,
          ALIGN_TRANSLATION_FF_GAIN_DEFAULT);
  private static final AtomicReference<AlignGains> ACTIVE_ALIGN_GAINS =
      new AtomicReference<>(DEFAULT_ALIGN_GAINS);
  private static final double ALIGN_TUNER_TIMEOUT_SECS = 3.5;
  private static final double ALIGN_TUNER_KICKBACK_TIME = 0.6;
  private static final double ALIGN_TUNER_KICKBACK_SPEED = 1.5;
  private static final int ALIGN_TUNER_TRIAL_COUNT = 3;
  private static final Translation2d CENTRAL_REEF_CENTER = calculateReefCenter();

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
      // Clear any controller overrides
      PPHolonomicDriveController.clearXYFeedbackOverride();
    } catch (Throwable ignored) {
    }
  }

  public static AlignGains getCurrentAlignGains() {
    return ACTIVE_ALIGN_GAINS.get();
  }

  public static void setAlignGains(AlignGains gains) {
    ACTIVE_ALIGN_GAINS.set(gains);
    Logger.recordOutput(
        "Autonomy/AlignGains",
        new double[] {
          gains.translationKp(), gains.translationKd(), gains.rotationKp(), gains.rotationKd()
        });
    Logger.recordOutput("Autonomy/AlignFeedforward", gains.feedforward());
  }

  public static void resetAlignGains() {
    setAlignGains(DEFAULT_ALIGN_GAINS);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
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
    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

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
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
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

  // use a holonomic controller to align to a target pose like elite FRC teams
  public static Command chasePoseRobotRelativeCommand(
      SwerveSubsystem drive, Supplier<Transform2d> targetOffset) {
    return holonomicAlignCommand(
        drive, () -> drive.getPose().plus(targetOffset.get()), () -> 0.0, false, true);
  }

  /** Command to align the robot to a target while giving the driver tangent control in Y. */
  public static Command chasePoseRobotRelativeCommandYOverride(
      SwerveSubsystem drive, Supplier<Transform2d> targetOffset, DoubleSupplier yDriver) {
    return holonomicAlignCommand(
        drive, () -> drive.getPose().plus(targetOffset.get()), yDriver, true, false);
  }

  /** Runs an aggressive full-field alignment auto-tune across reefs, sources, and processor. */
  public static Command autoTuneAlignGainsFullField(SwerveSubsystem drive) {
    List<AlignTarget> targets =
        List.of(
            new AlignTarget("Reef 1", () -> GlobalConstants.FieldMap.Coordinates.REEF_1.getPose()),
            new AlignTarget("Reef 2", () -> GlobalConstants.FieldMap.Coordinates.REEF_2.getPose()),
            new AlignTarget("Reef 3", () -> GlobalConstants.FieldMap.Coordinates.REEF_3.getPose()),
            new AlignTarget("Reef 4", () -> GlobalConstants.FieldMap.Coordinates.REEF_4.getPose()),
            new AlignTarget("Reef 5", () -> GlobalConstants.FieldMap.Coordinates.REEF_5.getPose()),
            new AlignTarget("Reef 6", () -> GlobalConstants.FieldMap.Coordinates.REEF_6.getPose()),
            new AlignTarget(
                "Left Source",
                () -> GlobalConstants.FieldMap.Coordinates.LEFT_CORAL_STATION.getPose()),
            new AlignTarget(
                "Right Source",
                () -> GlobalConstants.FieldMap.Coordinates.RIGHT_CORAL_STATION.getPose()),
            new AlignTarget(
                "Processor", () -> GlobalConstants.FieldMap.Coordinates.PROCESSOR.getPose()));

    return Commands.defer(
        () -> {
          List<AlignTuningTrial> trials = new ArrayList<>();
          List<Command> iterations = new ArrayList<>();
          for (AlignTarget target : targets) {
            iterations.add(
                runAlignTuningIteration(drive, target.name(), target.poseSupplier(), trials, true));
          }
          Command[] iterationArray = iterations.toArray(new Command[0]);
          return Commands.sequence(
              Commands.runOnce(() -> setAlignContext("AutoTune", "FullField")),
              Commands.runOnce(() -> setAlignGains(DEFAULT_ALIGN_GAINS)),
              Commands.sequence(iterationArray),
              Commands.runOnce(
                  () -> {
                    AlignGains chosen = pickBestAlignGains(trials);
                    setAlignGains(chosen);
                    Logger.recordOutput(
                        "Autonomy/AlignAutoTuner/Recommended",
                        new double[] {
                          chosen.translationKp(),
                          chosen.translationKd(),
                          chosen.rotationKp(),
                          chosen.rotationKd(),
                          chosen.feedforward()
                        });
                  }),
              Commands.runOnce(DriveCommands::clearAlignTelemetry));
        },
        Set.of(drive));
  }

  private static Command holonomicAlignCommand(
      SwerveSubsystem drive,
      Supplier<Pose2d> targetSupplier,
      DoubleSupplier manualTangentSupplier,
      boolean allowManualTangent,
      boolean finishWhenAligned) {
    return holonomicAlignCommand(
        drive,
        targetSupplier,
        manualTangentSupplier,
        allowManualTangent,
        finishWhenAligned,
        () -> ACTIVE_ALIGN_GAINS.get(),
        telemetry -> {});
  }

  private static Command holonomicAlignCommand(
      SwerveSubsystem drive,
      Supplier<Pose2d> targetSupplier,
      DoubleSupplier manualTangentSupplier,
      boolean allowManualTangent,
      boolean finishWhenAligned,
      Supplier<AlignGains> gainsSupplier,
      Consumer<AlignLoopTelemetry> loopTelemetryConsumer) {
    return Commands.defer(
        () -> {
          AlignGains gains = gainsSupplier.get();
          PIDController xController =
              new PIDController(gains.translationKp(), 0.0, gains.translationKd());
          PIDController yController =
              new PIDController(gains.translationKp(), 0.0, gains.translationKd());
          ProfiledPIDController thetaController =
              new ProfiledPIDController(
                  gains.rotationKp(), 0.0, gains.rotationKd(), ALIGN_ROTATION_CONSTRAINTS);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
          HolonomicDriveController holonomicController =
              new HolonomicDriveController(xController, yController, thetaController);
          holonomicController.setTolerance(
              new Pose2d(
                  new Translation2d(0.03, 0.03), new Rotation2d(Units.degreesToRadians(1.5))));

          SlewRateLimiter vxLimiter = new SlewRateLimiter(ALIGN_TRANSLATION_SLEW_RATE);
          SlewRateLimiter vyLimiter = new SlewRateLimiter(ALIGN_TRANSLATION_SLEW_RATE);
          SlewRateLimiter omegaLimiter = new SlewRateLimiter(ALIGN_ROTATION_SLEW_RATE);
          Timer loopTimer = new Timer();

          return new FunctionalCommand(
              () -> {
                thetaController.reset(drive.getRotation().getRadians());
                loopTimer.restart();
              },
              () -> {
                Pose2d currentPose = drive.getPose();
                Pose2d targetPose = targetSupplier.get();

                ChassisSpeeds pidCorrection =
                    holonomicController.calculate(
                        currentPose, targetPose, 0.0, targetPose.getRotation());
                Translation2d feedforwardVelocity =
                    calculateAlignFeedforward(currentPose, targetPose, gains.feedforward());

                ChassisSpeeds fieldRelativeSpeeds =
                    new ChassisSpeeds(
                        feedforwardVelocity.getX() + pidCorrection.vxMetersPerSecond,
                        feedforwardVelocity.getY() + pidCorrection.vyMetersPerSecond,
                        pidCorrection.omegaRadiansPerSecond);

                if (allowManualTangent) {
                  double manualMetersPerSecond =
                      MathUtil.applyDeadband(manualTangentSupplier.getAsDouble(), 0.07)
                          * ALIGN_MANUAL_MAX_SPEED;
                  if (Math.abs(manualMetersPerSecond) > 1e-3) {
                    Translation2d manualVelocity =
                        new Translation2d(0.0, manualMetersPerSecond)
                            .rotateBy(targetPose.getRotation());
                    fieldRelativeSpeeds =
                        new ChassisSpeeds(
                            fieldRelativeSpeeds.vxMetersPerSecond + manualVelocity.getX(),
                            fieldRelativeSpeeds.vyMetersPerSecond + manualVelocity.getY(),
                            fieldRelativeSpeeds.omegaRadiansPerSecond);
                  }
                }

                ChassisSpeeds limited = clampChassisSpeeds(fieldRelativeSpeeds);
                double vx = vxLimiter.calculate(limited.vxMetersPerSecond);
                double vy = vyLimiter.calculate(limited.vyMetersPerSecond);
                double omega = omegaLimiter.calculate(limited.omegaRadiansPerSecond);

                ChassisSpeeds robotRelativeSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drive.getRotation());
                drive.runVelocity(robotRelativeSpeeds);

                Pose2d errorPose = targetPose.relativeTo(currentPose);
                Logger.recordOutput(
                    "Autonomy/AlignErrorMeters",
                    new double[] {
                      errorPose.getX(), errorPose.getY(), errorPose.getRotation().getDegrees()
                    });
                loopTelemetryConsumer.accept(
                    new AlignLoopTelemetry(
                        loopTimer.get(), errorPose, fieldRelativeSpeeds, feedforwardVelocity));
              },
              interrupted -> drive.runVelocity(new ChassisSpeeds()),
              () -> finishWhenAligned && holonomicController.atReference(),
              drive);
        },
        Set.of(drive));
  }

  private static Translation2d calculateAlignFeedforward(
      Pose2d currentPose, Pose2d targetPose, double feedforwardGain) {
    Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
    double distance = toTarget.getNorm();
    if (distance < ALIGN_TRANSLATION_FF_DEADBAND) {
      return new Translation2d();
    }

    double feedforwardSpeed =
        MathUtil.clamp(distance * feedforwardGain, 0.0, ALIGN_MAX_TRANSLATIONAL_SPEED);
    return new Translation2d(feedforwardSpeed, toTarget.getAngle());
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

  public record AlignGains(
      double translationKp,
      double translationKd,
      double rotationKp,
      double rotationKd,
      double feedforward) {
    public AlignGains withTranslationKp(double kp) {
      return new AlignGains(kp, translationKd, rotationKp, rotationKd, feedforward);
    }

    public AlignGains withRotationKp(double kp) {
      return new AlignGains(translationKp, translationKd, kp, rotationKd, feedforward);
    }

    public AlignGains withFeedforward(double ff) {
      return new AlignGains(translationKp, translationKd, rotationKp, rotationKd, ff);
    }
  }

  private record AlignLoopTelemetry(
      double timestampSeconds,
      Pose2d errorPose,
      ChassisSpeeds fieldRelativeSpeeds,
      Translation2d feedforwardVelocity) {}

  private static class AlignTuningTrial {
    String targetName = "";
    AlignGains gainsUsed = DEFAULT_ALIGN_GAINS;
    double initialDistance = 0.0;
    double maxDistance = 0.0;
    double finalDistance = 0.0;
    double maxRotationErrorDeg = 0.0;
    double settleTimeSec = 0.0;
    double maxFieldSpeed = 0.0;
    boolean timedOut = false;
  }

  /** Auto-tunes alignment feedforward and PID by running multiple align attempts. */
  public static Command autoTuneAlignGains(SwerveSubsystem drive, Supplier<Pose2d> targetSupplier) {
    return Commands.defer(
        () -> {
          List<AlignTuningTrial> trials = new ArrayList<>();
          List<Command> iterations = new ArrayList<>();
          for (int i = 0; i < ALIGN_TUNER_TRIAL_COUNT; i++) {
            iterations.add(
                runAlignTuningIteration(drive, "Custom Target", targetSupplier, trials, false));
          }
          Command[] iterationArray = iterations.toArray(new Command[0]);
          return Commands.sequence(
              Commands.runOnce(() -> setAlignContext("AutoTune", "Align")),
              Commands.runOnce(() -> setAlignGains(DEFAULT_ALIGN_GAINS)),
              Commands.sequence(iterationArray),
              Commands.runOnce(
                  () -> {
                    AlignGains chosen = pickBestAlignGains(trials);
                    setAlignGains(chosen);
                    Logger.recordOutput(
                        "Autonomy/AlignAutoTuner/Recommended",
                        new double[] {
                          chosen.translationKp(),
                          chosen.translationKd(),
                          chosen.rotationKp(),
                          chosen.rotationKd(),
                          chosen.feedforward()
                        });
                  }),
              Commands.runOnce(DriveCommands::clearAlignTelemetry));
        },
        Set.of(drive));
  }

  private static Command runAlignTuningIteration(
      SwerveSubsystem drive,
      String targetName,
      Supplier<Pose2d> targetSupplier,
      List<AlignTuningTrial> trials,
      boolean avoidCentralReef) {
    return Commands.defer(
        () -> {
          AlignTuningTrial trial = new AlignTuningTrial();
          trial.targetName = targetName;
          trial.gainsUsed = ACTIVE_ALIGN_GAINS.get();
          Timer timer = new Timer();
          Consumer<AlignLoopTelemetry> telemetryConsumer =
              telemetry -> {
                double distance = telemetry.errorPose().getTranslation().getNorm();
                if (trial.initialDistance < 1e-3) {
                  trial.initialDistance = distance;
                }
                trial.maxDistance = Math.max(trial.maxDistance, distance);
                trial.finalDistance = distance;
                trial.maxRotationErrorDeg =
                    Math.max(
                        trial.maxRotationErrorDeg,
                        Math.abs(telemetry.errorPose().getRotation().getDegrees()));
                ChassisSpeeds speeds = telemetry.fieldRelativeSpeeds();
                double fieldSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                trial.maxFieldSpeed = Math.max(trial.maxFieldSpeed, fieldSpeed);
              };

          Command detour =
              avoidCentralReef ? createAlignTunerBypass(drive, targetSupplier) : Commands.none();

          Command tuneAlign =
              holonomicAlignCommand(
                      drive,
                      targetSupplier,
                      () -> 0.0,
                      false,
                      true,
                      () -> trial.gainsUsed,
                      telemetryConsumer)
                  .beforeStarting(timer::restart);

          Command timeoutFlag =
              Commands.sequence(
                  Commands.waitSeconds(ALIGN_TUNER_TIMEOUT_SECS),
                  Commands.runOnce(() -> trial.timedOut = true));

          return Commands.sequence(
                  createAlignTunerKickback(drive, targetSupplier),
                  detour,
                  Commands.race(tuneAlign, timeoutFlag))
              .finallyDo(
                  () -> {
                    timer.stop();
                    trial.settleTimeSec = timer.get();
                    trials.add(trial);
                    AlignGains updated = refineAlignGains(trial.gainsUsed, trial);
                    setAlignGains(updated);
                    Logger.recordOutput("Autonomy/AlignAutoTuner/LastTarget", trial.targetName);
                    Logger.recordOutput(
                        "Autonomy/AlignAutoTuner/LastTrialMetrics",
                        new double[] {
                          trial.initialDistance,
                          trial.finalDistance,
                          trial.maxFieldSpeed,
                          trial.settleTimeSec,
                          trial.maxRotationErrorDeg,
                          trial.timedOut ? 1.0 : 0.0
                        });
                    Logger.recordOutput(
                        "Autonomy/AlignAutoTuner/LastTrialSec", trial.settleTimeSec);
                  });
        },
        Set.of(drive));
  }

  private static Command createAlignTunerKickback(
      SwerveSubsystem drive, Supplier<Pose2d> targetSupplier) {
    return Commands.sequence(
        Commands.deadline(
            Commands.waitSeconds(ALIGN_TUNER_KICKBACK_TIME),
            Commands.run(
                () -> {
                  Pose2d target = targetSupplier.get();
                  Rotation2d awayHeading = target.getRotation().plus(Rotation2d.k180deg);
                  Translation2d retreatVector =
                      new Translation2d(ALIGN_TUNER_KICKBACK_SPEED, 0.0).rotateBy(awayHeading);
                  ChassisSpeeds speeds =
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          retreatVector.getX(), retreatVector.getY(), 0.0, drive.getRotation());
                  drive.runVelocity(speeds);
                },
                drive)),
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds())));
  }

  private static Command createAlignTunerBypass(
      SwerveSubsystem drive, Supplier<Pose2d> targetSupplier) {
    return Commands.defer(
        () -> {
          Pose2d start = drive.getPose();
          Pose2d target = targetSupplier.get();
          Pose2d waypoint = calculateReefBypassWaypoint(start, target);
          if (waypoint == null) {
            return Commands.none();
          }
          Supplier<Pose2d> waypointSupplier = () -> waypoint;
          return holonomicAlignCommand(drive, waypointSupplier, () -> 0.0, false, true)
              .withTimeout(ALIGN_BYPASS_TIMEOUT_SECS);
        },
        Set.of(drive));
  }

  private static AlignGains refineAlignGains(AlignGains current, AlignTuningTrial trial) {
    AlignGains updated = current;
    if (trial.timedOut) {
      updated = updated.withFeedforward(current.feedforward() * 1.2);
      updated = updated.withTranslationKp(current.translationKp() * 1.1);
      return clampAlignGains(updated, current.translationKd(), current.rotationKd());
    }

    double speedTarget = ALIGN_MAX_TRANSLATIONAL_SPEED * 0.9;
    if (trial.maxFieldSpeed < speedTarget) {
      updated = updated.withFeedforward(current.feedforward() * 1.12);
    }

    if (trial.settleTimeSec > 1.2 || trial.finalDistance > 0.03) {
      updated = updated.withTranslationKp(current.translationKp() * 1.08);
    } else if (trial.maxDistance > trial.initialDistance * 1.15) {
      updated = updated.withTranslationKp(current.translationKp() * 0.92);
    }

    if (trial.maxRotationErrorDeg > 7.0) {
      updated = updated.withRotationKp(current.rotationKp() * 1.05);
    }

    return clampAlignGains(updated, current.translationKd(), current.rotationKd());
  }

  private static AlignGains clampAlignGains(
      AlignGains gains, double translationKd, double rotationKd) {
    double ff = MathUtil.clamp(gains.feedforward(), 0.5, 7.0);
    double transKp = MathUtil.clamp(gains.translationKp(), 1.0, 10.0);
    double rotKp = MathUtil.clamp(gains.rotationKp(), 2.0, 12.0);
    return new AlignGains(transKp, translationKd, rotKp, rotationKd, ff);
  }

  private static AlignGains pickBestAlignGains(List<AlignTuningTrial> trials) {
    return trials.stream()
        .filter(trial -> !trial.timedOut)
        .min(Comparator.comparingDouble(trial -> trial.settleTimeSec + trial.finalDistance * 5))
        .map(trial -> trial.gainsUsed)
        .orElse(ACTIVE_ALIGN_GAINS.get());
  }

  private static Pose2d calculateReefBypassWaypoint(Pose2d start, Pose2d target) {
    if (!pathCrossesCentralReef(start, target)) {
      return null;
    }
    Translation2d startT = start.getTranslation();
    Translation2d targetT = target.getTranslation();
    Translation2d delta = targetT.minus(startT);
    double length = delta.getNorm();
    if (length < 1e-3) {
      return null;
    }
    Translation2d direction = new Translation2d(delta.getX() / length, delta.getY() / length);
    Translation2d normal = new Translation2d(-direction.getY(), direction.getX());
    Translation2d mid = startT.plus(delta.times(0.5));
    double clearance = CENTRAL_REEF_RADIUS_METERS + CENTRAL_REEF_MARGIN_METERS;
    Translation2d candidateA = mid.plus(normal.times(clearance));
    Translation2d candidateB = mid.minus(normal.times(clearance));
    double distA = candidateA.getDistance(CENTRAL_REEF_CENTER);
    double distB = candidateB.getDistance(CENTRAL_REEF_CENTER);
    Translation2d chosen = distA > distB ? candidateA : candidateB;
    return new Pose2d(chosen, target.getRotation());
  }

  private static boolean pathCrossesCentralReef(Pose2d start, Pose2d target) {
    double distance =
        distanceFromPointToSegment(
            CENTRAL_REEF_CENTER, start.getTranslation(), target.getTranslation());
    return distance < CENTRAL_REEF_RADIUS_METERS + CENTRAL_REEF_MARGIN_METERS;
  }

  private static double distanceFromPointToSegment(
      Translation2d point, Translation2d segmentStart, Translation2d segmentEnd) {
    Translation2d seg = segmentEnd.minus(segmentStart);
    double segLengthSquared = Math.pow(seg.getNorm(), 2);
    if (segLengthSquared < 1e-6) {
      return point.getDistance(segmentStart);
    }
    Translation2d startToPoint = point.minus(segmentStart);
    double t =
        (startToPoint.getX() * seg.getX() + startToPoint.getY() * seg.getY()) / segLengthSquared;
    t = MathUtil.clamp(t, 0.0, 1.0);
    Translation2d projection =
        new Translation2d(
            segmentStart.getX() + t * seg.getX(), segmentStart.getY() + t * seg.getY());
    return point.getDistance(projection);
  }

  private static Translation2d calculateReefCenter() {
    Pose2d[] faces =
        new Pose2d[] {
          GlobalConstants.FieldMap.Coordinates.REEF_1.getPose(),
          GlobalConstants.FieldMap.Coordinates.REEF_2.getPose(),
          GlobalConstants.FieldMap.Coordinates.REEF_3.getPose(),
          GlobalConstants.FieldMap.Coordinates.REEF_4.getPose(),
          GlobalConstants.FieldMap.Coordinates.REEF_5.getPose(),
          GlobalConstants.FieldMap.Coordinates.REEF_6.getPose()
        };
    double x = 0.0;
    double y = 0.0;
    for (Pose2d pose : faces) {
      x += pose.getX();
      y += pose.getY();
    }
    return new Translation2d(x / faces.length, y / faces.length);
  }

  private record AlignTarget(String name, Supplier<Pose2d> poseSupplier) {}

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
          new Translation2d(forwardOffset, totalLateral).rotateBy(facePose.getRotation());
      return new Pose2d(facePose.getTranslation().plus(branchTransform), facePose.getRotation());
    };
  }

  // run the pathfind then follow command and then use PID to align on termination
  public static Command pathfindThenPIDCommand(SwerveSubsystem drive, Supplier<Pose2d> target) {
    PathConstraints constraints = new PathConstraints(0.5, 1, 0.5, 0.5);

    double endVelocity = 0.0;

    return Commands.sequence(
        AutoBuilder.pathfindToPose(target.get(), constraints, endVelocity),
        holonomicAlignCommand(drive, target, () -> 0.0, false, true));
  }

  // Overload that accepts a context string; delegates to the base implementation
  public static Command pathfindThenPIDCommand(
      SwerveSubsystem drive, Supplier<Pose2d> target, String context) {
    return pathfindThenPIDCommand(drive, target);
  }

  /** Updates the path to override for the coral offset */
  public static Command overridePathplannerCoralOffset(DoubleSupplier offset) {
    return Commands.run(
        () ->
            PPHolonomicDriveController.overrideYFeedback(
                () -> {
                  // Calculate feedback from your custom PID controller
                  return offset.getAsDouble();
                }));
  }

  /** clears all x and y overrides */
  public static Command clearXYOverrides() {
    return Commands.run(() -> PPHolonomicDriveController.clearXYFeedbackOverride());
  }

  /** align to processor */
  public static Command alignToProcessorCommand() {
    PathConstraints constraints = new PathConstraints(10, 5, 5, 5);
    Pose2d tagPose = Coordinates.PROCESSOR.getPose();
    Rotation2d tagHeading = tagPose.getRotation(); // tags face into the field
    double standoffMeters =
        Math.abs(GlobalConstants.AlignOffsets.BUMPER_TO_CENTER_OFFSET)
            + GlobalConstants.AlignOffsets.SOURCE_TO_TAG_STANDOFF
            + Units.inchesToMeters(4); // small buffer away from the wall

    // Place the robot in front of the processor (toward the field) and face the tag
    Translation2d offsetVector = new Translation2d(-standoffMeters, 0.0).rotateBy(tagHeading);
    Pose2d target =
        new Pose2d(
            tagPose.getTranslation().plus(offsetVector), tagHeading.plus(Rotation2d.k180deg));
    Logger.recordOutput("Targets/Processor", target);
    return AutoBuilder.pathfindToPose(target, constraints);
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

          return holonomicAlignCommand(drive, target, () -> 0.0, false, true);
        },
        Set.of(drive));
  }

  // Convenience wrapper: faceIndex in [1..6], branchOffsetIndex: -1 for left (A), +1 for right (B)
  public static Command alignToReefBranchCommandAuto(
      SwerveSubsystem drive, int faceIndex, int branchOffsetIndex) {
    BooleanSupplier left = () -> branchOffsetIndex < 0;
    Supplier<Integer> face = () -> Math.max(1, Math.min(6, faceIndex));
    return alignToReefCommandAuto(drive, left, face);
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

          return holonomicAlignCommand(drive, target, () -> 0.0, false, false);
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
      SwerveSubsystem drive, DoubleSupplier yDriver) {
    return Commands.defer(
        () -> {
          Supplier<Boolean> alignLeft = () -> chooseLeftCoralStation(drive);
          DoubleSupplier driverOverride = () -> yDriver.getAsDouble() * (alignLeft.get() ? -1 : 1);
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

          return holonomicAlignCommand(drive, target, driverOverride, true, false);
        },
        Set.of(drive));
  }

  public static Command alignToNearestCoralStationCommandAuto(SwerveSubsystem drive) {
    return Commands.defer(
        () -> {
          Supplier<Pose2d> target =
              () -> {
                boolean leftStation = chooseLeftCoralStation(drive);
                Pose2d base =
                    leftStation
                        ? Coordinates.LEFT_CORAL_STATION.getPose()
                        : Coordinates.RIGHT_CORAL_STATION.getPose();
                double sideSign = leftStation ? -1.0 : 1.0;
                return coralStationTargetPose(base, sideSign);
              };

          return holonomicAlignCommand(drive, target, () -> 0.0, false, true);
        },
        Set.of(drive));
  }

  public static Command alignToCoralStationCommandAuto(SwerveSubsystem drive, boolean leftStation) {
    return Commands.defer(
        () -> {
          Supplier<Pose2d> target =
              () -> {
                Pose2d base =
                    leftStation
                        ? Coordinates.LEFT_CORAL_STATION.getPose()
                        : Coordinates.RIGHT_CORAL_STATION.getPose();
                double sideSign = leftStation ? -1.0 : 1.0;
                return coralStationTargetPose(base, sideSign);
              };

          return holonomicAlignCommand(drive, target, () -> 0.0, false, true);
        },
        Set.of(drive));
  }

  /** helper methods for alignment */

  // returns the coordinates of the nearest coral station
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
