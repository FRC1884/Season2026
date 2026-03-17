package org.Griffins1884.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.util.AllianceFlipUtil;
import org.Griffins1884.frc2026.util.RobotLogging;
import org.littletonrobotics.junction.Logger;

public class AutoAlignToPoseCommand extends Command {
  private final ProfiledPIDController driveController;
  private final ProfiledPIDController thetaController;
  private final SwerveSubsystem drive;
  private final int tuningId = System.identityHashCode(this);
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private final double ffMinRadius = 0.0, ffMaxRadius = 0.1;
  private final Pose2d target;
  private final double constraintFactor;
  private final double endVelocity;
  private final double toleranceOverride;
  private final boolean stopOnEnd;

  public AutoAlignToPoseCommand(SwerveSubsystem drive, Pose2d target) {
    this(drive, target, 1.0, 0.0, 0.1, true, true);
  }

  public AutoAlignToPoseCommand(
      SwerveSubsystem drive,
      Pose2d target,
      double constraintFactor,
      double endVelocity,
      double tolerance) {
    this(drive, target, constraintFactor, endVelocity, tolerance, true, true);
  }

  public AutoAlignToPoseCommand(
      SwerveSubsystem drive,
      Pose2d target,
      double constraintFactor,
      double endVelocity,
      double tolerance,
      boolean targetInBlueFrame) {
    this(drive, target, constraintFactor, endVelocity, tolerance, targetInBlueFrame, true);
  }

  public AutoAlignToPoseCommand(
      SwerveSubsystem drive,
      Pose2d target,
      double constraintFactor,
      double endVelocity,
      double tolerance,
      boolean targetInBlueFrame,
      boolean stopOnEnd) {
    this.drive = drive;
    this.target =
        target == null ? null : (targetInBlueFrame ? AllianceFlipUtil.apply(target) : target);
    this.constraintFactor = Math.max(0.0, constraintFactor);
    this.endVelocity = endVelocity;
    this.toleranceOverride = tolerance;
    this.stopOnEnd = stopOnEnd;
    this.driveController =
        new ProfiledPIDController(
            AlignConstants.Auto.TRANSLATION_GAINS.kP().get(),
            AlignConstants.Auto.TRANSLATION_GAINS.kI().get(),
            AlignConstants.Auto.TRANSLATION_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                AlignConstants.Auto.MAX_LINEAR_SPEED_MPS.get() * this.constraintFactor,
                AlignConstants.Auto.MAX_LINEAR_ACCEL_MPS2.get() * this.constraintFactor),
            AlignConstants.LOOP_PERIOD_SEC);
    this.thetaController =
        new ProfiledPIDController(
            AlignConstants.Auto.ROTATION_GAINS.kP().get(),
            AlignConstants.Auto.ROTATION_GAINS.kI().get(),
            AlignConstants.Auto.ROTATION_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                AlignConstants.Auto.MAX_ANGULAR_SPEED_RAD_PER_SEC.get(),
                AlignConstants.Auto.MAX_ANGULAR_ACCEL_RAD_PER_SEC2.get()),
            AlignConstants.LOOP_PERIOD_SEC);
    applyTuning();
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    if (target == null) return;
    updateTuningIfChanged(true);

    Pose2d currentPose = drive.getPose();

    // Vector from robot to target
    Translation2d toTarget = target.getTranslation().minus(currentPose.getTranslation());
    double distance = toTarget.getNorm();

    // Get robot-relative speeds and convert to field-relative
    ChassisSpeeds robotSpeeds = drive.getRobotRelativeSpeeds();
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, currentPose.getRotation());

    // Unit vector pointing from robot → target
    double ux = (distance > 1e-6) ? toTarget.getX() / distance : 0.0;
    double uy = (distance > 1e-6) ? toTarget.getY() / distance : 0.0;

    // Project current velocity onto the robot→target direction
    // Positive = moving toward target
    double velocityTowardTarget =
        fieldSpeeds.vxMetersPerSecond * ux + fieldSpeeds.vyMetersPerSecond * uy;

    // Distance decreases as we move toward target → derivative is negative
    double distanceRate = -velocityTowardTarget;

    // Seed the profiled controllers with current state
    driveController.reset(distance, distanceRate);
    double toleranceMeters =
        Double.isNaN(toleranceOverride)
            ? AlignConstants.Auto.TRANSLATION_TOLERANCE_METERS.get()
            : toleranceOverride;
    driveController.setTolerance(toleranceMeters);

    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldSpeeds.omegaRadiansPerSecond);

    thetaController.setTolerance(
        Units.degreesToRadians(AlignConstants.Auto.ROTATION_TOLERANCE_DEG.get()));
  }

  @Override
  public void execute() {
    if (target == null) {
      return;
    }
    updateTuningIfChanged(false);

    Pose2d currentPose = drive.getPose();

    if (RobotLogging.isDebugMode()) {
      Logger.recordOutput("DriveToPose/currentPose", currentPose);
      Logger.recordOutput("DriveToPose/targetLocation", target.toString());
      Logger.recordOutput("DriveToPose/targetPose", target);
    }

    double currentDistance = currentPose.getTranslation().getDistance(target.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    if (RobotLogging.isDebugMode()) {
      Logger.recordOutput("DriveToPose/ffScaler", ffScaler);
    }

    double driveFFVelocity =
        currentDistance > AlignConstants.Auto.FEEDFORWARD_DEADBAND_METERS.get()
            ? AlignConstants.Auto.FEEDFORWARD_KV.get() * driveController.getSetpoint().velocity
            : 0.0;
    if (RobotLogging.isDebugMode()) {
      Logger.recordOutput("DriveToPose/DriveFFVelocity", driveFFVelocity);
    }

    double driveVelocityScalar =
        driveFFVelocity * ffScaler
            + driveController.calculate(
                driveErrorAbs, new TrapezoidProfile.State(0.0, endVelocity));
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    if (RobotLogging.isDebugMode()) {
      Logger.recordOutput("DriveToPose/DrivePoseError", driveErrorAbs);
    }

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), target.getRotation().getRadians());
    thetaErrorAbs = Math.abs(currentPose.getRotation().minus(target.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // Command speeds
    var driveVelocity =
        new Translation2d(driveVelocityScalar, 0.0)
            .rotateBy(currentPose.getTranslation().minus(target.getTranslation()).getAngle());
    double maxLinearSpeed = AlignConstants.Auto.MAX_LINEAR_SPEED_MPS.get() * constraintFactor;
    if (driveVelocity.getNorm() > maxLinearSpeed) {
      driveVelocity = driveVelocity.times(maxLinearSpeed / driveVelocity.getNorm());
    }
    thetaVelocity =
        MathUtil.clamp(
            thetaVelocity,
            -AlignConstants.Auto.MAX_ANGULAR_SPEED_RAD_PER_SEC.get(),
            AlignConstants.Auto.MAX_ANGULAR_SPEED_RAD_PER_SEC.get());
    if (RobotLogging.isDebugMode()) {
      Logger.recordOutput("DriveToPose/DriveVelocitySetpoint", driveVelocity);
      Logger.recordOutput("DriveToPose/ThetaVelocitySetpointRadPerSec", thetaVelocity);
    }
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    if (stopOnEnd) {
      drive.runVelocity(new ChassisSpeeds());
    }
  }

  @Override
  public boolean isFinished() {
    return target == null || (driveController.atGoal() && thetaController.atGoal());
  }

  private void updateTuningIfChanged(boolean force) {
    // Apply new constants only when tunables change to avoid unnecessary allocations and
    // controller churn on the roboRIO.
    boolean changed =
        AlignConstants.Auto.TRANSLATION_GAINS.kP().hasChanged(tuningId)
            || AlignConstants.Auto.TRANSLATION_GAINS.kI().hasChanged(tuningId)
            || AlignConstants.Auto.TRANSLATION_GAINS.kD().hasChanged(tuningId)
            || AlignConstants.Auto.ROTATION_GAINS.kP().hasChanged(tuningId)
            || AlignConstants.Auto.ROTATION_GAINS.kI().hasChanged(tuningId)
            || AlignConstants.Auto.ROTATION_GAINS.kD().hasChanged(tuningId)
            || AlignConstants.Auto.FEEDFORWARD_KV.hasChanged(tuningId)
            || AlignConstants.Auto.FEEDFORWARD_DEADBAND_METERS.hasChanged(tuningId)
            || AlignConstants.Auto.MAX_LINEAR_SPEED_MPS.hasChanged(tuningId)
            || AlignConstants.Auto.MAX_LINEAR_ACCEL_MPS2.hasChanged(tuningId)
            || AlignConstants.Auto.MAX_ANGULAR_SPEED_RAD_PER_SEC.hasChanged(tuningId)
            || AlignConstants.Auto.MAX_ANGULAR_ACCEL_RAD_PER_SEC2.hasChanged(tuningId)
            || AlignConstants.Auto.TRANSLATION_TOLERANCE_METERS.hasChanged(tuningId)
            || AlignConstants.Auto.ROTATION_TOLERANCE_DEG.hasChanged(tuningId);
    if (force || changed) {
      applyTuning();
    }
  }

  private void applyTuning() {
    driveController.setPID(
        AlignConstants.Auto.TRANSLATION_GAINS.kP().get(),
        AlignConstants.Auto.TRANSLATION_GAINS.kI().get(),
        AlignConstants.Auto.TRANSLATION_GAINS.kD().get());
    thetaController.setPID(
        AlignConstants.Auto.ROTATION_GAINS.kP().get(),
        AlignConstants.Auto.ROTATION_GAINS.kI().get(),
        AlignConstants.Auto.ROTATION_GAINS.kD().get());
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(
            AlignConstants.Auto.MAX_LINEAR_SPEED_MPS.get() * constraintFactor,
            AlignConstants.Auto.MAX_LINEAR_ACCEL_MPS2.get() * constraintFactor));
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(
            AlignConstants.Auto.MAX_ANGULAR_SPEED_RAD_PER_SEC.get(),
            AlignConstants.Auto.MAX_ANGULAR_ACCEL_RAD_PER_SEC2.get()));
    double toleranceMeters =
        Double.isNaN(toleranceOverride)
            ? AlignConstants.Auto.TRANSLATION_TOLERANCE_METERS.get()
            : toleranceOverride;
    driveController.setTolerance(toleranceMeters);
    thetaController.setTolerance(
        Units.degreesToRadians(AlignConstants.Auto.ROTATION_TOLERANCE_DEG.get()));
  }
}
