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
import org.littletonrobotics.junction.Logger;

public class AutoAlignToPoseCommand extends Command {
  private final ProfiledPIDController driveController;
  private final ProfiledPIDController thetaController;
  private final SwerveSubsystem drive;
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private final double ffMinRadius = 0.0, ffMaxRadius = 0.1;
  private final Pose2d target;
  private final double constraintFactor;
  private final double endVelocity;
  private final double toleranceOverride;

  public AutoAlignToPoseCommand(SwerveSubsystem drive, Pose2d target) {
    this(drive, target, 1.0, 0.0, Double.NaN);
  }

  public AutoAlignToPoseCommand(
      SwerveSubsystem drive,
      Pose2d target,
      double constraintFactor,
      double endVelocity,
      double tolerance) {
    this.drive = drive;
    this.target = target;
    this.constraintFactor = Math.max(0.0, constraintFactor);
    this.endVelocity = endVelocity;
    this.toleranceOverride = tolerance;
    AlignConstants.AlignGains gains = AlignConstants.getAlignGains();
    this.driveController =
        new ProfiledPIDController(
            gains.translationKp(),
            gains.translationKi(),
            gains.translationKd(),
            new TrapezoidProfile.Constraints(
                AlignConstants.ALIGN_MAX_TRANSLATIONAL_SPEED.get() * this.constraintFactor,
                AlignConstants.ALIGN_MAX_TRANSLATIONAL_ACCELERATION.get() * this.constraintFactor),
            AlignConstants.ALIGN_CONTROLLER_LOOP_PERIOD_SEC.get());
    this.thetaController =
        new ProfiledPIDController(
            gains.rotationKp(),
            gains.rotationKi(),
            gains.rotationKd(),
            new TrapezoidProfile.Constraints(
                AlignConstants.ALIGN_MAX_ANGULAR_SPEED.get(),
                AlignConstants.ALIGN_MAX_ANGULAR_ACCELERATION.get()),
            AlignConstants.ALIGN_CONTROLLER_LOOP_PERIOD_SEC.get());
    applyTuning();
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    if (target == null) return;
    applyTuning();

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
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldSpeeds.omegaRadiansPerSecond);

    thetaController.setTolerance(
        Units.degreesToRadians(AlignConstants.ALIGN_ROTATION_TOLERANCE_DEG.get()));
  }

  @Override
  public void execute() {
    if (target == null) {
      return;
    }
    applyTuning();

    Pose2d currentPose = drive.getPose();

    Logger.recordOutput("DriveToPose/currentPose", currentPose);
    Logger.recordOutput("DriveToPose/targetLocation", target.toString());
    Logger.recordOutput("DriveToPose/targetPose", target);

    double currentDistance = currentPose.getTranslation().getDistance(target.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    Logger.recordOutput("DriveToPose/ffScaler", ffScaler);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(
                driveErrorAbs, new TrapezoidProfile.State(0.0, endVelocity));
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    Logger.recordOutput("DriveToPose/DrivePoseError", driveErrorAbs);

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
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return target == null || (driveController.atGoal() && thetaController.atGoal());
  }

  private void applyTuning() {
    AlignConstants.AlignGains gains = AlignConstants.getAlignGains();
    driveController.setPID(gains.translationKp(), gains.translationKi(), gains.translationKd());
    thetaController.setPID(gains.rotationKp(), gains.rotationKi(), gains.rotationKd());
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(
            AlignConstants.ALIGN_MAX_TRANSLATIONAL_SPEED.get() * constraintFactor,
            AlignConstants.ALIGN_MAX_TRANSLATIONAL_ACCELERATION.get() * constraintFactor));
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(
            AlignConstants.ALIGN_MAX_ANGULAR_SPEED.get(),
            AlignConstants.ALIGN_MAX_ANGULAR_ACCELERATION.get()));
    double toleranceMeters =
        Double.isNaN(toleranceOverride)
            ? AlignConstants.ALIGN_TRANSLATION_TOLERANCE_METERS.get()
            : toleranceOverride;
    driveController.setTolerance(toleranceMeters);
  }
}
