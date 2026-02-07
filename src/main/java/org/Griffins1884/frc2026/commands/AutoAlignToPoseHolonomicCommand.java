package org.Griffins1884.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Holonomic align-to-pose controller.
 *
 * <p>Compared to {@link AutoAlignToPoseCommand} (1D distance-to-goal), this runs independent X/Y
 * profiled controllers in field space plus a profiled theta controller. This tends to be easier to
 * tune and avoids approach-angle weirdness near the target.
 */
public class AutoAlignToPoseHolonomicCommand extends Command {
  private final SwerveSubsystem drive;
  private final Pose2d target;
  private final double constraintFactor;
  private final double endVelocity;
  private final double toleranceOverride;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  public AutoAlignToPoseHolonomicCommand(SwerveSubsystem drive, Pose2d target) {
    this(drive, target, 1.0, 0.0, Double.NaN);
  }

  public AutoAlignToPoseHolonomicCommand(
      SwerveSubsystem drive,
      Pose2d target,
      double constraintFactor,
      double endVelocity,
      double toleranceMeters) {
    this.drive = drive;
    this.target = target;
    this.constraintFactor = Math.max(0.0, constraintFactor);
    this.endVelocity = endVelocity;
    this.toleranceOverride = toleranceMeters;

    AlignConstants.AlignGains gains = AlignConstants.getAlignGains();
    TrapezoidProfile.Constraints translationConstraints =
        new TrapezoidProfile.Constraints(
            AlignConstants.ALIGN_MAX_TRANSLATIONAL_SPEED.get() * this.constraintFactor,
            AlignConstants.ALIGN_MAX_TRANSLATIONAL_ACCELERATION.get() * this.constraintFactor);
    TrapezoidProfile.Constraints rotationConstraints =
        new TrapezoidProfile.Constraints(
            AlignConstants.ALIGN_MAX_ANGULAR_SPEED.get(),
            AlignConstants.ALIGN_MAX_ANGULAR_ACCELERATION.get());

    xController =
        new ProfiledPIDController(
            gains.translationKp(),
            gains.translationKi(),
            gains.translationKd(),
            translationConstraints,
            AlignConstants.ALIGN_CONTROLLER_LOOP_PERIOD_SEC.get());
    yController =
        new ProfiledPIDController(
            gains.translationKp(),
            gains.translationKi(),
            gains.translationKd(),
            translationConstraints,
            AlignConstants.ALIGN_CONTROLLER_LOOP_PERIOD_SEC.get());
    thetaController =
        new ProfiledPIDController(
            gains.rotationKp(),
            gains.rotationKi(),
            gains.rotationKd(),
            rotationConstraints,
            AlignConstants.ALIGN_CONTROLLER_LOOP_PERIOD_SEC.get());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    if (target == null) return;

    Pose2d currentPose = drive.getPose();
    ChassisSpeeds robotSpeeds = drive.getRobotRelativeSpeeds();
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, currentPose.getRotation());

    // Seed controllers with current state to avoid an initial transient.
    xController.reset(currentPose.getX(), fieldSpeeds.vxMetersPerSecond);
    yController.reset(currentPose.getY(), fieldSpeeds.vyMetersPerSecond);
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldSpeeds.omegaRadiansPerSecond);

    double toleranceMeters =
        Double.isNaN(toleranceOverride)
            ? AlignConstants.ALIGN_TRANSLATION_TOLERANCE_METERS.get()
            : toleranceOverride;
    xController.setTolerance(toleranceMeters);
    yController.setTolerance(toleranceMeters);
    thetaController.setTolerance(Units.degreesToRadians(2.0));
  }

  @Override
  public void execute() {
    if (target == null) {
      return;
    }

    Pose2d currentPose = drive.getPose();
    AlignConstants.AlignGains gains = AlignConstants.getAlignGains();

    Logger.recordOutput("DriveToPoseHolonomic/currentPose", currentPose);
    Logger.recordOutput("DriveToPoseHolonomic/targetPose", target);

    double xError = target.getX() - currentPose.getX();
    double yError = target.getY() - currentPose.getY();
    double dist = Math.hypot(xError, yError);
    Logger.recordOutput("DriveToPoseHolonomic/xErrorMeters", xError);
    Logger.recordOutput("DriveToPoseHolonomic/yErrorMeters", yError);
    Logger.recordOutput("DriveToPoseHolonomic/distanceMeters", dist);

    // Translation feedforward, scaled down near the target.
    double ffScaler = MathUtil.clamp(dist / 0.10, 0.0, 1.0);
    double deadband = gains.feedforwardGains().deadbandMeters();
    double xFF =
        Math.abs(xError) > deadband
            ? gains.feedforwardGains().kV() * xController.getSetpoint().velocity
            : 0.0;
    double yFF =
        Math.abs(yError) > deadband
            ? gains.feedforwardGains().kV() * yController.getSetpoint().velocity
            : 0.0;

    double xVelocity =
        xFF * ffScaler
            + xController.calculate(
                currentPose.getX(), new TrapezoidProfile.State(target.getX(), endVelocity));
    double yVelocity =
        yFF * ffScaler
            + yController.calculate(
                currentPose.getY(), new TrapezoidProfile.State(target.getY(), endVelocity));

    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), target.getRotation().getRadians());

    if (xController.atGoal()) xVelocity = 0.0;
    if (yController.atGoal()) yVelocity = 0.0;
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    // Clamp outputs defensively.
    double maxLinearSpeed = AlignConstants.ALIGN_MAX_TRANSLATIONAL_SPEED.get() * constraintFactor;
    double linearNorm = Math.hypot(xVelocity, yVelocity);
    if (linearNorm > maxLinearSpeed) {
      double scale = maxLinearSpeed / linearNorm;
      xVelocity *= scale;
      yVelocity *= scale;
    }
    thetaVelocity =
        MathUtil.clamp(
            thetaVelocity,
            -AlignConstants.ALIGN_MAX_ANGULAR_SPEED.get(),
            AlignConstants.ALIGN_MAX_ANGULAR_SPEED.get());

    Logger.recordOutput("DriveToPoseHolonomic/xVelocityMps", xVelocity);
    Logger.recordOutput("DriveToPoseHolonomic/yVelocityMps", yVelocity);
    Logger.recordOutput("DriveToPoseHolonomic/thetaVelocityRadPerSec", thetaVelocity);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, yVelocity, thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return target == null
        || (xController.atGoal() && yController.atGoal() && thetaController.atGoal());
  }
}
