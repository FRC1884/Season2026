package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class AutoAlignToPoseCommand extends Command {
  private final ProfiledPIDController driveController;
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          AlignConstants.DEFAULT_ALIGN_GAINS.rotationKp(),
          AlignConstants.DEFAULT_ALIGN_GAINS.rotationKi(),
          AlignConstants.DEFAULT_ALIGN_GAINS.rotationKd(),
          new TrapezoidProfile.Constraints(
              AlignConstants.ALIGN_MAX_ANGULAR_SPEED,
              AlignConstants.ALIGN_MAX_ANGULAR_ACCELERATION),
          AlignConstants.ALIGN_CONTROLLER_LOOP_PERIOD_SEC);
  private final SwerveSubsystem drive;
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private final double ffMinRadius = 0.0, ffMaxRadius = 0.1;
  private final Pose2d target;

  public AutoAlignToPoseCommand(SwerveSubsystem drive, Pose2d target) {
    this(drive, target, 1.0);
  }

  public AutoAlignToPoseCommand(SwerveSubsystem drive, Pose2d target, double constraintFactor) {
    this.drive = drive;
    this.target = target;
    double clampedFactor = Math.max(0.0, constraintFactor);
    this.driveController =
        new ProfiledPIDController(
            AlignConstants.DEFAULT_ALIGN_GAINS.translationKp(),
            AlignConstants.DEFAULT_ALIGN_GAINS.translationKi(),
            AlignConstants.DEFAULT_ALIGN_GAINS.translationKd(),
            new TrapezoidProfile.Constraints(
                AlignConstants.ALIGN_MAX_TRANSLATIONAL_SPEED * clampedFactor,
                AlignConstants.ALIGN_MAX_TRANSLATIONAL_ACCELERATION * clampedFactor),
            AlignConstants.ALIGN_CONTROLLER_LOOP_PERIOD_SEC);
    this.driveController.setTolerance(1);
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    if (target == null) {
      return;
    }

    Pose2d currentPose = drive.getPose();

    driveController.reset(currentPose.getTranslation().getDistance(target.getTranslation()), 0.0);
    thetaController.reset(currentPose.getRotation().getRadians(), 0.0);
    thetaController.setTolerance(Units.degreesToRadians(2.0));

    driveController.setTolerance(0.04);
  }

  @Override
  public void execute() {
    if (target == null) {
      return;
    }

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
            + driveController.calculate(driveErrorAbs, 0.0);
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
}
