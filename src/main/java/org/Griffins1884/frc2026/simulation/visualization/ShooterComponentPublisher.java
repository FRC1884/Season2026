package org.Griffins1884.frc2026.simulation.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.Griffins1884.frc2026.simulation.shooter.ShotSimulationConfig;

/** Builds field-space shooter component poses from turret yaw and pivot state. */
public final class ShooterComponentPublisher {
  private ShooterComponentPublisher() {}

  public static Pose3d createPivotPose(
      Pose2d robotPose,
      Rotation2d turretYaw,
      double pivotMotorRotations,
      ShotSimulationConfig config) {
    Translation3d pivotRoot = config != null ? config.turretMountMeters() : new Translation3d();
    double launchAngleDegrees =
        config != null ? config.launchAngleDegrees(pivotMotorRotations) : 0.0;
    return new Pose3d(
        TurretComponentPublisher.rotateIntoField(robotPose, pivotRoot),
        new Rotation3d(
            0.0,
            -Math.toRadians(launchAngleDegrees),
            robotPose.getRotation().plus(turretYaw).getRadians()));
  }

  public static Pose3d createExitPose(
      Pose2d robotPose,
      Rotation2d turretYaw,
      double pivotMotorRotations,
      ShotSimulationConfig config) {
    Translation3d exitPosition =
        config != null
            ? config.shooterExitPositionMeters(pivotMotorRotations, turretYaw)
            : new Translation3d();
    double launchAngleDegrees =
        config != null ? config.launchAngleDegrees(pivotMotorRotations) : 0.0;
    return new Pose3d(
        TurretComponentPublisher.rotateIntoField(robotPose, exitPosition),
        new Rotation3d(
            0.0,
            -Math.toRadians(launchAngleDegrees),
            robotPose.getRotation().plus(turretYaw).getRadians()));
  }

  public static Pose3d createPivotPose(
      Pose3d robotPose,
      Rotation2d turretYaw,
      double pivotMotorRotations,
      ShotSimulationConfig config) {
    Translation3d pivotRoot = config != null ? config.turretMountMeters() : new Translation3d();
    double launchAngleDegrees =
        config != null ? config.launchAngleDegrees(pivotMotorRotations) : 0.0;
    return robotPose.transformBy(
        new Transform3d(
            pivotRoot,
            new Rotation3d(0.0, -Math.toRadians(launchAngleDegrees), turretYaw.getRadians())));
  }

  public static Pose3d createExitPose(
      Pose3d robotPose,
      Rotation2d turretYaw,
      double pivotMotorRotations,
      ShotSimulationConfig config) {
    Translation3d exitPosition =
        config != null
            ? config.shooterExitPositionMeters(pivotMotorRotations, turretYaw)
            : new Translation3d();
    double launchAngleDegrees =
        config != null ? config.launchAngleDegrees(pivotMotorRotations) : 0.0;
    return robotPose.transformBy(
        new Transform3d(
            exitPosition,
            new Rotation3d(0.0, -Math.toRadians(launchAngleDegrees), turretYaw.getRadians())));
  }
}
