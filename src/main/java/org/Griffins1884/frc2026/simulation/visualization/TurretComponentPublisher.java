package org.Griffins1884.frc2026.simulation.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.Griffins1884.frc2026.util.ballistics.ShotModelConfig;

/** Builds field-space turret component poses for AdvantageScope visualization. */
public final class TurretComponentPublisher {
  private TurretComponentPublisher() {}

  public static Pose2d createPose2d(
      Pose2d robotPose, Rotation2d turretYaw, ShotModelConfig config) {
    Translation3d mount =
        config != null ? config.turretMount().toTranslation3d() : new Translation3d();
    Translation2d fieldTranslation =
        new Translation2d(mount.getX(), mount.getY())
            .rotateBy(robotPose.getRotation())
            .plus(robotPose.getTranslation());
    return new Pose2d(fieldTranslation, robotPose.getRotation().plus(turretYaw));
  }

  public static Pose3d createPose3d(
      Pose2d robotPose, Rotation2d turretYaw, ShotModelConfig config) {
    Translation3d mount =
        config != null ? config.turretMount().toTranslation3d() : new Translation3d();
    Translation3d fieldTranslation = rotateIntoField(robotPose, mount);
    return new Pose3d(
        fieldTranslation,
        new Rotation3d(0.0, 0.0, robotPose.getRotation().plus(turretYaw).getRadians()));
  }

  static Translation3d rotateIntoField(Pose2d robotPose, Translation3d robotRelativeTranslation) {
    Translation2d fieldXY =
        new Translation2d(robotRelativeTranslation.getX(), robotRelativeTranslation.getY())
            .rotateBy(robotPose.getRotation())
            .plus(robotPose.getTranslation());
    return new Translation3d(fieldXY.getX(), fieldXY.getY(), robotRelativeTranslation.getZ());
  }
}
