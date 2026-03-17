package org.Griffins1884.frc2026.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.Griffins1884.frc2026.GlobalConstants;
import org.junit.jupiter.api.Test;

class TurretCommandsAllianceSymmetryTest {
  private static final double EPSILON = 1e-9;

  @Test
  void shootingWhileMovingProducesMirroredAimPointForHubTargets() {
    Pose2d bluePose = new Pose2d(3.2, 4.1, new Rotation2d());
    Pose2d redPose = mirrorPose(bluePose);

    Translation2d blueTarget = GlobalConstants.FieldConstants.Hub.topCenterPoint.toTranslation2d();
    Translation2d redTarget =
        GlobalConstants.FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();

    Translation2d blueVelocity = new Translation2d(1.1, -0.35);
    Translation2d redVelocity = mirrorVector(blueVelocity);
    Translation2d blueAcceleration = new Translation2d(0.18, 0.05);
    Translation2d redAcceleration = mirrorVector(blueAcceleration);

    Translation2d blueAimPoint =
        TurretCommands.shootingWhileMoving(
            () -> bluePose,
            () -> blueTarget,
            () -> blueVelocity,
            () -> blueAcceleration,
            distance -> 0.82);
    Translation2d redAimPoint =
        TurretCommands.shootingWhileMoving(
            () -> redPose,
            () -> redTarget,
            () -> redVelocity,
            () -> redAcceleration,
            distance -> 0.82);

    assertEquals(mirrorX(blueAimPoint.getX()), redAimPoint.getX(), EPSILON);
    assertEquals(blueAimPoint.getY(), redAimPoint.getY(), EPSILON);
  }

  @Test
  void shootingWhileMovingProducesMirroredAimPointForArbitraryTargets() {
    Pose2d bluePose = new Pose2d(4.8, 2.3, Rotation2d.fromDegrees(12.0));
    Pose2d redPose = mirrorPose(bluePose);

    Translation2d blueTarget = new Translation2d(6.1, 6.7);
    Translation2d redTarget = mirrorTranslation(blueTarget);

    Translation2d blueVelocity = new Translation2d(0.7, 0.42);
    Translation2d redVelocity = mirrorVector(blueVelocity);
    Translation2d blueAcceleration = new Translation2d(-0.11, 0.09);
    Translation2d redAcceleration = mirrorVector(blueAcceleration);

    Translation2d blueAimPoint =
        TurretCommands.shootingWhileMoving(
            () -> bluePose,
            () -> blueTarget,
            () -> blueVelocity,
            () -> blueAcceleration,
            distance -> 0.67);
    Translation2d redAimPoint =
        TurretCommands.shootingWhileMoving(
            () -> redPose,
            () -> redTarget,
            () -> redVelocity,
            () -> redAcceleration,
            distance -> 0.67);

    assertEquals(mirrorX(blueAimPoint.getX()), redAimPoint.getX(), EPSILON);
    assertEquals(blueAimPoint.getY(), redAimPoint.getY(), EPSILON);
  }

  private static Pose2d mirrorPose(Pose2d pose) {
    return new Pose2d(mirrorTranslation(pose.getTranslation()), pose.getRotation());
  }

  private static Translation2d mirrorTranslation(Translation2d translation) {
    return new Translation2d(mirrorX(translation.getX()), translation.getY());
  }

  private static Translation2d mirrorVector(Translation2d vector) {
    return new Translation2d(-vector.getX(), vector.getY());
  }

  private static double mirrorX(double x) {
    return GlobalConstants.FieldConstants.fieldLength - x;
  }
}
