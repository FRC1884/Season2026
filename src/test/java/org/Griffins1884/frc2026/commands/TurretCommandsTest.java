package org.Griffins1884.frc2026.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class TurretCommandsTest {
  private static final double EPSILON = 1e-6;

  @Test
  void shootingWhileMoving_zeroMotionReturnsTarget() {
    Translation2d target = new Translation2d(4.0, 2.0);

    Translation2d result =
        TurretCommands.shootingWhileMoving(
            () -> new Pose2d(1.0, -1.0, new Rotation2d()),
            () -> target,
            () -> new Translation2d(),
            () -> new Translation2d(),
            distanceMeters -> 1.0);

    assertEquals(target.getX(), result.getX(), EPSILON);
    assertEquals(target.getY(), result.getY(), EPSILON);
  }

  @Test
  void shootingWhileMoving_accountsForReleaseTranslationInFieldFrame() {
    Translation2d result =
        TurretCommands.shootingWhileMoving(
            () -> new Pose2d(),
            () -> new Translation2d(10.0, 0.0),
            () -> new Translation2d(2.0, 0.0),
            () -> new Translation2d(),
            distanceMeters -> 1.5);

    // target - v * tof = 10 - 2 * 1.5
    assertEquals(7.0, result.getX(), EPSILON);
    assertEquals(0.0, result.getY(), EPSILON);
  }

  @Test
  void shootingWhileMoving_sanitizesInvalidMotionInputs() {
    Translation2d target = new Translation2d(6.0, -1.0);

    Translation2d result =
        TurretCommands.shootingWhileMoving(
            () -> new Pose2d(),
            () -> target,
            () -> new Translation2d(Double.NaN, Double.POSITIVE_INFINITY),
            () -> new Translation2d(Double.NEGATIVE_INFINITY, Double.NaN),
            distanceMeters -> 1.0);

    assertEquals(target.getX(), result.getX(), EPSILON);
    assertEquals(target.getY(), result.getY(), EPSILON);
    assertTrue(Double.isFinite(result.getX()));
    assertTrue(Double.isFinite(result.getY()));
  }

  @Test
  void shootingWhileMoving_returnsTargetWhenShotTimeEstimatorInvalid() {
    Translation2d target = new Translation2d(5.0, 0.5);

    Translation2d result =
        TurretCommands.shootingWhileMoving(
            () -> new Pose2d(),
            () -> target,
            () -> new Translation2d(1.0, 0.0),
            () -> new Translation2d(),
            distanceMeters -> Double.NaN);

    assertEquals(target.getX(), result.getX(), EPSILON);
    assertEquals(target.getY(), result.getY(), EPSILON);
  }
}
