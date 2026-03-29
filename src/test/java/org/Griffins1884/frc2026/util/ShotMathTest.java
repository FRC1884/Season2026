package org.Griffins1884.frc2026.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class ShotMathTest {
  private static final double EPSILON = 1e-6;

  @Test
  void returnsExpectedLookupValuesAtKnownDistance() {
    assertEquals(3890.0, ShotMath.getShooterRpm(5.0), EPSILON);
    assertEquals(0.251, ShotMath.getPivotPosition(5.0), EPSILON);
    assertEquals(1.3217045454545455, ShotMath.getTimeOfFlightSeconds(5.0), EPSILON);
  }

  @Test
  void leavesTargetUnchangedWhenRobotIsNearlyStationary() {
    Pose2d robotPose = new Pose2d(1.0, 2.0, new Rotation2d());
    Translation2d target = new Translation2d(5.0, 6.0);

    Translation2d compensated =
        ShotMath.compensateTarget(robotPose, target, new Translation2d(0.05, 0.0));

    assertEquals(target.getX(), compensated.getX(), EPSILON);
    assertEquals(target.getY(), compensated.getY(), EPSILON);
  }

  @Test
  void matchesReferenceLeadCompensation() {
    Pose2d robotPose = new Pose2d();
    Translation2d target = new Translation2d(5.0, 0.0);

    Translation2d compensated =
        ShotMath.compensateTarget(robotPose, target, new Translation2d(1.0, 0.0));

    assertEquals(3.796408914197519, compensated.getX(), EPSILON);
    assertEquals(0.0, compensated.getY(), EPSILON);
  }
}
