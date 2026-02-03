package org.Griffins1884.frc2026.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class TurretUtil {
  private TurretUtil() {}

  /** Returns the robot-relative turret angle (radians) needed to point at a field target. */
  public static double turretAngleToTarget(Pose2d robotPose, Translation2d target) {
    Translation2d delta = target.minus(robotPose.getTranslation());
    double fieldAngle = Math.atan2(delta.getY(), delta.getX());
    return MathUtil.angleModulus(fieldAngle - robotPose.getRotation().getRadians());
  }

  /**
   * Calculates the shortest angular path between current and target angles Prevents turret from
   * rotating more than 180 degrees
   */
  public static double wrapAngleToShortest(double currentAngle, double targetAngle) {
    // Calculate the angular difference
    double diff = targetAngle - currentAngle;

    // Wrap to shortest path using atan2 for proper quadrant handling
    diff = Math.atan2(Math.sin(diff), Math.cos(diff));

    return currentAngle + diff;
  }
}
