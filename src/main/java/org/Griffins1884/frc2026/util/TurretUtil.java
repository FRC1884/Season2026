package org.Griffins1884.frc2026.util;

import static java.lang.Math.PI;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class TurretUtil {
  private TurretUtil() {}

  /** Returns the robot-relative turret angle (radians) needed to point at a field target. */
  public static double turretAngleToTarget(Pose2d robotPose, Translation2d target) {
    double dx = target.getX() - robotPose.getX();
    double dy = target.getY() - robotPose.getY();

    double targetAngleField = Math.atan2(dy, dx);

    double desiredTurretRobot = targetAngleField - robotPose.getRotation().getRadians();

    return wrap0To2PI(desiredTurretRobot);
  }

  public static double wrap0To2PI(double angleRad) {
    angleRad %= (2.0 * PI);
    if (angleRad < 0) angleRad += 2.0 * PI;
    return angleRad;
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
