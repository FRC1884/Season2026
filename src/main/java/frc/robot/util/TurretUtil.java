package frc.robot.util;

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
}
