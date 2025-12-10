package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class ReefIntersectUtil {

  private static final double INCH_TO_M = 0.0254;

  // ------------------------------------------
  //  BASIC GEOMETRY HELPERS
  // ------------------------------------------
  static int orientation(Translation2d a, Translation2d b, Translation2d c) {
    double v =
        (b.getY() - a.getY()) * (c.getX() - b.getX())
            - (b.getX() - a.getX()) * (c.getY() - b.getY());
    if (v == 0) return 0;
    return (v > 0) ? 1 : 2;
  }

  static boolean onSegment(Translation2d a, Translation2d c, Translation2d b) {
    return c.getX() <= Math.max(a.getX(), b.getX())
        && c.getX() >= Math.min(a.getX(), b.getX())
        && c.getY() <= Math.max(a.getY(), b.getY())
        && c.getY() >= Math.min(a.getY(), b.getY());
  }

  static boolean segmentsIntersect(
      Translation2d p1, Translation2d q1, Translation2d p2, Translation2d q2) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false;
  }

  static boolean segmentIntersectsPolygon(Translation2d a, Translation2d b, Translation2d[] poly) {
    for (int i = 0; i < poly.length; i++) {
      Translation2d p1 = poly[i];
      Translation2d p2 = poly[(i + 1) % poly.length];
      if (segmentsIntersect(a, b, p1, p2)) return true;
    }
    return false;
  }

  // ------------------------------------------
  //  EXPAND (OFFSET) POLYGON FOR ROBOT WIDTH
  // ------------------------------------------
  public static Translation2d[] expandPolygon(Translation2d[] poly, double radius) {
    int n = poly.length;
    Translation2d[] out = new Translation2d[n];

    for (int i = 0; i < n; i++) {
      Translation2d a = poly[(i - 1 + n) % n];
      Translation2d b = poly[i];
      Translation2d c = poly[(i + 1) % n];

      Translation2d e1 = new Translation2d(b.getY() - a.getY(), a.getX() - b.getX());
      Translation2d e2 = new Translation2d(c.getY() - b.getY(), b.getX() - c.getX());
      e1 = e1.div(e1.getNorm());
      e2 = e2.div(e2.getNorm());

      Translation2d avg = new Translation2d(e1.getX() + e2.getX(), e1.getY() + e2.getY());
      avg = avg.div(avg.getNorm());

      out[i] = b.plus(avg.times(radius));
    }
    return out;
  }

  // ------------------------------------------
  //  REBUILD HEXAGON VERTICES FROM SEGMENT CENTERS
  // ------------------------------------------
  public static Translation2d[] getHexVertices() {
    double IN = 0.0254;

    return new Translation2d[] {
      RotationalAllianceFlipUtil.apply(new Translation2d(176.6 * IN, 195.08 * IN)),
      RotationalAllianceFlipUtil.apply(new Translation2d(144.0 * IN, 178.78 * IN)),
      RotationalAllianceFlipUtil.apply(new Translation2d(144.0 * IN, 138.365 * IN)),
      RotationalAllianceFlipUtil.apply(new Translation2d(176.745 * IN, 121.9925 * IN)),
      RotationalAllianceFlipUtil.apply(new Translation2d(209.49 * IN, 138.365 * IN)),
      RotationalAllianceFlipUtil.apply(new Translation2d(209.49 * IN, 178.635 * IN)),
    };
  }

  public static boolean vectorsInTheSameDirection(Translation2d vector1, Translation2d vector2) {
    return Math.abs(vector1.getAngle().minus(vector2.getAngle()).getDegrees()) > 20;
  }

  public static Pose2d center() {
    return RotationalAllianceFlipUtil.apply(
        new Pose2d(
            new Translation2d(Units.inchesToMeters(176.745), Units.inchesToMeters(158.50)),
            Rotation2d.kZero));
  }

  // ------------------------------------------
  //  COLLISION CHECK
  // ------------------------------------------
  public static boolean willCollide(Pose2d robotPose, Pose2d targetPose, double robotWidthMeters) {

    Translation2d start = robotPose.getTranslation();
    Translation2d end = targetPose.getTranslation();

    double r = robotWidthMeters / 2.0;

    Translation2d[] inflated = expandPolygon(getHexVertices(), r);

    if (segmentIntersectsPolygon(start, end, inflated)) return true;

    return segmentIntersectsPolygon(end, start, getHexVertices());
  }

  public static boolean willCollideVector(
      Pose2d robotPose, Pose2d targetPose, double robotWidthMeters) {

    Translation2d reefCenter = RotationalAllianceFlipUtil.apply(new Translation2d(176.745, 158.50));

    Translation2d start = robotPose.getTranslation();
    Translation2d end = targetPose.getTranslation();

    double r = robotWidthMeters / 2.0;

    Translation2d[] inflated = expandPolygon(getHexVertices(), r);

    boolean towardsCenter = vectorsInTheSameDirection(reefCenter.minus(start), end.minus(start));

    if (segmentIntersectsPolygon(start, end, inflated) && towardsCenter) return true;

    return segmentIntersectsPolygon(end, start, getHexVertices());
  }
}
