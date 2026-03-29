package org.Griffins1884.frc2026.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Lookup-table shot math with TOF-based moving-shot compensation.
 *
 * <p>RPM and pivot-position values come from this codebase's historical lookup tables. The TOF
 * table and moving-shot convergence constants follow the reference implementation.
 */
public final class ShotMath {
  private static final double METERS_TO_INCHES = 39.37;
  private static final double MOVING_SHOT_SPEED_THRESHOLD_MPS = 0.1;
  private static final double MOVING_SHOT_CONVERGENCE_THRESHOLD = 0.02;
  private static final int MOVING_SHOT_MIN_ITERATIONS = 6;

  private static final double[] LOOKUP_DISTANCES_METERS = {
    2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9,
    4.0, 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 4.7, 4.8, 4.9, 5.0, 5.1, 5.2, 5.3, 5.4, 5.5, 5.6, 5.7,
    5.8, 5.9
  };

  private static final double[] SHOOTER_RPM = {
    2910.0, 2910.0, 2940.0, 2950.0, 2960.0, 3000.0, 3090.0, 3160.0, 3190.0, 3210.0, 3260.0,
    3310.0, 3350.0, 3410.0, 3430.0, 3450.0, 3450.0, 3470.0, 3490.0, 3510.0, 3540.0, 3620.0,
    3620.0, 3700.0, 3730.0, 3770.0, 3830.0, 3850.0, 3890.0, 3930.0, 3970.0, 4010.0, 4130.0,
    4150.0, 4170.0, 4210.0, 4230.0, 4290.0
  };

  private static final double[] PIVOT_POSITION = {
    0.1, 0.1, 0.18, 0.18, 0.18, 0.19, 0.19, 0.195, 0.198, 0.2, 0.208, 0.21, 0.215, 0.22, 0.226,
    0.228, 0.235, 0.237, 0.24, 0.241, 0.243, 0.245, 0.248, 0.25, 0.25, 0.25, 0.25, 0.25, 0.251,
    0.252, 0.253, 0.254, 0.255, 0.256, 0.257, 0.258, 0.259, 0.26
  };

  private static final double[] TOF_DISTANCES_INCHES = {
    43.5, 53.5, 71.5, 102.5, 140.5, 178.5, 222.5, 262.5, 293.5
  };

  private static final double[] TIME_OF_FLIGHT_SECONDS = {
    0.9, 0.9, 0.98, 1.08, 1.18, 1.28, 1.38, 1.44, 1.60
  };

  private ShotMath() {}

  public record ShotSetpoint(
      Translation2d aimPoint,
      double distanceMeters,
      double shooterRpm,
      double pivotPosition,
      double timeOfFlightSeconds) {}

  public static ShotSetpoint solve(
      Pose2d robotPose, Translation2d staticTarget, Translation2d fieldVelocityMetersPerSecond) {
    if (robotPose == null || staticTarget == null) {
      return new ShotSetpoint(new Translation2d(), 0.0, 0.0, 0.0, 0.0);
    }

    Translation2d aimPoint =
        compensateTarget(robotPose, staticTarget, sanitizeVector(fieldVelocityMetersPerSecond));
    double distanceMeters = robotPose.getTranslation().getDistance(aimPoint);
    return new ShotSetpoint(
        aimPoint,
        distanceMeters,
        getShooterRpm(distanceMeters),
        getPivotPosition(distanceMeters),
        getTimeOfFlightSeconds(distanceMeters));
  }

  public static Translation2d compensateTarget(
      Pose2d robotPose, Translation2d staticTarget, Translation2d fieldVelocityMetersPerSecond) {
    if (robotPose == null || staticTarget == null) {
      return new Translation2d();
    }

    Translation2d fieldVelocity = sanitizeVector(fieldVelocityMetersPerSecond);
    if (fieldVelocity.getNorm() <= MOVING_SHOT_SPEED_THRESHOLD_MPS) {
      return staticTarget;
    }

    double dix = staticTarget.getX() - robotPose.getX();
    double diy = staticTarget.getY() - robotPose.getY();
    double dih = Math.hypot(dix, diy);
    if (dih <= 1e-9) {
      return staticTarget;
    }

    double dnewx = dix - fieldVelocity.getX() * getTimeOfFlightSeconds(dih);
    double dnewy = diy - fieldVelocity.getY() * getTimeOfFlightSeconds(dih);
    double dnew = Math.hypot(dnewx, dnewy);
    double percentDiff = (dnew - dih) / dih;
    dih = dnew;
    int count = 0;
    double dnewTof = getTimeOfFlightSeconds(dnew);

    while ((percentDiff >= MOVING_SHOT_CONVERGENCE_THRESHOLD)
        || (count <= MOVING_SHOT_MIN_ITERATIONS)) {
      count++;
      dnewx = dix - fieldVelocity.getX() * dnewTof;
      dnewy = diy - fieldVelocity.getY() * dnewTof;
      dnew = Math.hypot(dnewx, dnewy);
      percentDiff = (dnew - dih) / Math.max(Math.abs(dih), 1e-9);
      dih = dnew;
      dnewTof = getTimeOfFlightSeconds(dnew);
    }

    return new Translation2d(dnewx + robotPose.getX(), dnewy + robotPose.getY());
  }

  public static double getShooterRpm(double distanceMeters) {
    return lookupMeters(distanceMeters, SHOOTER_RPM);
  }

  public static double getPivotPosition(double distanceMeters) {
    return lookupMeters(distanceMeters, PIVOT_POSITION);
  }

  public static double getTimeOfFlightSeconds(double distanceMeters) {
    return lookupInches(distanceMeters * METERS_TO_INCHES, TIME_OF_FLIGHT_SECONDS);
  }

  private static double lookupMeters(double distanceMeters, double[] values) {
    if (distanceMeters < LOOKUP_DISTANCES_METERS[0]) {
      return values[0];
    }

    for (int i = 1; i < LOOKUP_DISTANCES_METERS.length; i++) {
      if (distanceMeters < LOOKUP_DISTANCES_METERS[i]) {
        return values[i - 1]
            + ((values[i] - values[i - 1]) * (distanceMeters - LOOKUP_DISTANCES_METERS[i - 1]))
                / (LOOKUP_DISTANCES_METERS[i] - LOOKUP_DISTANCES_METERS[i - 1]);
      }
    }

    int lastIndex = LOOKUP_DISTANCES_METERS.length - 1;
    return values[lastIndex]
        + ((distanceMeters - LOOKUP_DISTANCES_METERS[lastIndex])
                * (values[lastIndex] - values[lastIndex - 1]))
            / (LOOKUP_DISTANCES_METERS[lastIndex] - LOOKUP_DISTANCES_METERS[lastIndex - 1]);
  }

  private static double lookupInches(double distanceInches, double[] values) {
    if (distanceInches < TOF_DISTANCES_INCHES[0]) {
      return values[0];
    }

    for (int i = 1; i < TOF_DISTANCES_INCHES.length; i++) {
      if (distanceInches < TOF_DISTANCES_INCHES[i]) {
        return values[i - 1]
            + ((values[i] - values[i - 1]) * (distanceInches - TOF_DISTANCES_INCHES[i - 1]))
                / (TOF_DISTANCES_INCHES[i] - TOF_DISTANCES_INCHES[i - 1]);
      }
    }

    int lastIndex = TOF_DISTANCES_INCHES.length - 1;
    return values[lastIndex]
        + ((distanceInches - TOF_DISTANCES_INCHES[lastIndex])
                * (values[lastIndex] - values[lastIndex - 1]))
            / (TOF_DISTANCES_INCHES[lastIndex] - TOF_DISTANCES_INCHES[lastIndex - 1]);
  }

  private static Translation2d sanitizeVector(Translation2d vector) {
    if (vector == null || !Double.isFinite(vector.getX()) || !Double.isFinite(vector.getY())) {
      return new Translation2d();
    }
    return vector;
  }
}
