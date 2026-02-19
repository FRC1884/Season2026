package org.Griffins1884.frc2026.util;

import java.util.Optional;

/**
 * Projectile-based shot solver for an FRC-style robot shooter.
 *
 * <p>All units are SI: meters, seconds, radians. Angles returned are in degrees.
 *
 * <p>Physics model (no drag):
 *
 * <pre>
 * Δh = x·tan(θ) − (g·x²) / (2·v²·cos²(θ))
 * v = sqrt( (g·x²) / (2·cos²(θ)·(x·tan(θ) − Δh)) )
 * </pre>
 *
 * <p>Calibration:
 *
 * <ol>
 *   <li>Lock the pivot at a known angle (ex: 45°).</li>
 *   <li>Shoot from several distances and record the RPM used.</li>
 *   <li>Compute required exit velocity v using the physics equation.</li>
 *   <li>Fit a linear regression from RPM → v.</li>
 *   <li>Paste kV (slope) and kS (intercept) below.</li>
 * </ol>
 */
public final class ShotSolver {
  private ShotSolver() {}

  // ==== CALIBRATION CONSTANTS (TEAM MUST TUNE) ====
  /** meters/sec per RPM */
  public static double kV = 0.00285;

  /** velocity offset (m/s) */
  public static double kS = 0.35;

  // ==== ROBOT GEOMETRY ====
  public static double SHOOTER_HEIGHT = 0.72; // meters
  public static double DEFAULT_TARGET_HEIGHT = 2.05; // meters

  // ==== SEARCH POLICY ====
  public static double MIN_ANGLE_DEG = 25.0;
  public static double MAX_ANGLE_DEG = 65.0;
  public static double ANGLE_STEP_DEG = 0.25;
  public static double MAX_EXIT_VELOCITY = 30.0; // m/s, tune per robot

  private static final double GRAVITY = 9.80665;
  private static final double EPSILON = 1e-9;

  public record ShotSolution(double angleDegrees, double rpm) {}

  /** Solve for the default target height. */
  public static ShotSolution solveForDistance(double distanceMeters) {
    return solveForDistanceAndHeight(distanceMeters, DEFAULT_TARGET_HEIGHT);
  }

  /** Solve for a specific target height. */
  public static ShotSolution solveForDistanceAndHeight(
      double distanceMeters, double targetHeightMeters) {
    ShotSolution solution = searchOptimalSolution(distanceMeters, targetHeightMeters);
    if (solution == null) {
      throw new RuntimeException(
          "No valid shot solution for distance="
              + distanceMeters
              + "m, targetHeight="
              + targetHeightMeters
              + "m");
    }
    return solution;
  }

  private static ShotSolution searchOptimalSolution(double x, double targetHeight) {
    if (!Double.isFinite(x) || x <= 0.0 || !Double.isFinite(targetHeight)) {
      return null;
    }
    double deltaH = targetHeight - SHOOTER_HEIGHT;
    ShotSolution best = null;
    double bestVelocity = Double.POSITIVE_INFINITY;

    for (double deg = MIN_ANGLE_DEG; deg <= MAX_ANGLE_DEG + 1e-9; deg += ANGLE_STEP_DEG) {
      double thetaRad = Math.toRadians(deg);
      Optional<Double> velocityOpt = computeVelocity(x, deltaH, thetaRad);
      if (velocityOpt.isEmpty()) {
        continue;
      }
      double v = velocityOpt.get();
      if (!Double.isFinite(v) || v <= 0.0 || v > MAX_EXIT_VELOCITY) {
        continue;
      }
      if (v < bestVelocity) {
        double rpm = velocityToRPM(v);
        if (!Double.isFinite(rpm) || rpm <= 0.0) {
          continue;
        }
        bestVelocity = v;
        best = new ShotSolution(deg, rpm);
      }
    }
    return best;
  }

  private static Optional<Double> computeVelocity(double x, double deltaH, double thetaRad) {
    double cos = Math.cos(thetaRad);
    double sin = Math.sin(thetaRad);
    if (Math.abs(cos) < EPSILON) {
      return Optional.empty();
    }
    double tan = sin / cos;
    double denom = 2.0 * cos * cos * (x * tan - deltaH);
    if (denom <= EPSILON) {
      return Optional.empty();
    }
    double numerator = GRAVITY * x * x;
    double v2 = numerator / denom;
    if (v2 <= 0.0 || !Double.isFinite(v2)) {
      return Optional.empty();
    }
    double v = Math.sqrt(v2);
    if (!Double.isFinite(v)) {
      return Optional.empty();
    }
    return Optional.of(v);
  }

  private static double velocityToRPM(double velocity) {
    if (Math.abs(kV) < EPSILON) {
      return Double.NaN;
    }
    return (velocity - kS) / kV;
  }
}
