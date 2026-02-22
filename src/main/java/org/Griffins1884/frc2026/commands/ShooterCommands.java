package org.Griffins1884.frc2026.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.function.DoubleSupplier;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterConstants;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotConstants;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotSubsystem;

public class ShooterCommands {
  private static final double GRAVITY = 9.80665;
  private static final double shooterDistanceCenter = 0.02; // TODO: Tune
  private static final double LEGACY_PIVOT_DEGREES_SLOPE = 11.875;
  private static final double LEGACY_PIVOT_DEGREES_OFFSET = 18.0;
  private static final double LEGACY_PIVOT_DEGREES_MIN = 18.0;
  private static final double LEGACY_PIVOT_DEGREES_MAX = 37.0;

  private static final double TABLE_MIN_DISTANCE_METERS = 2.2;
  private static final double TABLE_MAX_DISTANCE_METERS = 5.9;
  private static final double TABLE_STEP_METERS = 0.1;

  private static final double PIVOT_OUTPUT_MIN = 0.0;
  private static final double PIVOT_OUTPUT_MAX = 1.6;
  private static final double PIVOT_ANGLE_FROM_VERTICAL_DOWN_MIN_DEG = 30.0;
  private static final double PIVOT_ANGLE_FROM_VERTICAL_DOWN_MAX_DEG = 63.0;
  private static final double PIVOT_OUTPUT_SEARCH_STEP = 0.002;

  private static final double SOLVER_MIN_RPM = 0.0;
  private static final double SOLVER_MAX_RPM = 5500.0;
  private static final double BALANCED_WEIGHT_RPM = 0.5;
  private static final double BALANCED_WEIGHT_TOF = 0.5;
  private static final double TOF_NORMALIZATION_SECONDS = 2.0;

  public enum Vals {
    RPM,
    ANGLE
  }

  private record LookupPoint(double pivotOutput, double rpm) {}

  private static final NavigableMap<Double, Double> legacyAngleByDistance = buildLegacyAngleTable();
  private static final NavigableMap<Double, Double> legacyRpmByDistance = buildLegacyRpmTable();
  private static final NavigableMap<Double, LookupPoint> ballisticLookupByDistance =
      buildBallisticLookupTable();
  private static final NavigableMap<Double, Double> ballisticAngleByDistance =
      buildBallisticAngleTable();
  private static final NavigableMap<Double, Double> ballisticRpmByDistance =
      buildBallisticRpmTable();

  public static Map<Vals, Double> calc(
      Pose2d robot, Translation2d target, Superstructure.SuperState state) {
    // Distances
    double distanceX;
    double distanceY;
    double distance;
    double yawAngle;

    // Calculate the Straight line distance in (m) to the hub
    distanceX = robot.getX() - target.getX();
    distanceY = robot.getY() - target.getY();
    yawAngle = robot.getRotation().getDegrees();
    distanceX -= shooterDistanceCenter * Math.sin(Math.toRadians(yawAngle));
    distanceY += shooterDistanceCenter * Math.cos(Math.toRadians(yawAngle));

    distance =
        (double) Math.round(Math.hypot(Math.abs(distanceX), Math.abs(distanceY)) * 100) / 100;

    return dataPack(distance, state);
  }

  public static double getShooterRpm(double distanceMeters) {
    return lookupInterpolated(getActiveRpmTable(), distanceMeters);
  }

  public static double getPivotAngleDegrees(double distanceMeters) {
    double pivotOutput = getPivotAngleOutput(distanceMeters);
    if (ShooterConstants.SHOT_LOOKUP_MODE == ShooterConstants.ShotLookupMode.BALLISTIC_MODEL) {
      return pivotOutputToVerticalDownDegrees(pivotOutput);
    }
    double pivotAngleDeg = (LEGACY_PIVOT_DEGREES_SLOPE * pivotOutput) + LEGACY_PIVOT_DEGREES_OFFSET;
    return (pivotAngleDeg > LEGACY_PIVOT_DEGREES_MAX || pivotAngleDeg < LEGACY_PIVOT_DEGREES_MIN)
        ? 0
        : pivotAngleDeg;
  }

  public static double getPivotAngleOutput(double distanceMeters) {
    return lookupInterpolated(getActiveAngleTable(), distanceMeters);
  }

  public static double getPivotAngleRad(double distanceMeters) {
    return Math.toRadians(getPivotAngleDegrees(distanceMeters));
  }

  public static Map<Vals, Double> dataPack(double distanceMeters, Superstructure.SuperState state) {
    Map<Vals, Double> data = new HashMap<>();
    if (state == Superstructure.SuperState.FERRYING) {
      data.put(Vals.RPM, calcFerrying(distanceMeters));
      data.put(Vals.ANGLE, ShooterPivotConstants.FERRYING_ANGLE_RAD);
    } else {
      data.put(Vals.RPM, getShooterRpm(distanceMeters));
      data.put(Vals.ANGLE, getPivotAngleOutput(distanceMeters));
    }
    return data;
  }

  public static double calcFerrying(double distance) {
    return Math.max(
        Math.sqrt((distance / ShooterConstants.MAX_DISTANCE)) * ShooterConstants.TARGET_RPM,
        ShooterConstants.TARGET_RPM);
  }

  private static double lookupInterpolated(
      NavigableMap<Double, Double> table, double distanceMeters) {
    if (table.isEmpty() || Double.isNaN(distanceMeters)) {
      return 0.0;
    }
    var floor = table.floorEntry(distanceMeters);
    var ceil = table.ceilingEntry(distanceMeters);
    if (floor == null) {
      return ceil.getValue();
    }
    if (ceil == null) {
      return floor.getValue();
    }
    if (Double.compare(floor.getKey(), ceil.getKey()) == 0) {
      return floor.getValue();
    }
    double t = (distanceMeters - floor.getKey()) / (ceil.getKey() - floor.getKey());
    return floor.getValue() + (ceil.getValue() - floor.getValue()) * t;
  }

  private static NavigableMap<Double, Double> buildLegacyAngleTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(2.2, 0.1);
    table.put(2.3, 0.1);
    table.put(2.4, 0.18);
    table.put(2.5, 0.18);
    table.put(2.6, 0.18);
    table.put(2.7, 0.19);
    table.put(2.8, 0.19);
    table.put(2.9, 0.195);
    table.put(3.0, 0.198);
    table.put(3.1, 0.2);
    table.put(3.2, 0.208);
    table.put(3.3, 0.21);
    table.put(3.4, 0.215);
    table.put(3.5, 0.22);
    table.put(3.6, 0.226);
    table.put(3.7, 0.228);
    table.put(3.8, 0.235);
    table.put(3.9, 0.237);
    table.put(4.0, 0.24);
    table.put(4.1, 0.241);
    table.put(4.2, 0.243);
    table.put(4.3, 0.245);
    table.put(4.4, 0.248);
    table.put(4.5, 0.25);
    table.put(4.6, 0.25);
    table.put(4.7, 0.25);
    table.put(4.8, 0.25);
    table.put(4.9, 0.25);
    table.put(5.0, 0.251);
    table.put(5.1, 0.252);
    table.put(5.2, 0.253);
    table.put(5.3, 0.254);
    table.put(5.4, 0.255);
    table.put(5.5, 0.256);
    table.put(5.6, 0.257);
    table.put(5.7, 0.258);
    table.put(5.8, 0.259);
    table.put(5.9, 0.26);
    return table;
  }

  private static NavigableMap<Double, Double> buildLegacyRpmTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    table.put(2.2, 2910.0);
    table.put(2.3, 2910.0);
    table.put(2.4, 2940.0);
    table.put(2.5, 2950.0);
    table.put(2.6, 2960.0);
    table.put(2.7, 3000.0);
    table.put(2.8, 3090.0);
    table.put(2.9, 3160.0);
    table.put(3.0, 3190.0);
    table.put(3.1, 3210.0);
    table.put(3.2, 3260.0);
    table.put(3.3, 3310.0);
    table.put(3.4, 3350.0);
    table.put(3.5, 3410.0);
    table.put(3.6, 3430.0);
    table.put(3.7, 3450.0);
    table.put(3.8, 3450.0);
    table.put(3.9, 3470.0);
    table.put(4.0, 3490.0);
    table.put(4.1, 3510.0);
    table.put(4.2, 3540.0);
    table.put(4.3, 3580.0);
    table.put(4.4, 3580.0);
    table.put(4.5, 3660.0);
    table.put(4.6, 3690.0);
    table.put(4.7, 3730.0);
    table.put(4.8, 3790.0);
    table.put(4.9, 3810.0);
    table.put(5.0, 3850.0);
    table.put(5.1, 3890.0);
    table.put(5.2, 3930.0);
    table.put(5.3, 3970.0);
    table.put(5.4, 4090.0);
    table.put(5.5, 4110.0);
    table.put(5.6, 4130.0);
    table.put(5.7, 4170.0);
    table.put(5.8, 4190.0);
    table.put(5.9, 4250.0);
    return table;
  }

  private static NavigableMap<Double, Double> buildBallisticAngleTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    for (var entry : ballisticLookupByDistance.entrySet()) {
      table.put(entry.getKey(), entry.getValue().pivotOutput());
    }
    return table;
  }

  private static NavigableMap<Double, Double> buildBallisticRpmTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    for (var entry : ballisticLookupByDistance.entrySet()) {
      table.put(entry.getKey(), entry.getValue().rpm());
    }
    return table;
  }

  private static NavigableMap<Double, LookupPoint> buildBallisticLookupTable() {
    NavigableMap<Double, LookupPoint> table = new TreeMap<>();

    int steps =
        (int)
            Math.round((TABLE_MAX_DISTANCE_METERS - TABLE_MIN_DISTANCE_METERS) / TABLE_STEP_METERS);
    for (int i = 0; i <= steps; i++) {
      double distanceMeters = roundToTenth(TABLE_MIN_DISTANCE_METERS + (i * TABLE_STEP_METERS));
      LookupPoint point = solveBalancedLookup(distanceMeters);
      if (point == null) {
        point =
            table.isEmpty()
                ? new LookupPoint(PIVOT_OUTPUT_MIN, SOLVER_MAX_RPM)
                : table.lastEntry().getValue();
      }
      table.put(distanceMeters, point);
    }
    return table;
  }

  private static NavigableMap<Double, Double> getActiveAngleTable() {
    return ShooterConstants.SHOT_LOOKUP_MODE == ShooterConstants.ShotLookupMode.BALLISTIC_MODEL
        ? ballisticAngleByDistance
        : legacyAngleByDistance;
  }

  private static NavigableMap<Double, Double> getActiveRpmTable() {
    return ShooterConstants.SHOT_LOOKUP_MODE == ShooterConstants.ShotLookupMode.BALLISTIC_MODEL
        ? ballisticRpmByDistance
        : legacyRpmByDistance;
  }

  private static LookupPoint solveBalancedLookup(double distanceMeters) {
    double bestScore = Double.POSITIVE_INFINITY;
    LookupPoint bestPoint = null;

    for (double output = PIVOT_OUTPUT_MIN;
        output <= PIVOT_OUTPUT_MAX + 1e-9;
        output += PIVOT_OUTPUT_SEARCH_STEP) {
      double pivotDeg = pivotOutputToVerticalDownDegrees(output);
      double launchElevationDeg = 90.0 - pivotDeg;
      double launchElevationRad = Math.toRadians(launchElevationDeg);
      double cos = Math.cos(launchElevationRad);
      if (Math.abs(cos) < 1e-9) {
        continue;
      }

      Double exitVelocityOpt = solveRequiredExitVelocity(distanceMeters, launchElevationRad);
      if (exitVelocityOpt == null) {
        continue;
      }
      double exitVelocity = exitVelocityOpt;
      double rpm = velocityToRpm(exitVelocity);
      if (!Double.isFinite(rpm) || rpm < SOLVER_MIN_RPM || rpm > SOLVER_MAX_RPM) {
        continue;
      }

      double tof = distanceMeters / (exitVelocity * cos);
      if (!Double.isFinite(tof) || tof <= 0.0) {
        continue;
      }

      double score =
          (BALANCED_WEIGHT_RPM * (rpm / SOLVER_MAX_RPM))
              + (BALANCED_WEIGHT_TOF * (tof / TOF_NORMALIZATION_SECONDS));
      if (score < bestScore) {
        bestScore = score;
        bestPoint = new LookupPoint(output, rpm);
      }
    }
    return bestPoint;
  }

  private static Double solveRequiredExitVelocity(
      double distanceMeters, double launchElevationRad) {
    double deltaH = ShooterConstants.TARGET_HEIGHT_METERS - ShooterConstants.EXIT_HEIGHT_METERS;
    double cos = Math.cos(launchElevationRad);
    double tan = Math.tan(launchElevationRad);
    double denominator = 2.0 * cos * cos * ((distanceMeters * tan) - deltaH);
    if (denominator <= 1e-9) {
      return null;
    }
    double vSquared = (GRAVITY * distanceMeters * distanceMeters) / denominator;
    if (vSquared <= 0.0 || !Double.isFinite(vSquared)) {
      return null;
    }
    double exitVelocity = Math.sqrt(vSquared);
    return Double.isFinite(exitVelocity) ? exitVelocity : null;
  }

  private static double velocityToRpm(double exitVelocityMetersPerSec) {
    return (exitVelocityMetersPerSec * 60.0)
        / (2.0
            * Math.PI
            * ShooterConstants.FLYWHEEL_RADIUS_METERS
            * ShooterConstants.FLYWHEEL_GEAR_RATIO
            * ShooterConstants.SLIP_FACTOR.get());
  }

  private static double pivotOutputToVerticalDownDegrees(double pivotOutput) {
    double clampedOutput = Math.max(PIVOT_OUTPUT_MIN, Math.min(PIVOT_OUTPUT_MAX, pivotOutput));
    double ratio = (clampedOutput - PIVOT_OUTPUT_MIN) / (PIVOT_OUTPUT_MAX - PIVOT_OUTPUT_MIN);
    return PIVOT_ANGLE_FROM_VERTICAL_DOWN_MIN_DEG
        + ratio * (PIVOT_ANGLE_FROM_VERTICAL_DOWN_MAX_DEG - PIVOT_ANGLE_FROM_VERTICAL_DOWN_MIN_DEG);
  }

  private static double roundToTenth(double value) {
    return Math.round(value * 10.0) / 10.0;
  }

  public record ShotTimeEstimate(
      double timeSeconds,
      double exitVelocityMetersPerSecond,
      double predictedHeightMeters,
      double heightErrorMeters,
      boolean feasible) {}

  public static Command pivotOpenLoop(ShooterPivotSubsystem pivot, DoubleSupplier percentSupplier) {
    return Commands.runEnd(
        () -> pivot.setOpenLoop(percentSupplier.getAsDouble()), pivot::stopOpenLoop, pivot);
  }
}
