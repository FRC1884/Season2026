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
import org.Griffins1884.frc2026.subsystems.shooter.ShooterConstants;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {
  private static final double GRAVITY = 9.80665;
  private static final double shooterDistanceCenter = 0.02; // TODO: Tune

  public enum Vals {
    RPM,
    ANGLE
  }

  private static final NavigableMap<Double, Double> angleByDistanceDeg = buildAngleTable();
  private static final NavigableMap<Double, Double> rpmByDistance = buildRpmTable();

  public static Map<Vals, Double> calc(Pose2d robot, Translation2d target) {
    // Distance Vector Calculation
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
    Logger.recordOutput("Shooter/DistanceToHubMeters", distance);

    return dataPack(distance);
  }

  public static double getShooterRpm(double distanceMeters) {
    return lookupInterpolated(rpmByDistance, distanceMeters);
  }

  public static double getPivotAngleDegrees(double distanceMeters) {
    return lookupInterpolated(angleByDistanceDeg, distanceMeters);
  }

  /** Placeholder conversion from degrees to pivot units (0.1 to 1.6 scale). */
  public static double pivotUnitsFromDegrees(double degrees) {
    return degrees;
  }

  public static double getPivotAngleOutput(double distanceMeters) {
    return pivotUnitsFromDegrees(getPivotAngleDegrees(distanceMeters));
  }

  public static double getPivotAngleRad(double distanceMeters) {
    return Math.toRadians(getPivotAngleDegrees(distanceMeters));
  }

  public static Map<Vals, Double> dataPack(double distanceMeters) {
    Map<Vals, Double> data = new HashMap<>();
    data.put(Vals.RPM, getShooterRpm(distanceMeters));
    data.put(Vals.ANGLE, getPivotAngleOutput(distanceMeters));
    return data;
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

  private static NavigableMap<Double, Double> buildAngleTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    // Distance meters -> pivot angle setpoint.
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

  private static NavigableMap<Double, Double> buildRpmTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    // Distance meters -> shooter RPM.
    table.put(2.2, 2800.0);
    table.put(2.3, 2800.0);
    table.put(2.4, 2830.0);
    table.put(2.5, 2840.0);
    table.put(2.6, 2850.0);
    table.put(2.7, 2890.0);
    table.put(2.8, 2980.0);
    table.put(2.9, 3050.0);
    table.put(3.0, 3080.0);
    table.put(3.1, 3100.0);
    table.put(3.2, 3150.0);
    table.put(3.3, 3200.0);
    table.put(3.4, 3240.0);
    table.put(3.5, 3250.0);
    table.put(3.6, 3270.0);
    table.put(3.7, 3290.0);
    table.put(3.8, 3300.0);
    table.put(3.9, 3310.0);
    table.put(4.0, 3380.0);
    table.put(4.1, 3400.0);
    table.put(4.2, 3430.0);
    table.put(4.3, 3470.0);
    table.put(4.4, 3470.0);
    table.put(4.5, 3550.0);
    table.put(4.6, 3580.0);
    table.put(4.7, 3620.0);
    table.put(4.8, 3680.0);
    table.put(4.9, 3700.0);
    table.put(5.0, 3740.0);
    table.put(5.1, 3780.0);
    table.put(5.2, 3820.0);
    table.put(5.3, 3860.0);
    table.put(5.4, 3980.0);
    table.put(5.5, 4000.0);
    table.put(5.6, 4020.0);
    table.put(5.7, 4060.0);
    table.put(5.8, 4080.0);
    table.put(5.9, 4140.0);
    return table;
  }

  public record ShotTimeEstimate(
      double timeSeconds,
      double exitVelocityMetersPerSecond,
      double predictedHeightMeters,
      double heightErrorMeters,
      boolean feasible) {}

  public static ShotTimeEstimate estimateShotTimeDetailed(
      double distanceMeters,
      double hoodAngleRad,
      double shooterExitHeightMeters,
      double targetHeightMeters,
      double wheelRpm,
      double wheelRadiusMeters,
      double gearRatio,
      double slipFactor) {
    if (distanceMeters <= 0.0) {
      return new ShotTimeEstimate(0.0, 0.0, shooterExitHeightMeters, 0.0, false);
    }
    double exitVelocity =
        (wheelRpm / 60.0) * (2.0 * Math.PI) * wheelRadiusMeters * gearRatio * slipFactor;
    double cos = Math.cos(hoodAngleRad);
    if (Math.abs(cos) < 1e-6 || exitVelocity <= 1e-6) {
      return new ShotTimeEstimate(0.0, exitVelocity, shooterExitHeightMeters, 0.0, false);
    }
    double timeSeconds = distanceMeters / (exitVelocity * cos);
    double predictedHeight =
        shooterExitHeightMeters
            + exitVelocity * Math.sin(hoodAngleRad) * timeSeconds
            - 0.5 * GRAVITY * timeSeconds * timeSeconds;
    double heightError = targetHeightMeters - predictedHeight;
    boolean feasible = !Double.isNaN(timeSeconds) && timeSeconds > 0.0;
    return new ShotTimeEstimate(timeSeconds, exitVelocity, predictedHeight, heightError, feasible);
  }

  public static double estimateShotTimeSeconds(double distanceMeters, double hoodAngleRad) {
    ShotTimeEstimate estimate =
        estimateShotTimeDetailed(
            distanceMeters,
            hoodAngleRad,
            ShooterConstants.EXIT_HEIGHT_METERS,
            ShooterConstants.TARGET_HEIGHT_METERS,
            getShooterRpm(distanceMeters),
            ShooterConstants.FLYWHEEL_RADIUS_METERS,
            ShooterConstants.FLYWHEEL_GEAR_RATIO,
            ShooterConstants.SLIP_FACTOR.get());
    return estimate.timeSeconds();
  }

  public static Command pivotOpenLoop(ShooterPivotSubsystem pivot, DoubleSupplier percentSupplier) {
    return Commands.runEnd(
        () -> pivot.setOpenLoop(percentSupplier.getAsDouble()), pivot::stopOpenLoop, pivot);
  }
}
