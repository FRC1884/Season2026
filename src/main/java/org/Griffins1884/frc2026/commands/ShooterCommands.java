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

    // Calculate the Straight line distance in (m) to the hub
    distanceX = robot.getX() - target.getX();
    distanceY = robot.getY() - target.getY();

    distance =
        (double) Math.round(Math.hypot(Math.abs(distanceX), Math.abs(distanceY)) * 100) / 100;
    Logger.recordOutput("Shooter/DistanceToHubMeters", distance);

    return dataPack(distance);
  }

  public static double getShooterRpm(double distanceMeters) {
    return lookupInterpolated(rpmByDistance, distanceMeters);
  }

  public static double getPivotAngleOutput(double distanceMeters) {
    return lookupInterpolated(angleByDistanceDeg, distanceMeters);
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
    // Distance meters -> pivot angle encoder position. Add/edit your mapping here.
    table.put(0.0,0.0);
    table.put(0.1,0.0);
    table.put(0.2,0.0);
    table.put(0.3,0.0);
    table.put(0.4,0.0);
    table.put(0.5,0.0);
    table.put(0.6,0.0);
    table.put(0.7,0.0);
    table.put(0.8,0.0);
    table.put(0.9,0.0);
    table.put(1.0,0.0);
    table.put(1.1,0.0);
    table.put(1.2,0.0);
    table.put(1.3,0.0);
    table.put(1.4,0.0);
    table.put(1.5,0.0);
    table.put(1.6,0.0);
    table.put(1.7,0.0);
    table.put(1.8,0.0);
    table.put(1.9,0.0);
    table.put(2.0,0.0);
    table.put(2.1,0.0);
    table.put(2.2,0.0);
    table.put(2.3,0.0);
    table.put(2.4,0.0);
    table.put(2.5,0.0);
    table.put(2.6,0.0);
    table.put(2.7,0.0);
    table.put(2.8,0.0);
    table.put(2.9,0.0);
    table.put(3.0,0.0);
    table.put(3.1,0.0);
    table.put(3.2,0.0);
    table.put(3.3,0.0);
    table.put(3.4,0.0);
    table.put(3.5,0.0);
    table.put(3.6,0.0);
    table.put(3.7,0.0);
    table.put(3.8,0.0);
    table.put(3.9,0.0);
    table.put(4.0,0.0);
    table.put(4.1,0.0);
    table.put(4.2,0.0);
    table.put(4.3,0.0);
    table.put(4.4,0.0);
    table.put(4.5,0.0);
    table.put(4.6,0.0);
    table.put(4.7,0.0);
    table.put(4.8,0.0);
    table.put(4.9,0.0);
    table.put(5.0,0.0);
    table.put(5.1,0.0);
    table.put(5.2,0.0);
    table.put(5.3,0.0);
    table.put(5.4,0.0);
    table.put(5.5,0.0);
    table.put(5.6,0.0);
    table.put(5.7,0.0);
    table.put(5.8,0.0);
    table.put(5.9,0.0);
    table.put(6.0,0.0);
    table.put(6.1,0.0);
    table.put(6.2,0.0);
    table.put(6.3,0.0);
    table.put(6.4,0.0);
    table.put(6.5,0.0);
    table.put(6.6,0.0);
    table.put(6.7,0.0);
    table.put(6.8,0.0);
    table.put(6.9,0.0);
    table.put(7.0,0.0);
    
    return table;
  }

  private static NavigableMap<Double, Double> buildRpmTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    // Distance meters -> shooter RPM. Add/edit your mapping here.
    table.put(0.0,0.0);
    table.put(0.1,0.0);
    table.put(0.2,0.0);
    table.put(0.3,0.0);
    table.put(0.4,0.0);
    table.put(0.5,0.0);
    table.put(0.6,0.0);
    table.put(0.7,0.0);
    table.put(0.8,0.0);
    table.put(0.9,0.0);
    table.put(1.0,0.0);
    table.put(1.1,0.0);
    table.put(1.2,0.0);
    table.put(1.3,0.0);
    table.put(1.4,0.0);
    table.put(1.5,0.0);
    table.put(1.6,0.0);
    table.put(1.7,0.0);
    table.put(1.8,0.0);
    table.put(1.9,0.0);
    table.put(2.0,0.0);
    table.put(2.1,0.0);
    table.put(2.2,0.0);
    table.put(2.3,0.0);
    table.put(2.4,0.0);
    table.put(2.5,0.0);
    table.put(2.6,0.0);
    table.put(2.7,0.0);
    table.put(2.8,0.0);
    table.put(2.9,0.0);
    table.put(3.0,0.0);
    table.put(3.1,0.0);
    table.put(3.2,0.0);
    table.put(3.3,0.0);
    table.put(3.4,0.0);
    table.put(3.5,0.0);
    table.put(3.6,0.0);
    table.put(3.7,0.0);
    table.put(3.8,0.0);
    table.put(3.9,0.0);
    table.put(4.0,0.0);
    table.put(4.1,0.0);
    table.put(4.2,0.0);
    table.put(4.3,0.0);
    table.put(4.4,0.0);
    table.put(4.5,0.0);
    table.put(4.6,0.0);
    table.put(4.7,0.0);
    table.put(4.8,0.0);
    table.put(4.9,0.0);
    table.put(5.0,0.0);
    table.put(5.1,0.0);
    table.put(5.2,0.0);
    table.put(5.3,0.0);
    table.put(5.4,0.0);
    table.put(5.5,0.0);
    table.put(5.6,0.0);
    table.put(5.7,0.0);
    table.put(5.8,0.0);
    table.put(5.9,0.0);
    table.put(6.0,0.0);
    table.put(6.1,0.0);
    table.put(6.2,0.0);
    table.put(6.3,0.0);
    table.put(6.4,0.0);
    table.put(6.5,0.0);
    table.put(6.6,0.0);
    table.put(6.7,0.0);
    table.put(6.8,0.0);
    table.put(6.9,0.0);
    table.put(7.0,0.0);

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
