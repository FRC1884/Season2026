package org.Griffins1884.frc2026.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.Set;
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
    Translation2d distance2d;

    // Distances
    double distanceX;
    double distanceY;
    double distance;
    double yawAngle;

    // Angle/RPM
    double value;

    distance2d =
        new Translation2d(
            Math.abs(robot.getX() - target.getX()), Math.abs(robot.getY() - target.getY()));

    // Calculate the Straight line distance in (m) to the hub
    distanceX = robot.getX() - target.getX();
    distanceY = robot.getY() - target.getY();
    yawAngle = robot.getRotation().getDegrees();
    distanceX -= shooterDistanceCenter * Math.sin(Math.toRadians(yawAngle));
    distanceY += shooterDistanceCenter * Math.cos(Math.toRadians(yawAngle));

    distance =
        (double) Math.round(Math.hypot(Math.abs(distanceX), Math.abs(distanceY)) * 100) / 100;

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
    // Distance meters -> pivot angle radians. Add/edit your mapping here.
    table.put(2.0, 0.20);
    table.put(3.0, 0.35);
    table.put(4.0, 0.55);
    return table;
  }

  private static NavigableMap<Double, Double> buildRpmTable() {
    NavigableMap<Double, Double> table = new TreeMap<>();
    // Distance meters -> shooter RPM. Add/edit your mapping here.
    table.put(2.0, 2200.0);
    table.put(3.0, 2500.0);
    table.put(4.0, 3000.0);
    return table;
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
