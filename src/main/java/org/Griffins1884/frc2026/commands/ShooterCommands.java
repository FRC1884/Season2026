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
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ShooterCommands {
  private static final double GRAVITY = 9.80665;
  private static final NavigableMap<Double, Double> lookupTable = new TreeMap<>(interpolate());
  private static final double hubRadius = 0.60;
  private static final double shooterDistanceEdge = 0.37; // Todo: Tune
  private static final double shooterDistanceCenter = 0.02; // Todo: Tune

  private static final double segment1Start = 0.97;
  private static final double segment1End = 2.98;
  private static final double segment2Start = 2.99;
  private static final double segment2End = 4.04;
  private static final double segment3Start = 4.05;
  private static final double segment3End = 6.20;

  private enum Segment {
    SEGMENT_1,
    SEGMENT_2,
    SEGMENT_3;
  }

  public enum Vals {
    RPM,
    ANGLE
  }

  private static Segment currentSegment = Segment.SEGMENT_1;
  private static final double minLookupDistance = shooterDistanceEdge + hubRadius + segment1Start;
  private static final double maxLookupDistance = shooterDistanceEdge + hubRadius + segment3End;

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

  public static List<Map<Double, Double>> lookupTable() {
    /*
     * Key: Distance From Hub (double)
     * Value: Angle (double)
     * Value: Velocity (double, but technically integer)
     * Keys are spaced out by increments of 0.1
     *
     * Closest point is 0m away
     * Farthest point is 6.2m
     * */

    // Calculated manually and entered

    Map<Double, Double> segment1 = new HashMap<>(); // THETA:0.1
    Map<Double, Double> segment2 = new HashMap<>(); // RPM:2000
    Map<Double, Double> segment3 = new HashMap<>(); // RPM:2500

    segment1.put(0.9652019304, (double) 1638);
    segment1.put(1.092202184, (double) 1650);
    segment1.put(1.219202438, (double) 1670);
    segment1.put(1.346202692, (double) 1685);
    segment1.put(1.473202946, (double) 1730);
    segment1.put(1.6002032, (double) 1750);
    segment1.put(1.727203454, (double) 1760);
    segment1.put(1.854203708, (double) 1770);
    segment1.put(1.981203962, (double) 1836);
    segment1.put(2.108204216, (double) 1855);
    segment1.put(2.23520447, (double) 1895);
    segment1.put(2.362204724, (double) 1915);
    segment1.put(2.489204978, (double) 1935);
    segment1.put(2.616205232, (double) 1955);
    segment1.put(2.743205486, (double) 1961);
    segment1.put(2.87020574, (double) 1970);
    segment1.put(2.984505969, (double) 2000);

    segment2.put(2.997205994, 0.18);
    segment2.put(3.124206248, 0.2);
    segment2.put(3.251206502, 0.25);
    segment2.put(3.378206756, 0.28);
    segment2.put(3.50520701, 0.38);
    segment2.put(3.632207264, 0.42);
    segment2.put(3.759207518, 0.48);
    segment2.put(3.886207772, 0.53);
    segment2.put(4.013208026, 0.57);
    segment2.put(4.038608077, 0.6);

    segment3.put(4.051308103, 0.1);
    segment3.put(4.14020828, 0.1);
    segment3.put(4.267208534, 0.25);
    segment3.put(4.394208788, 0.3);
    segment3.put(4.521209042, 0.3);
    segment3.put(4.648209296, 0.31);
    segment3.put(4.77520955, 0.34);
    segment3.put(4.902209804, 0.37);
    segment3.put(5.029210058, 0.45);
    segment3.put(5.156210312, 0.4);
    segment3.put(5.283210566, 0.6);
    segment3.put(5.41021082, 0.7);
    segment3.put(5.537211074, 0.74);
    segment3.put(5.664211328, 0.97);
    segment3.put(5.791211582, 0.97);
    segment3.put(5.918211836, 1.2);
    segment3.put(6.04521209, 1.21);
    segment3.put(6.197612395, 1.25);

    List<Map<Double, Double>> temp = new ArrayList<>(List.of(segment1, segment2, segment3));

    return temp;
  }

  /*
   * Key: Distance From Hub (double)
   * Value: Angle (double) (0-90)
   * Keys are spaced out by increments of 0.1
   *
   * Closest point is 0m away
   * Farthest point is 6.2m
   */
  public static Map<Double, Double> interpolate() {
    Map<Double, Double> table = new HashMap<>();

    for (int i = 0; i < lookupTable().size(); i++) {
      Map<Double, Double> temp = lookupTable().get(i);
      Set<Double> lookupTable = temp.keySet();
      List<Double> sortedKeys = new ArrayList<>(lookupTable);
      Collections.sort(sortedKeys);

      double[] x = new double[lookupTable.size()];
      double[] y = new double[lookupTable.size()];

      for (int j = 0; j < lookupTable.size(); j++) {
        x[j] = sortedKeys.get(j);
        y[j] = temp.get(sortedKeys.get(j));
      }
      SplineInterpolator interpolator = new SplineInterpolator();
      PolynomialSplineFunction function = interpolator.interpolate(x, y);
      double domainStart = sortedKeys.get(0);
      double domainEnd = sortedKeys.get(sortedKeys.size() - 1);

      if (i == 0) {
        double sampleStart = Math.max(segment1Start, domainStart);
        double sampleEnd = Math.min(segment1End, domainEnd);
        for (double j = sampleStart; j <= sampleEnd + 1e-9; j += 0.01) {
          double sample = Math.min(j, sampleEnd);
          table.put(sample + shooterDistanceEdge + hubRadius, function.value(sample));
        }
      } else if (i == 1) {
        double sampleStart = Math.max(segment2Start, domainStart);
        double sampleEnd = Math.min(segment2End, domainEnd);
        for (double j = sampleStart; j <= sampleEnd + 1e-9; j += 0.01) {
          double sample = Math.min(j, sampleEnd);
          table.put(sample + shooterDistanceEdge + hubRadius, function.value(sample));
        }
      } else if (i == 2) {
        double sampleStart = Math.max(segment3Start, domainStart);
        double sampleEnd = Math.min(segment3End, domainEnd);
        for (double j = sampleStart; j <= sampleEnd + 1e-9; j += 0.01) {
          double sample = Math.min(j, sampleEnd);
          table.put(sample + shooterDistanceEdge + hubRadius, function.value(sample));
        }
      }
    }
    return table;
  }

  public static boolean isAngle(double distance) {
    if (distance <= segment1End) {
      return false;
    } else {
      return true;
    }
  }

  public static void setCurrentSegment(double distance) {
    if (!isAngle(distance)) {
      currentSegment = Segment.SEGMENT_1;
    } else {
      if (distance <= shooterDistanceEdge + hubRadius + segment2End) {
        currentSegment = Segment.SEGMENT_2;
      } else if (distance <= shooterDistanceEdge + hubRadius + segment3End) {
        currentSegment = Segment.SEGMENT_3;
      } else {
        currentSegment = Segment.SEGMENT_3;
      }
    }
  }

  private static double lookupValueOrNearest(double distance) {
    if (lookupTable.isEmpty()) {
      return 0.1;
    }
    var floor = lookupTable.floorEntry(distance);
    var ceil = lookupTable.ceilingEntry(distance);
    if (floor == null) {
      return ceil.getValue();
    }
    if (ceil == null) {
      return floor.getValue();
    }
    if (Double.compare(floor.getKey(), ceil.getKey()) == 0) {
      return floor.getValue();
    }
    double t = (distance - floor.getKey()) / (ceil.getKey() - floor.getKey());
    return floor.getValue() + (ceil.getValue() - floor.getValue()) * t;
  }

  public static double getRPM(double distance) {
    setCurrentSegment(distance);
    if (currentSegment == Segment.SEGMENT_1) {
      return lookupValueOrNearest(distance);
    } else if (currentSegment == Segment.SEGMENT_2) {
      return 2000.0;
    } else if (currentSegment == Segment.SEGMENT_3) {
      return 2500.0;
    } else {
      return 0.0;
    }
  }

  public static double getBestAngle(double distance) {
    setCurrentSegment(distance);
    if (currentSegment == Segment.SEGMENT_1) {
      return 0.1;
    } else if (currentSegment == Segment.SEGMENT_2) {
      return lookupValueOrNearest(segment2Start + hubRadius + shooterDistanceEdge);
    } else if (currentSegment == Segment.SEGMENT_3) {
      return lookupValueOrNearest(segment3Start + hubRadius + shooterDistanceEdge);
    }
    return 0.1;
  }

  public static double getValue(double distance) {
    distance = (double) Math.round(distance * 100) / 100;
    if (!Double.isNaN(distance) && !isAngle(distance)) {
      return lookupValueOrNearest(distance);
    } else {
      if (Double.isNaN(distance)) {
        return 0.1;
      } else if (distance > maxLookupDistance) {
        return lookupValueOrNearest(maxLookupDistance);
      } else if (distance < minLookupDistance) {
        return 0.1;
      } else if (isAngle(distance)) {
        return getBestAngle(distance);
      }
      return 0.1;
    }
  }

  public static Map<Vals, Double> dataPack(double distance) {
    Map<Vals, Double> data = new HashMap<>();
    data.put(Vals.RPM, 4500.0); // getRPM(distance));
    data.put(Vals.ANGLE, getValue(distance));
    return data;
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
            getRPM(distanceMeters),
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
