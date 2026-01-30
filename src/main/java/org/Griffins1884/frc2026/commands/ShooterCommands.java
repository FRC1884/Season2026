package org.Griffins1884.frc2026.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ShooterCommands {
  private static final Map<Double, Double> lookupTable = interpolate();

  public static double calc(Pose2d robot) {
    // Distance Vector Calculation
    Translation2d distance2d;

    // Distances
    double distanceX;
    double distanceY;
    double distance;

    // Angle
    double theta;

    // Set hub pose based on alliance
    if (DriverStation.getAlliance().isEmpty()){
      return 0.0;
    }else{
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
          distance2d = new Translation2d(robot.getX() - 11.9, robot.getY() - 4.03);
        } else {
          distance2d = new Translation2d(robot.getX() - 4.63, robot.getY() - 4.03);
        }
    }
    // Calculate the Straight line distance in (m) to the hub
    distanceX = distance2d.getX();
    distanceY = distance2d.getY();

    distance = (double) Math.round(Math.hypot(distanceX, distanceY) * 10) / 10;

    theta = find(distance);
    return theta;
  }

  public static Map<Double, Double> lookupTable() {
    /*
     * Key: Distance From Hub (double)
     * Value: Angle (double) (0-90)
     * Keys are spaced out by increments of 0.1
     *
     * Closest point is 0m away
     * Farthest point is 6.2m
     * */

    // Calculated manually and entered

    Map<Double, Double> table = new HashMap<Double, Double>();

    // 0.0-0.9
    table.put(0.0, 0.0);
    table.put(0.1, 0.0);
    table.put(0.2, 0.0);
    table.put(0.3, 0.0);
    table.put(0.4, 0.0);
    table.put(0.5, 0.0);
    table.put(0.6, 0.0);
    table.put(0.7, 0.0);
    table.put(0.8, 0.0);
    table.put(0.9, 0.0);

    // 1.0-1.9
    table.put(1.0, 0.0);
    table.put(1.1, 0.0);
    table.put(1.2, 0.0);
    table.put(1.3, 0.0);
    table.put(1.4, 0.0);
    table.put(1.5, 0.0);
    table.put(1.6, 0.0);
    table.put(1.7, 0.0);
    table.put(1.8, 0.0);
    table.put(1.9, 0.0);

    // 2.0-2.9
    table.put(2.0, 0.0);
    table.put(2.1, 0.0);
    table.put(2.2, 0.0);
    table.put(2.3, 0.0);
    table.put(2.4, 0.0);
    table.put(2.5, 0.0);
    table.put(2.6, 0.0);
    table.put(2.7, 0.0);
    table.put(2.8, 0.0);
    table.put(2.9, 0.0);

    // 3.0-3.9
    table.put(3.0, 0.0);
    table.put(3.1, 0.0);
    table.put(3.2, 0.0);
    table.put(3.3, 0.0);
    table.put(3.4, 0.0);
    table.put(3.5, 0.0);
    table.put(3.6, 0.0);
    table.put(3.7, 0.0);
    table.put(3.8, 0.0);
    table.put(3.9, 0.0);

    // 4.0-4.9
    table.put(4.0, 0.0);
    table.put(4.1, 0.0);
    table.put(4.2, 0.0);
    table.put(4.3, 0.0);
    table.put(4.4, 0.0);
    table.put(4.5, 0.0);
    table.put(4.6, 0.0);
    table.put(4.7, 0.0);
    table.put(4.8, 0.0);
    table.put(4.9, 0.0);

    // 5.0-5.9
    table.put(5.0, 0.0);
    table.put(5.1, 0.0);
    table.put(5.2, 0.0);
    table.put(5.3, 0.0);
    table.put(5.4, 0.0);
    table.put(5.5, 0.0);
    table.put(5.6, 0.0);
    table.put(5.7, 0.0);
    table.put(5.8, 0.0);
    table.put(5.9, 0.0);

    // 6.0-6.9
    table.put(6.0, 0.0);
    table.put(6.1, 0.0);
    table.put(6.2, 0.0);

    return table;
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
    Map<Double, Double> temp = lookupTable();

    double[] x = new double[temp.size()];
    double[] y = new double[temp.size()];
    for (int i = 0; i < temp.size(); i++) {
        x[i] = (double) temp.keySet().toArray()[i];
        y[i] = (double) temp.keySet().toArray()[i];
    }

    SplineInterpolator interpolator = new SplineInterpolator();
    PolynomialSplineFunction function = interpolator.interpolate(x,y);

    // Calculated manually and entered
    Map<Double, Double> table = new HashMap<Double, Double>();

    for (double i=0;i<=6.2;i+=0.1){
        table.put(i, function.value(i));
    }

    return table;
  }

  public static double find(double distance) {
    try {
      return lookupTable.get(distance);
    } catch (Exception e) {
      if (distance > 6.2) {
        return lookupTable.get(6.2);
      } else if (distance < 0.0) {
        return lookupTable.get(0.0);
      }
    }
    return 0.0;
  }
}