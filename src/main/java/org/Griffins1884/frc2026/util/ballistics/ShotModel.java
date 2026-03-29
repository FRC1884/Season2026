package org.Griffins1884.frc2026.util.ballistics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Generic interface for configurable shot prediction and search. */
public interface ShotModel {
  ShotPrediction predict(ShotScenario scenario, LaunchCommand launchCommand);

  ShotSolution solve(ShotScenario scenario);

  String name();

  record ShotScenario(
      Translation3d targetPositionMeters, Translation2d robotVelocityMetersPerSecond) {
    public ShotScenario {
      targetPositionMeters =
          targetPositionMeters != null ? targetPositionMeters : new Translation3d();
      robotVelocityMetersPerSecond =
          robotVelocityMetersPerSecond != null ? robotVelocityMetersPerSecond : new Translation2d();
    }
  }

  record LaunchCommand(double wheelRpm, double launchAngleDegrees) {}

  record ShotPrediction(
      boolean feasible,
      double timeOfFlightSeconds,
      double launchAngleDegrees,
      double launchMotorRotations,
      double wheelRpm,
      double exitVelocityMetersPerSecond,
      double turretYawDegrees,
      double horizontalDistanceMeters,
      double closestApproachErrorMeters,
      Translation3d closestApproachPositionMeters,
      Translation3d targetPositionMeters,
      double robotRadialVelocityMetersPerSecond,
      double robotTangentialVelocityMetersPerSecond) {}

  record ShotSolution(LaunchCommand launchCommand, ShotPrediction prediction, int evaluations) {}
}
