package org.Griffins1884.frc2026.util.ballistics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Generic interface for configurable shot prediction and search. */
public interface ShotModel {
  ShotPrediction predict(ShotScenario scenario, LaunchCommand launchCommand);

  ShotSolution solve(ShotScenario scenario);

  String name();

  record ShotScenario(
      Translation3d targetPositionMeters,
      Translation2d robotVelocityMetersPerSecond,
      EntryWindow entryWindow,
      ClearanceConstraint clearanceConstraint) {
    public ShotScenario {
      targetPositionMeters =
          targetPositionMeters != null ? targetPositionMeters : new Translation3d();
      robotVelocityMetersPerSecond =
          robotVelocityMetersPerSecond != null ? robotVelocityMetersPerSecond : new Translation2d();
      entryWindow = entryWindow != null ? entryWindow : EntryWindow.unconstrained();
      clearanceConstraint =
          clearanceConstraint != null ? clearanceConstraint : ClearanceConstraint.unconstrained();
    }

    public ShotScenario(
        Translation3d targetPositionMeters, Translation2d robotVelocityMetersPerSecond) {
      this(targetPositionMeters, robotVelocityMetersPerSecond, null, null);
    }
  }

  record EntryWindow(
      double horizontalRadiusMeters,
      boolean requireDescendingAtEntry,
      double preferredUpperEntryHeightMeters,
      double preferredUpperEntryRadiusMeters,
      boolean requirePreferredUpperEntry) {
    public EntryWindow {
      horizontalRadiusMeters = Math.max(0.0, horizontalRadiusMeters);
      preferredUpperEntryRadiusMeters = Math.max(0.0, preferredUpperEntryRadiusMeters);
    }

    public EntryWindow(double horizontalRadiusMeters, boolean requireDescendingAtEntry) {
      this(
          horizontalRadiusMeters,
          requireDescendingAtEntry,
          Double.NaN,
          Double.POSITIVE_INFINITY,
          false);
    }

    public static EntryWindow unconstrained() {
      return new EntryWindow(
          Double.POSITIVE_INFINITY, false, Double.NaN, Double.POSITIVE_INFINITY, false);
    }
  }

  record ClearanceConstraint(
      Translation3d startMeters,
      double startRadiusMeters,
      Translation3d endMeters,
      double endRadiusMeters) {
    public ClearanceConstraint {
      startMeters = startMeters != null ? startMeters : new Translation3d();
      endMeters = endMeters != null ? endMeters : startMeters;
      startRadiusMeters = Math.max(0.0, startRadiusMeters);
      endRadiusMeters = Math.max(0.0, endRadiusMeters);
    }

    public ClearanceConstraint(Translation3d centerMeters, double radiusMeters) {
      this(centerMeters, radiusMeters, centerMeters, radiusMeters);
    }

    public static ClearanceConstraint unconstrained() {
      return new ClearanceConstraint(new Translation3d(), 0.0, new Translation3d(), 0.0);
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
      double clearanceMarginMeters,
      Translation3d closestApproachPositionMeters,
      Translation3d targetPositionMeters,
      double robotRadialVelocityMetersPerSecond,
      double robotTangentialVelocityMetersPerSecond) {}

  record ShotSolution(LaunchCommand launchCommand, ShotPrediction prediction, int evaluations) {}
}
