package org.Griffins1884.frc2026.util.ballistics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class AdvancedBallisticsShotModelTest {
  @Test
  void pivotCalibrationRoundTripsLaunchAngle() {
    ShotModelConfig config = ShotModelConfig.defaultConfig();
    double launchAngleDegrees = 41.5;

    double motorRotations =
        config.pivotCalibration().launchAngleDegreesToMotorRotations(launchAngleDegrees);
    double roundTripDegrees =
        config.pivotCalibration().motorRotationsToLaunchAngleDegrees(motorRotations);

    assertEquals(launchAngleDegrees, roundTripDegrees, 1e-9);
  }

  @Test
  void pivotCalibrationSupportsDecreasingAngleWithMoreRotations() {
    ShotModelConfig config = ShotModelConfig.defaultConfig();

    assertEquals(60.0, config.launchAngleDegrees(0.0), 1e-9);
    assertEquals(27.0, config.launchAngleDegrees(1.6), 1e-9);
    assertEquals(0.8, config.pivotCalibration().launchAngleDegreesToMotorRotations(43.5), 1e-9);
  }

  @Test
  void shooterExitRotatesWithTurretYaw() {
    ShotModelConfig config = ShotModelConfig.defaultConfig();

    Translation3d forwardExit =
        config.shooterExitPositionMetersForLaunchAngle(60.0, new Rotation2d());
    Translation3d leftExit =
        config.shooterExitPositionMetersForLaunchAngle(60.0, Rotation2d.fromDegrees(90.0));

    assertEquals(-0.0177 + 0.010047, forwardExit.getX(), 1e-9);
    assertEquals(0.0, forwardExit.getY(), 1e-9);
    assertEquals(-0.0177, leftExit.getX(), 1e-9);
    assertEquals(0.010047, leftExit.getY(), 1e-9);
    assertEquals(forwardExit.getZ(), leftExit.getZ(), 1e-9);
  }

  @Test
  void solverFindsReasonableStationaryShot() {
    AdvancedBallisticsShotModel model =
        new AdvancedBallisticsShotModel(ShotModelConfig.defaultConfig());
    ShotModel.ShotScenario scenario =
        new ShotModel.ShotScenario(new Translation3d(4.0, 0.0, 2.05), new Translation2d(0.0, 0.0));

    ShotModel.ShotSolution solution = model.solve(scenario);

    assertTrue(solution.evaluations() > 0);
    assertTrue(solution.prediction().closestApproachErrorMeters() < 0.75);
    assertTrue(solution.prediction().timeOfFlightSeconds() > 0.0);
  }
}
