package org.Griffins1884.frc2026.simulation.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterConstants;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotConstants;

/** Geometry and simple projectile constants used by the shot-math visualizer. */
public record ShotSimulationConfig(
    Translation3d turretMountMeters,
    ExitSample lowAngleExitSample,
    ExitSample highAngleExitSample,
    PivotCalibration pivotCalibration,
    FlywheelCalibration flywheelCalibration,
    PhysicsConfig physics,
    double integrationStepSeconds,
    double maxSimulationTimeSeconds) {

  public ShotSimulationConfig {
    turretMountMeters = turretMountMeters != null ? turretMountMeters : new Translation3d();
    lowAngleExitSample =
        lowAngleExitSample != null
            ? lowAngleExitSample
            : new ExitSample(27.0, 0.057400, 0.0, 0.670908);
    highAngleExitSample =
        highAngleExitSample != null
            ? highAngleExitSample
            : new ExitSample(60.0, 0.010047, 0.0, 0.625973);
    pivotCalibration =
        pivotCalibration != null
            ? pivotCalibration
            : new PivotCalibration(
                ShooterPivotConstants.SOFT_LIMIT_MIN,
                ShooterPivotConstants.SOFT_LIMIT_MAX,
                pivotPositionToLaunchAngleDegrees(ShooterPivotConstants.SOFT_LIMIT_MIN),
                pivotPositionToLaunchAngleDegrees(ShooterPivotConstants.SOFT_LIMIT_MAX));
    flywheelCalibration =
        flywheelCalibration != null
            ? flywheelCalibration
            : new FlywheelCalibration(
                ShooterConstants.FLYWHEEL_RADIUS_METERS,
                ShooterConstants.FLYWHEEL_GEAR_RATIO,
                ShooterConstants.SLIP_FACTOR.get());
    physics = physics != null ? physics : new PhysicsConfig(9.80665, 0.0, 0.0);
    integrationStepSeconds = integrationStepSeconds > 0.0 ? integrationStepSeconds : 0.05;
    maxSimulationTimeSeconds = maxSimulationTimeSeconds > 0.0 ? maxSimulationTimeSeconds : 5.0;
  }

  public static ShotSimulationConfig defaultConfig() {
    return new ShotSimulationConfig(
        new Translation3d(-0.0177, 0.0, 0.0),
        new ExitSample(27.0, 0.057400, 0.0, 0.670908),
        new ExitSample(60.0, 0.010047, 0.0, 0.625973),
        new PivotCalibration(
            ShooterPivotConstants.SOFT_LIMIT_MIN,
            ShooterPivotConstants.SOFT_LIMIT_MAX,
            pivotPositionToLaunchAngleDegrees(ShooterPivotConstants.SOFT_LIMIT_MIN),
            pivotPositionToLaunchAngleDegrees(ShooterPivotConstants.SOFT_LIMIT_MAX)),
        new FlywheelCalibration(
            ShooterConstants.FLYWHEEL_RADIUS_METERS,
            ShooterConstants.FLYWHEEL_GEAR_RATIO,
            ShooterConstants.SLIP_FACTOR.get()),
        new PhysicsConfig(9.80665, 0.0, 0.0),
        0.05,
        5.0);
  }

  public double launchAngleDegrees(double pivotMotorRotations) {
    return pivotCalibration.motorRotationsToLaunchAngleDegrees(pivotMotorRotations);
  }

  public double exitVelocityMetersPerSecond(double wheelRpm) {
    return flywheelCalibration.rpmToExitVelocity(wheelRpm);
  }

  public Translation3d shooterExitPositionMeters(
      double pivotMotorRotations, Rotation2d turretYawFromRobotForward) {
    double launchAngleDegrees = launchAngleDegrees(pivotMotorRotations);
    Translation3d localExit = exitPositionLocalMeters(launchAngleDegrees);
    Translation3d rotatedLocalExit = rotateAboutZ(localExit, turretYawFromRobotForward);
    return turretMountMeters.plus(rotatedLocalExit);
  }

  private Translation3d exitPositionLocalMeters(double launchAngleDegrees) {
    double lowAngle = lowAngleExitSample.launchAngleDegrees();
    double highAngle = highAngleExitSample.launchAngleDegrees();
    if (Math.abs(highAngle - lowAngle) < 1e-9) {
      return lowAngleExitSample.toTranslation3d();
    }
    double clampedAngle =
        clamp(launchAngleDegrees, Math.min(lowAngle, highAngle), Math.max(lowAngle, highAngle));
    double ratio = (clampedAngle - lowAngle) / (highAngle - lowAngle);
    return new Translation3d(
        lerp(lowAngleExitSample.localXMeters(), highAngleExitSample.localXMeters(), ratio),
        lerp(lowAngleExitSample.localYMeters(), highAngleExitSample.localYMeters(), ratio),
        lerp(lowAngleExitSample.localZMeters(), highAngleExitSample.localZMeters(), ratio));
  }

  private static Translation3d rotateAboutZ(Translation3d translation, Rotation2d yaw) {
    double cos = yaw.getCos();
    double sin = yaw.getSin();
    return new Translation3d(
        translation.getX() * cos - translation.getY() * sin,
        translation.getX() * sin + translation.getY() * cos,
        translation.getZ());
  }

  private static double lerp(double a, double b, double t) {
    return a + ((b - a) * t);
  }

  private static double pivotPositionToLaunchAngleDegrees(double pivotPosition) {
    double pivotDegreesFromVerticalDown = (11.875 * pivotPosition) + 18.0;
    return 90.0 - pivotDegreesFromVerticalDown;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  public record ExitSample(
      double launchAngleDegrees, double localXMeters, double localYMeters, double localZMeters) {
    public Translation3d toTranslation3d() {
      return new Translation3d(localXMeters, localYMeters, localZMeters);
    }
  }

  public record PivotCalibration(
      double minMotorRotations,
      double maxMotorRotations,
      double launchAngleAtMinMotorRotationsDegrees,
      double launchAngleAtMaxMotorRotationsDegrees) {

    public double motorRotationsToLaunchAngleDegrees(double motorRotations) {
      if (Math.abs(maxMotorRotations - minMotorRotations) < 1e-9) {
        return launchAngleAtMinMotorRotationsDegrees;
      }
      double clamped = clamp(motorRotations, minMotorRotations, maxMotorRotations);
      double ratio = (clamped - minMotorRotations) / (maxMotorRotations - minMotorRotations);
      return launchAngleAtMinMotorRotationsDegrees
          + ratio * (launchAngleAtMaxMotorRotationsDegrees - launchAngleAtMinMotorRotationsDegrees);
    }
  }

  public record FlywheelCalibration(double wheelRadiusMeters, double gearRatio, double slipFactor) {
    public double rpmToExitVelocity(double wheelRpm) {
      double wheelRotationsPerSecond = wheelRpm / 60.0;
      double tangentialVelocity = wheelRotationsPerSecond * 2.0 * Math.PI * wheelRadiusMeters;
      return tangentialVelocity * gearRatio * slipFactor;
    }
  }

  public record PhysicsConfig(
      double gravityMetersPerSecondSquared,
      double linearDragPerSecond,
      double quadraticDragPerMeter) {}
}
