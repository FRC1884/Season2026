package org.Griffins1884.frc2026.util.ballistics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Geometry, calibration, physics, and search parameters for shot modeling.
 *
 * <p>This version fixes two issues with the old config:
 *
 * <ol>
 *   <li>Pivot calibration supports mechanisms where launch angle decreases as motor rotations
 *       increase.
 *   <li>The shooter exit point is no longer treated as fixed. It is modeled as:
 *       <ul>
 *         <li>changing with pivot angle, and
 *         <li>rotating with turret yaw around the turret center.
 *       </ul>
 * </ol>
 *
 * <p>Coordinate convention assumed here:
 *
 * <ul>
 *   <li>+X = forward
 *   <li>+Y = left
 *   <li>+Z = up
 * </ul>
 */
public record ShotModelConfig(
    MountPose turretMount,
    ExitGeometry exitGeometry,
    PivotCalibration pivotCalibration,
    FlywheelConfig flywheel,
    PhysicsConfig physics,
    SearchConfig search) {

  public ShotModelConfig {
    turretMount = turretMount != null ? turretMount : new MountPose(0.0, 0.0, 0.0);

    exitGeometry =
        exitGeometry != null
            ? exitGeometry
            : new ExitGeometry(
                new ExitSample(60.0, 0.010047, 0.0, 0.625973),
                new ExitSample(27.0, 0.057400, 0.0, 0.670908));

    pivotCalibration =
        pivotCalibration != null ? pivotCalibration : new PivotCalibration(0.0, 1.6, 60.0, 27.0);

    flywheel = flywheel != null ? flywheel : new FlywheelConfig(0.0508, 1.0, 0.83, 2000.0, 5500.0);

    physics = physics != null ? physics : new PhysicsConfig(9.80665, 0.0, 0.0);

    search = search != null ? search : new SearchConfig(250.0, 1.0, 0.08, 5.0, 0.05);
  }

  /**
   * Competition/default config from the numbers you provided.
   *
   * <p>Inputs used:
   *
   * <ul>
   *   <li>Robot center = (0,0,0)
   *   <li>Turret center = -17.7 mm forward from robot center
   *   <li>At 60 deg launch: exit = (-7.653 mm, 0, 625.973 mm) in robot coordinates
   *   <li>At 27 deg launch: exit = (39.7 mm, 0, 670.908 mm) in robot coordinates
   * </ul>
   *
   * <p>Relative-to-turret exit points therefore become:
   *
   * <ul>
   *   <li>60 deg: (10.047 mm, 0, 625.973 mm)
   *   <li>27 deg: (57.400 mm, 0, 670.908 mm)
   * </ul>
   */
  public static ShotModelConfig defaultConfig() {
    return new ShotModelConfig(
        new MountPose(-0.0177, 0.0, 0.0),
        new ExitGeometry(
            new ExitSample(60.0, 0.010047, 0.0, 0.625973),
            new ExitSample(27.0, 0.057400, 0.0, 0.670908)),
        new PivotCalibration(0.0, 1.6, 60.0, 27.0),
        new FlywheelConfig(0.0508, 1.0, 0.83, 2000.0, 5500.0),
        new PhysicsConfig(9.80665, 0.0, 0.0),
        new SearchConfig(250.0, 1.0, 0.08, 5.0, 0.05));
  }

  /**
   * Returns shooter exit position in robot coordinates for the provided motor rotations and turret
   * yaw.
   *
   * <p>The exit point:
   *
   * <ol>
   *   <li>is interpolated from launch angle / pivot state, in turret-local coordinates
   *   <li>is then rotated about turret Z by turretYaw
   *   <li>is then translated by turretMount into robot coordinates
   * </ol>
   */
  public Translation3d shooterExitPositionMeters(
      double pivotMotorRotations, Rotation2d turretYawFromRobotForward) {
    double launchAngleDeg =
        pivotCalibration.motorRotationsToLaunchAngleDegrees(pivotMotorRotations);
    return shooterExitPositionMetersForLaunchAngle(launchAngleDeg, turretYawFromRobotForward);
  }

  /**
   * Returns shooter exit position in robot coordinates for a desired launch angle and turret yaw.
   */
  public Translation3d shooterExitPositionMetersForLaunchAngle(
      double launchAngleDegrees, Rotation2d turretYawFromRobotForward) {
    Translation3d localExit = exitGeometry.exitPositionLocalMeters(launchAngleDegrees);
    Translation3d rotatedLocalExit = rotateAboutZ(localExit, turretYawFromRobotForward);
    return turretMount.toTranslation3d().plus(rotatedLocalExit);
  }

  /** Returns the local (turret-frame) exit position for the given pivot motor rotations. */
  public Translation3d shooterExitLocalPositionMeters(double pivotMotorRotations) {
    double launchAngleDeg =
        pivotCalibration.motorRotationsToLaunchAngleDegrees(pivotMotorRotations);
    return exitGeometry.exitPositionLocalMeters(launchAngleDeg);
  }

  /** Returns the launch angle for a pivot motor position. */
  public double launchAngleDegrees(double pivotMotorRotations) {
    return pivotCalibration.motorRotationsToLaunchAngleDegrees(pivotMotorRotations);
  }

  /** Returns exit velocity estimate for wheel RPM. */
  public double exitVelocityMetersPerSecond(double wheelRpm) {
    return flywheel.rpmToExitVelocity(wheelRpm);
  }

  private static Translation3d rotateAboutZ(Translation3d translation, Rotation2d yaw) {
    double cos = yaw.getCos();
    double sin = yaw.getSin();

    double x = translation.getX() * cos - translation.getY() * sin;
    double y = translation.getX() * sin + translation.getY() * cos;
    double z = translation.getZ();

    return new Translation3d(x, y, z);
  }

  public record MountPose(double xMeters, double yMeters, double zMeters) {
    public Translation3d toTranslation3d() {
      return new Translation3d(xMeters, yMeters, zMeters);
    }
  }

  /**
   * Maps pivot motor rotations to launch angle.
   *
   * <p>This supports both increasing and decreasing angle-vs-rotations relationships.
   */
  public record PivotCalibration(
      double minMotorRotations,
      double maxMotorRotations,
      double launchAngleAtMinMotorRotationsDegrees,
      double launchAngleAtMaxMotorRotationsDegrees) {

    public double clampMotorRotations(double motorRotations) {
      return Math.max(minMotorRotations, Math.min(maxMotorRotations, motorRotations));
    }

    public double minLaunchAngleDegrees() {
      return Math.min(launchAngleAtMinMotorRotationsDegrees, launchAngleAtMaxMotorRotationsDegrees);
    }

    public double maxLaunchAngleDegrees() {
      return Math.max(launchAngleAtMinMotorRotationsDegrees, launchAngleAtMaxMotorRotationsDegrees);
    }

    public double clampLaunchAngleDegrees(double launchAngleDegrees) {
      return Math.max(
          minLaunchAngleDegrees(), Math.min(maxLaunchAngleDegrees(), launchAngleDegrees));
    }

    public double motorRotationsToLaunchAngleDegrees(double motorRotations) {
      if (Math.abs(maxMotorRotations - minMotorRotations) < 1e-9) {
        return launchAngleAtMinMotorRotationsDegrees;
      }

      double clamped = clampMotorRotations(motorRotations);
      double ratio = (clamped - minMotorRotations) / (maxMotorRotations - minMotorRotations);

      return launchAngleAtMinMotorRotationsDegrees
          + ratio * (launchAngleAtMaxMotorRotationsDegrees - launchAngleAtMinMotorRotationsDegrees);
    }

    public double launchAngleDegreesToMotorRotations(double launchAngleDegrees) {
      double clamped = clampLaunchAngleDegrees(launchAngleDegrees);
      double angleSpan =
          launchAngleAtMaxMotorRotationsDegrees - launchAngleAtMinMotorRotationsDegrees;

      if (Math.abs(angleSpan) < 1e-9) {
        return minMotorRotations;
      }

      double ratio = (clamped - launchAngleAtMinMotorRotationsDegrees) / angleSpan;
      double motorRotations = minMotorRotations + ratio * (maxMotorRotations - minMotorRotations);

      return clampMotorRotations(motorRotations);
    }
  }

  /**
   * A measured local shooter exit point corresponding to a particular launch angle.
   *
   * <p>These coordinates are relative to the turret frame before turret yaw rotation is applied.
   */
  public record ExitSample(
      double launchAngleDegrees, double localXMeters, double localYMeters, double localZMeters) {

    public Translation3d toTranslation3d() {
      return new Translation3d(localXMeters, localYMeters, localZMeters);
    }
  }

  /**
   * Models how the shooter exit point moves with pivot angle in the turret frame.
   *
   * <p>This is currently a linear interpolation between two measured samples. That is a much better
   * approximation than a fixed exit point, and turret rotation is then handled by rotating the
   * local point into robot coordinates.
   */
  public record ExitGeometry(ExitSample sampleA, ExitSample sampleB) {

    public ExitGeometry {
      sampleA = sampleA != null ? sampleA : new ExitSample(60.0, 0.010047, 0.0, 0.625973);
      sampleB = sampleB != null ? sampleB : new ExitSample(27.0, 0.057400, 0.0, 0.670908);
    }

    public double minLaunchAngleDegrees() {
      return Math.min(sampleA.launchAngleDegrees(), sampleB.launchAngleDegrees());
    }

    public double maxLaunchAngleDegrees() {
      return Math.max(sampleA.launchAngleDegrees(), sampleB.launchAngleDegrees());
    }

    public double clampLaunchAngleDegrees(double launchAngleDegrees) {
      return Math.max(
          minLaunchAngleDegrees(), Math.min(maxLaunchAngleDegrees(), launchAngleDegrees));
    }

    public Translation3d exitPositionLocalMeters(double launchAngleDegrees) {
      double clampedAngle = clampLaunchAngleDegrees(launchAngleDegrees);
      double angleA = sampleA.launchAngleDegrees();
      double angleB = sampleB.launchAngleDegrees();

      if (Math.abs(angleB - angleA) < 1e-9) {
        return sampleA.toTranslation3d();
      }

      double ratio = (clampedAngle - angleA) / (angleB - angleA);

      double x = lerp(sampleA.localXMeters(), sampleB.localXMeters(), ratio);
      double y = lerp(sampleA.localYMeters(), sampleB.localYMeters(), ratio);
      double z = lerp(sampleA.localZMeters(), sampleB.localZMeters(), ratio);

      return new Translation3d(x, y, z);
    }

    private static double lerp(double a, double b, double t) {
      return a + (b - a) * t;
    }
  }

  public record FlywheelConfig(
      double radiusMeters,
      double gearRatio,
      double slipFactor,
      double minWheelRpm,
      double maxWheelRpm) {

    public double clampWheelRpm(double wheelRpm) {
      return Math.max(minWheelRpm, Math.min(maxWheelRpm, wheelRpm));
    }

    public double rpmToExitVelocity(double wheelRpm) {
      return (clampWheelRpm(wheelRpm) / 60.0)
          * (2.0 * Math.PI)
          * radiusMeters
          * gearRatio
          * slipFactor;
    }
  }

  public record PhysicsConfig(
      double gravityMetersPerSecondSquared,
      double linearDragPerSecond,
      double quadraticDragPerMeter) {}

  public record SearchConfig(
      double rpmStep,
      double launchAngleStepDegrees,
      double integrationStepSeconds,
      double maxSimulationTimeSeconds,
      double solutionToleranceMeters) {}
}
