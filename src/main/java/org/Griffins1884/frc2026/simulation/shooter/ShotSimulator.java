package org.Griffins1884.frc2026.simulation.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/** Converts shot-math setpoints into field-space release poses, arcs, and projectile spawns. */
public final class ShotSimulator {
  private static final int MAX_SAMPLE_COUNT = 96;

  private final ShotSimulationConfig config;

  public ShotSimulator(ShotSimulationConfig config) {
    this.config = config != null ? config : ShotSimulationConfig.defaultConfig();
  }

  public Optional<SimulatedShot> solveHubShot(
      Pose2d robotPose,
      Translation2d fieldVelocity,
      Rotation2d turretYaw,
      double pivotMotorRotations,
      double shooterRpm,
      Translation3d fieldTargetPosition,
      Translation3d fieldConeTopPosition,
      double openingRadiusMeters,
      double topOpeningRadiusMeters,
      double coneClearanceMeters) {
    if (robotPose == null
        || fieldTargetPosition == null
        || fieldConeTopPosition == null
        || turretYaw == null
        || !Double.isFinite(robotPose.getX())
        || !Double.isFinite(robotPose.getY())
        || !Double.isFinite(shooterRpm)
        || shooterRpm <= 1.0) {
      return Optional.empty();
    }

    Translation2d sanitizedVelocity = fieldVelocity != null ? fieldVelocity : new Translation2d();
    Rotation2d robotHeading = robotPose.getRotation();
    Translation3d robotRelativeExit =
        config.shooterExitPositionMeters(pivotMotorRotations, turretYaw);
    Translation3d fieldExit = toFieldTranslation(robotPose, robotRelativeExit);

    double exitVelocityMetersPerSecond = config.exitVelocityMetersPerSecond(shooterRpm);
    double launchAngleRadians = Math.toRadians(config.launchAngleDegrees(pivotMotorRotations));
    double horizontalSpeed = exitVelocityMetersPerSecond * Math.cos(launchAngleRadians);
    Translation2d robotRelativeLaunchVelocity =
        new Translation2d(horizontalSpeed, 0.0).rotateBy(turretYaw);
    Translation2d fieldLaunchVelocity =
        robotRelativeLaunchVelocity.rotateBy(robotHeading).plus(sanitizedVelocity);
    Translation3d initialFieldVelocity =
        new Translation3d(
            fieldLaunchVelocity.getX(),
            fieldLaunchVelocity.getY(),
            exitVelocityMetersPerSecond * Math.sin(launchAngleRadians));

    Pose3d releasePose =
        new Pose3d(
            fieldExit,
            new Rotation3d(0.0, -launchAngleRadians, robotHeading.plus(turretYaw).getRadians()));
    Pose3d[] samples = sampleTrajectory(fieldExit, initialFieldVelocity);
    PredictionMetrics metrics =
        evaluatePrediction(
            samples,
            fieldTargetPosition,
            fieldConeTopPosition,
            openingRadiusMeters,
            topOpeningRadiusMeters,
            coneClearanceMeters);
    Pose3d impactPose = samples.length > 0 ? samples[samples.length - 1] : releasePose;
    return Optional.of(
        new SimulatedShot(
            metrics.feasible(),
            releasePose,
            initialFieldVelocity,
            samples,
            impactPose,
            metrics.closestApproachErrorMeters(),
            metrics.timeOfFlightSeconds()));
  }

  private Pose3d[] sampleTrajectory(
      Translation3d initialPosition, Translation3d initialVelocityMetersPerSecond) {
    List<Pose3d> samples = new ArrayList<>();
    ProjectileState sample =
        new ProjectileState(initialPosition, initialVelocityMetersPerSecond, 0.0);
    double dtSeconds = config.integrationStepSeconds();
    int maxSamples =
        Math.min(
            MAX_SAMPLE_COUNT, (int) Math.ceil(config.maxSimulationTimeSeconds() / dtSeconds) + 1);
    for (int i = 0; i < maxSamples; i++) {
      samples.add(sample.pose());
      if (!sample.active()) {
        break;
      }
      sample.advance(config.physics(), dtSeconds);
    }
    return samples.toArray(Pose3d[]::new);
  }

  private PredictionMetrics evaluatePrediction(
      Pose3d[] samples,
      Translation3d fieldTargetPosition,
      Translation3d fieldConeTopPosition,
      double openingRadiusMeters,
      double topOpeningRadiusMeters,
      double coneClearanceMeters) {
    if (samples == null || samples.length == 0) {
      return new PredictionMetrics(false, Double.NaN, Double.NaN);
    }

    double bestDistance = Double.POSITIVE_INFINITY;
    double bestTimeSeconds = 0.0;
    boolean feasible = false;
    double dtSeconds = config.integrationStepSeconds();
    double effectiveTopRadius =
        Math.max(0.0, topOpeningRadiusMeters - Math.max(0.0, coneClearanceMeters));
    double lowerZ = fieldTargetPosition.getZ();
    double upperZ = fieldConeTopPosition.getZ();
    double zSpan = Math.max(upperZ - lowerZ, 1e-9);

    for (int i = 0; i < samples.length; i++) {
      Translation3d samplePoint = samples[i].getTranslation();
      double distanceToCenter = samplePoint.getDistance(fieldTargetPosition);
      if (distanceToCenter < bestDistance) {
        bestDistance = distanceToCenter;
        bestTimeSeconds = i * dtSeconds;
      }

      double z = samplePoint.getZ();
      if (z < lowerZ || z > upperZ) {
        continue;
      }

      double interpolation = (z - lowerZ) / zSpan;
      double allowedRadius =
          openingRadiusMeters + ((effectiveTopRadius - openingRadiusMeters) * interpolation);
      double horizontalDistance =
          samplePoint.toTranslation2d().getDistance(fieldTargetPosition.toTranslation2d());
      if (horizontalDistance <= allowedRadius) {
        feasible = true;
      }
    }

    return new PredictionMetrics(feasible, bestDistance, bestTimeSeconds);
  }

  private static Translation3d toFieldTranslation(
      Pose2d robotPose, Translation3d robotRelativeTranslation) {
    Translation2d fieldXY =
        new Translation2d(robotRelativeTranslation.getX(), robotRelativeTranslation.getY())
            .rotateBy(robotPose.getRotation())
            .plus(robotPose.getTranslation());
    return new Translation3d(fieldXY.getX(), fieldXY.getY(), robotRelativeTranslation.getZ());
  }

  private record PredictionMetrics(
      boolean feasible, double closestApproachErrorMeters, double timeOfFlightSeconds) {}
}
