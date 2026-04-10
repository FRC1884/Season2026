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
        new Translation2d(0.0, horizontalSpeed).rotateBy(turretYaw.unaryMinus());
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
            metrics.timeOfFlightSeconds(),
            metrics.clearsTop(),
            metrics.descendsIntoBottom(),
            metrics.topClearanceMeters(),
            metrics.bottomEntryErrorMeters(),
            metrics.topClearancePose(),
            metrics.bottomEntryPose()));
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
      return new PredictionMetrics(
          false, Double.NaN, Double.NaN, false, false, Double.NaN, Double.NaN, null, null);
    }

    double bestDistance = Double.POSITIVE_INFINITY;
    double bestTimeSeconds = 0.0;
    double dtSeconds = config.integrationStepSeconds();
    double effectiveTopRadius =
        Math.max(0.0, topOpeningRadiusMeters - Math.max(0.0, coneClearanceMeters));
    TopClearanceResult topClearance =
        evaluateTopClearance(
            samples, fieldTargetPosition, fieldConeTopPosition, effectiveTopRadius);
    BottomEntryResult bottomEntry =
        evaluateBottomEntry(samples, fieldTargetPosition, openingRadiusMeters);

    for (int i = 0; i < samples.length; i++) {
      Translation3d samplePoint = samples[i].getTranslation();
      double distanceToCenter = samplePoint.getDistance(fieldTargetPosition);
      if (distanceToCenter < bestDistance) {
        bestDistance = distanceToCenter;
        bestTimeSeconds = i * dtSeconds;
      }
    }

    boolean scoringValid =
        topClearance.clearsTop()
            && bottomEntry.descendsIntoBottom()
            && topClearance.elapsedSeconds() < bottomEntry.elapsedSeconds();
    return new PredictionMetrics(
        scoringValid,
        bestDistance,
        bestTimeSeconds,
        topClearance.clearsTop(),
        bottomEntry.descendsIntoBottom(),
        topClearance.clearanceMeters(),
        bottomEntry.entryErrorMeters(),
        topClearance.pose(),
        bottomEntry.pose());
  }

  private TopClearanceResult evaluateTopClearance(
      Pose3d[] samples,
      Translation3d fieldTargetPosition,
      Translation3d fieldConeTopPosition,
      double effectiveTopRadiusMeters) {
    Translation2d release = samples[0].getTranslation().toTranslation2d();
    Translation2d target = fieldTargetPosition.toTranslation2d();
    Translation2d releaseToTarget = target.minus(release);
    double distance = releaseToTarget.getNorm();
    if (distance <= 1e-9) {
      Pose3d pose = samples[0];
      return new TopClearanceResult(false, pose.getZ() - fieldConeTopPosition.getZ(), 0.0, pose);
    }
    Translation2d direction = releaseToTarget.div(distance);
    double topEntryDistance = Math.max(0.0, distance - Math.max(0.0, effectiveTopRadiusMeters));
    Pose3d pose = interpolateAtProgress(samples, direction, topEntryDistance).pose();
    double elapsed = interpolateAtProgress(samples, direction, topEntryDistance).elapsedSeconds();
    double clearance = pose.getZ() - fieldConeTopPosition.getZ();
    return new TopClearanceResult(clearance >= 0.0, clearance, elapsed, pose);
  }

  private BottomEntryResult evaluateBottomEntry(
      Pose3d[] samples, Translation3d fieldTargetPosition, double openingRadiusMeters) {
    double dtSeconds = config.integrationStepSeconds();
    for (int i = 1; i < samples.length; i++) {
      Pose3d previous = samples[i - 1];
      Pose3d current = samples[i];
      double previousZ = previous.getZ();
      double currentZ = current.getZ();
      if (previousZ >= fieldTargetPosition.getZ() && currentZ <= fieldTargetPosition.getZ()) {
        double denom = previousZ - currentZ;
        double ratio =
            Math.abs(denom) <= 1e-9 ? 0.0 : (previousZ - fieldTargetPosition.getZ()) / denom;
        ratio = Math.max(0.0, Math.min(1.0, ratio));
        Translation3d translation =
            previous
                .getTranslation()
                .plus(current.getTranslation().minus(previous.getTranslation()).times(ratio));
        Pose3d pose = new Pose3d(translation, new Rotation3d());
        double error =
            pose.getTranslation()
                .toTranslation2d()
                .getDistance(fieldTargetPosition.toTranslation2d());
        double elapsedSeconds = (i - 1 + ratio) * dtSeconds;
        return new BottomEntryResult(error <= openingRadiusMeters, error, elapsedSeconds, pose);
      }
    }
    Pose3d last = samples[samples.length - 1];
    double error =
        last.getTranslation().toTranslation2d().getDistance(fieldTargetPosition.toTranslation2d());
    return new BottomEntryResult(false, error, (samples.length - 1) * dtSeconds, last);
  }

  private InterpolatedPose interpolateAtProgress(
      Pose3d[] samples, Translation2d direction, double targetProgressMeters) {
    double dtSeconds = config.integrationStepSeconds();
    Translation2d release = samples[0].getTranslation().toTranslation2d();
    Pose3d previousPose = samples[0];
    double previousProgress = 0.0;
    for (int i = 1; i < samples.length; i++) {
      Pose3d currentPose = samples[i];
      Translation2d relative = currentPose.getTranslation().toTranslation2d().minus(release);
      double currentProgress =
          relative.getX() * direction.getX() + relative.getY() * direction.getY();
      if (currentProgress >= targetProgressMeters) {
        double span = currentProgress - previousProgress;
        double ratio =
            Math.abs(span) <= 1e-9 ? 0.0 : (targetProgressMeters - previousProgress) / span;
        ratio = Math.max(0.0, Math.min(1.0, ratio));
        Translation3d translation =
            previousPose
                .getTranslation()
                .plus(
                    currentPose.getTranslation().minus(previousPose.getTranslation()).times(ratio));
        return new InterpolatedPose(
            new Pose3d(translation, new Rotation3d()), (i - 1 + ratio) * dtSeconds);
      }
      previousPose = currentPose;
      previousProgress = currentProgress;
    }
    return new InterpolatedPose(samples[samples.length - 1], (samples.length - 1) * dtSeconds);
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
      boolean feasible,
      double closestApproachErrorMeters,
      double timeOfFlightSeconds,
      boolean clearsTop,
      boolean descendsIntoBottom,
      double topClearanceMeters,
      double bottomEntryErrorMeters,
      Pose3d topClearancePose,
      Pose3d bottomEntryPose) {}

  private record TopClearanceResult(
      boolean clearsTop, double clearanceMeters, double elapsedSeconds, Pose3d pose) {}

  private record BottomEntryResult(
      boolean descendsIntoBottom, double entryErrorMeters, double elapsedSeconds, Pose3d pose) {}

  private record InterpolatedPose(Pose3d pose, double elapsedSeconds) {}
}
