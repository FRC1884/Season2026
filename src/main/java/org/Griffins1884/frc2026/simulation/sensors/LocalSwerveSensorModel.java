package org.Griffins1884.frc2026.simulation.sensors;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import java.util.Objects;
import org.Griffins1884.frc2026.simulation.sensors.LocalSwerveSensorModel.RawSwerveState;

/** Seeded latency-aware sensor staging for the active local swerve simulation runtime. */
public final class LocalSwerveSensorModel {
  private final SwerveSensorConfig config;
  private final LatencyQueue<SwerveSensorSample> latencyQueue = new LatencyQueue<>();
  private final SeededGaussianNoise yawNoise;
  private final SeededGaussianNoise pitchNoise;
  private final SeededGaussianNoise rollNoise;
  private final SeededGaussianNoise drivePositionNoise;
  private final SeededGaussianNoise driveVelocityNoise;
  private final SeededGaussianNoise steerPositionNoise;
  private final SeededGaussianNoise steerVelocityNoise;

  private SwerveSensorSample current;

  public LocalSwerveSensorModel(SwerveSensorConfig config, long seed) {
    this.config = Objects.requireNonNull(config);
    yawNoise = new SeededGaussianNoise(seed ^ 0xA1L, config.yawNoiseStdDevRad());
    pitchNoise = new SeededGaussianNoise(seed ^ 0xB2L, config.pitchNoiseStdDevRad());
    rollNoise = new SeededGaussianNoise(seed ^ 0xC3L, config.rollNoiseStdDevRad());
    drivePositionNoise =
        new SeededGaussianNoise(seed ^ 0xD4L, config.drivePositionNoiseStdDevRad());
    driveVelocityNoise =
        new SeededGaussianNoise(seed ^ 0xE5L, config.driveVelocityNoiseStdDevRadPerSec());
    steerPositionNoise =
        new SeededGaussianNoise(seed ^ 0xF6L, config.steerPositionNoiseStdDevRad());
    steerVelocityNoise =
        new SeededGaussianNoise(seed ^ 0x17L, config.steerVelocityNoiseStdDevRadPerSec());
  }

  public void observe(long nowNanos, RawSwerveState raw) {
    double[][] delayedDrivePositions = noisyArray(raw.drivePositionsRad(), drivePositionNoise);
    double[][] delayedDriveVelocities =
        noisyArray(raw.driveVelocitiesRadPerSec(), driveVelocityNoise);
    Rotation2d[][] delayedTurnPositions =
        noisyRotationArray(raw.turnPositions(), steerPositionNoise);
    double[][] delayedTurnVelocities =
        noisyArray(raw.turnVelocitiesRadPerSec(), steerVelocityNoise);
    double[][] delayedTurnRotations = raw.turnPositionsRotations();
    Rotation2d[] delayedYawPositions = noisyRotationVector(raw.cachedYawPositions(), yawNoise);

    SwerveSensorSample sample =
        new SwerveSensorSample(
            nowNanos / 1_000_000_000.0,
            noisyRotation(raw.yaw(), yawNoise),
            noisyRotation(raw.pitch(), pitchNoise),
            noisyRotation(raw.roll(), rollNoise),
            raw.yawVelocityRadPerSec() + yawNoise.sample(),
            raw.pitchVelocityRadPerSec() + pitchNoise.sample(),
            raw.rollVelocityRadPerSec() + rollNoise.sample(),
            raw.cachedTimestamps().clone(),
            delayedYawPositions,
            delayedDrivePositions,
            delayedDriveVelocities,
            delayedTurnPositions,
            delayedTurnVelocities,
            delayedTurnRotations);
    latencyQueue.enqueue(
        nowNanos + Math.max(config.gyroLatencyNanos(), config.moduleLatencyNanos()), sample);
    releaseReady(nowNanos);
    if (current == null) {
      current = sample;
    }
  }

  public void releaseReady(long nowNanos) {
    List<SwerveSensorSample> ready = latencyQueue.releaseReady(nowNanos);
    if (!ready.isEmpty()) {
      current = ready.get(ready.size() - 1);
    }
  }

  public SwerveSensorSample current() {
    return current;
  }

  private Rotation2d noisyRotation(Rotation2d rotation, SeededGaussianNoise noise) {
    return rotation.plus(Rotation2d.fromRadians(noise.sample()));
  }

  private Rotation2d[] noisyRotationVector(Rotation2d[] source, SeededGaussianNoise noise) {
    Rotation2d[] noisy = new Rotation2d[source.length];
    for (int i = 0; i < source.length; i++) {
      noisy[i] = noisyRotation(source[i], noise);
    }
    return noisy;
  }

  private Rotation2d[][] noisyRotationArray(Rotation2d[][] source, SeededGaussianNoise noise) {
    Rotation2d[][] noisy = new Rotation2d[source.length][];
    for (int i = 0; i < source.length; i++) {
      noisy[i] = noisyRotationVector(source[i], noise);
    }
    return noisy;
  }

  private double[][] noisyArray(double[][] source, SeededGaussianNoise noise) {
    double[][] noisy = new double[source.length][];
    for (int i = 0; i < source.length; i++) {
      noisy[i] = new double[source[i].length];
      for (int j = 0; j < source[i].length; j++) {
        noisy[i][j] = source[i][j] + noise.sample();
      }
    }
    return noisy;
  }

  public record RawSwerveState(
      Rotation2d yaw,
      Rotation2d pitch,
      Rotation2d roll,
      double yawVelocityRadPerSec,
      double pitchVelocityRadPerSec,
      double rollVelocityRadPerSec,
      double[] cachedTimestamps,
      Rotation2d[] cachedYawPositions,
      double[][] drivePositionsRad,
      double[][] driveVelocitiesRadPerSec,
      Rotation2d[][] turnPositions,
      double[][] turnVelocitiesRadPerSec,
      double[][] turnPositionsRotations) {
    public SwerveSensorSample toSample(double timestampSeconds) {
      return new SwerveSensorSample(
          timestampSeconds,
          yaw,
          pitch,
          roll,
          yawVelocityRadPerSec,
          pitchVelocityRadPerSec,
          rollVelocityRadPerSec,
          cachedTimestamps,
          cachedYawPositions,
          drivePositionsRad,
          driveVelocitiesRadPerSec,
          turnPositions,
          turnVelocitiesRadPerSec,
          turnPositionsRotations);
    }
  }
}
