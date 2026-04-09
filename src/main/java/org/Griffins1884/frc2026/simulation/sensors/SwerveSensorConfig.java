package org.Griffins1884.frc2026.simulation.sensors;

/** Deterministic noise/latency profile for active SIM drivetrain sensors. */
public record SwerveSensorConfig(
    long gyroLatencyNanos,
    long moduleLatencyNanos,
    double yawNoiseStdDevRad,
    double pitchNoiseStdDevRad,
    double rollNoiseStdDevRad,
    double drivePositionNoiseStdDevRad,
    double driveVelocityNoiseStdDevRadPerSec,
    double steerPositionNoiseStdDevRad,
    double steerVelocityNoiseStdDevRadPerSec) {
  public static SwerveSensorConfig defaults() {
    return new SwerveSensorConfig(
        20_000_000L,
        10_000_000L,
        Math.toRadians(0.1),
        Math.toRadians(0.15),
        Math.toRadians(0.15),
        0.0008,
        0.02,
        Math.toRadians(0.05),
        Math.toRadians(0.4));
  }
}
