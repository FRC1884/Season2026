package org.Griffins1884.frc2026.simulation.contracts;

/** Canonical sensor/result frame for deterministic simulation replay. */
public record SensorFrame(
    long simTimeNanos,
    int stepId,
    boolean predictionAvailable,
    boolean predictionFeasible,
    boolean shotReleased,
    int activeProjectileCount,
    int projectileSpawnCount,
    double closestApproachErrorMeters,
    double timeOfFlightSeconds) {
  public SensorFrame {
    if (simTimeNanos < 0L) {
      throw new IllegalArgumentException("simTimeNanos must be non-negative");
    }
    if (stepId < 0) {
      throw new IllegalArgumentException("stepId must be non-negative");
    }
  }
}
