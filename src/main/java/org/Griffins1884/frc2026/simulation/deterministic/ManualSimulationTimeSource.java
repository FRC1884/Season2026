package org.Griffins1884.frc2026.simulation.deterministic;

/** Fixed-step time source for deterministic replay and lockstep simulation. */
public final class ManualSimulationTimeSource implements SimulationTimeSource {
  private long currentNanos;

  public ManualSimulationTimeSource() {
    this(0L);
  }

  public ManualSimulationTimeSource(long initialNanos) {
    if (initialNanos < 0L) {
      throw new IllegalArgumentException("initialNanos must be non-negative");
    }
    currentNanos = initialNanos;
  }

  @Override
  public double nowSeconds() {
    return currentNanos / 1_000_000_000.0;
  }

  @Override
  public long nowNanos() {
    return currentNanos;
  }

  public void setNanos(long currentNanos) {
    if (currentNanos < 0L) {
      throw new IllegalArgumentException("currentNanos must be non-negative");
    }
    this.currentNanos = currentNanos;
  }

  public void advanceSeconds(double deltaSeconds) {
    if (!Double.isFinite(deltaSeconds) || deltaSeconds < 0.0) {
      throw new IllegalArgumentException("deltaSeconds must be finite and non-negative");
    }
    currentNanos += Math.round(deltaSeconds * 1_000_000_000.0);
  }
}
