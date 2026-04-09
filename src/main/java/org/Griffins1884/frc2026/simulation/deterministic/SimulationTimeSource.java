package org.Griffins1884.frc2026.simulation.deterministic;

/** Time source abstraction so deterministic simulation can avoid wall-clock time. */
public interface SimulationTimeSource {
  double nowSeconds();

  long nowNanos();
}
