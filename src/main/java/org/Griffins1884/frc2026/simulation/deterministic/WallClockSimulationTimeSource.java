package org.Griffins1884.frc2026.simulation.deterministic;

import edu.wpi.first.wpilibj.Timer;

/** Default runtime time source backed by WPILib wall-clock FPGA time. */
public final class WallClockSimulationTimeSource implements SimulationTimeSource {
  @Override
  public double nowSeconds() {
    return Timer.getFPGATimestamp();
  }

  @Override
  public long nowNanos() {
    return Math.round(Timer.getFPGATimestamp() * 1_000_000_000.0);
  }
}
