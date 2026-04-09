package org.Griffins1884.frc2026.simulation.control;

import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;

/** Applies a sensor frame to the control-host side of simulation. */
public interface SensorFrameApplicator {
  void apply(SensorFrame frame);
}
