package org.Griffins1884.frc2026.simulation.control;

import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;

/** Captures the current actuator state after a control tick. */
public interface ActuatorFrameSupplier {
  ActuatorFrame capture(int stepId, long simTimeNanos);
}
