package org.Griffins1884.frc2026.simulation.control;

/** Time-control surface for lockstep WPILib simulation hosts. */
public interface SimTimeController {
  void pause();

  void resume();

  void stepSeconds(double seconds);
}
