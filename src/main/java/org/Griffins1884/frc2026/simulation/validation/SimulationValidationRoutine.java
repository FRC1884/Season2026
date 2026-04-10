package org.Griffins1884.frc2026.simulation.validation;

/** Common lifecycle for unattended simulateJava validation routines. */
public interface SimulationValidationRoutine {
  void applyInputs();

  void captureStepAndMaybeExit();
}
