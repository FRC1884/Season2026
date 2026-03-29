package org.Griffins1884.frc2026.simulation.shooter;

/** Edge detector for simulated shot-release events. */
public final class ShotReleaseDetector {
  private boolean previouslyArmed = false;

  public boolean update(boolean armedNow) {
    boolean released = armedNow && !previouslyArmed;
    previouslyArmed = armedNow;
    return released;
  }

  public void reset() {
    previouslyArmed = false;
  }
}
