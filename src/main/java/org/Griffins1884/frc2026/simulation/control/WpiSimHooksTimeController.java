package org.Griffins1884.frc2026.simulation.control;

import edu.wpi.first.wpilibj.simulation.SimHooks;

/** Lockstep time controller backed by WPILib SimHooks. */
public final class WpiSimHooksTimeController implements SimTimeController {
  @Override
  public void pause() {
    SimHooks.pauseTiming();
  }

  @Override
  public void resume() {
    SimHooks.resumeTiming();
  }

  @Override
  public void stepSeconds(double seconds) {
    SimHooks.stepTiming(seconds);
  }
}
