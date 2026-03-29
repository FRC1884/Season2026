package org.Griffins1884.frc2026.simulation.maple;

import org.ironmaple.simulation.SimulatedArena;

/** Small facade for MapleSim arena access so simulation code does not depend on the singleton. */
public final class MapleArenaAdapter {
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
  }

  public void resetFieldForAuto() {
    SimulatedArena.getInstance().resetFieldForAuto();
  }
}
