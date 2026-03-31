package org.Griffins1884.frc2026.simulation.maple;

import org.griffins1884.sim3d.seasonspecific.rebuilt2026.Rebuilt2026MapleArena;
import org.ironmaple.simulation.SimulatedArena;

/** Ensures Maple uses the GriffinSim 2026 rebuilt arena instead of the vendordep default season. */
public final class MapleArenaSetup {
  private static boolean configured = false;

  private MapleArenaSetup() {}

  public static synchronized void ensure2026RebuiltArena() {
    if (configured) {
      return;
    }

    SimulatedArena.overrideInstance(new Rebuilt2026MapleArena(true));
    configured = true;
  }
}
