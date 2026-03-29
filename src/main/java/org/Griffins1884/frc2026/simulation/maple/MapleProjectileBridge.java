package org.Griffins1884.frc2026.simulation.maple;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Placeholder bridge for future MapleSim projectile/gamepiece integration.
 *
 * <p>The current workflow keeps projectile dynamics in the robot-side shot simulator and reserves
 * this class for a later handoff into MapleSim arena entities once the game-piece API is chosen.
 */
public final class MapleProjectileBridge {
  public void publishActiveProjectiles(Pose3d[] projectilePoses) {
    // Intentionally no-op for now. Projectile visualization is handled through AdvantageScope.
  }

  public void clear() {
    // Intentionally no-op.
  }
}
