package org.Griffins1884.frc2026.simulation.shooter;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class ProjectileStateTest {
  @Test
  void projectileFallsAndEventuallyDeactivates() {
    ProjectileState projectile =
        new ProjectileState(
            new Translation3d(0.0, 0.0, 0.5), new Translation3d(1.0, 0.0, 0.0), 0.0);

    projectile.advance(new ShotSimulationConfig.PhysicsConfig(9.80665, 0.0, 0.0), 0.1);
    assertTrue(projectile.positionMeters().getZ() < 0.5);

    for (int i = 0; i < 30 && projectile.active(); i++) {
      projectile.advance(new ShotSimulationConfig.PhysicsConfig(9.80665, 0.0, 0.0), 0.1);
    }

    assertFalse(projectile.active());
  }
}
