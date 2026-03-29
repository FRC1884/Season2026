package org.Griffins1884.frc2026.simulation.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.Griffins1884.frc2026.util.ballistics.ShotModelConfig;

/** Owns active simulated projectiles and advances them over time. */
public final class ProjectileManager {
  private final ShotModelConfig.PhysicsConfig physics;
  private final List<ProjectileState> activeProjectiles = new ArrayList<>();
  private int spawnedCount = 0;

  public ProjectileManager(ShotModelConfig.PhysicsConfig physics) {
    this.physics = physics;
  }

  public void spawn(SimulatedShot shot) {
    if (shot == null || !shot.feasible()) {
      return;
    }
    activeProjectiles.add(
        new ProjectileState(
            shot.releasePose().getTranslation(),
            shot.initialVelocityMetersPerSecond(),
            Timer.getFPGATimestamp()));
    spawnedCount++;
  }

  public void clear() {
    activeProjectiles.clear();
  }

  public void update(double dtSeconds) {
    Iterator<ProjectileState> iterator = activeProjectiles.iterator();
    while (iterator.hasNext()) {
      ProjectileState projectile = iterator.next();
      projectile.advance(physics, dtSeconds);
      if (!projectile.active()) {
        iterator.remove();
      }
    }
  }

  public Pose3d[] activeProjectilePoses() {
    return activeProjectiles.stream().map(ProjectileState::pose).toArray(Pose3d[]::new);
  }

  public int activeCount() {
    return activeProjectiles.size();
  }

  public int spawnedCount() {
    return spawnedCount;
  }
}
