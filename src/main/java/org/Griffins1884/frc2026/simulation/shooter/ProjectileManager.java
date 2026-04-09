package org.Griffins1884.frc2026.simulation.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.Griffins1884.frc2026.simulation.deterministic.SimulationTimeSource;
import org.Griffins1884.frc2026.simulation.deterministic.WallClockSimulationTimeSource;

/** Owns active simulated projectiles and advances them over time. */
public final class ProjectileManager {
  private final ShotSimulationConfig.PhysicsConfig physics;
  private final SimulationTimeSource timeSource;
  private final List<ProjectileState> activeProjectiles = new ArrayList<>();
  private int spawnedCount = 0;

  public ProjectileManager(ShotSimulationConfig.PhysicsConfig physics) {
    this(physics, new WallClockSimulationTimeSource());
  }

  public ProjectileManager(
      ShotSimulationConfig.PhysicsConfig physics, SimulationTimeSource timeSource) {
    this.physics = physics;
    this.timeSource = timeSource != null ? timeSource : new WallClockSimulationTimeSource();
  }

  public void spawn(SimulatedShot shot) {
    if (shot == null || !shot.feasible()) {
      return;
    }
    activeProjectiles.add(
        new ProjectileState(
            shot.releasePose().getTranslation(),
            shot.initialVelocityMetersPerSecond(),
            timeSource.nowSeconds()));
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
