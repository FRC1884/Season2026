package org.Griffins1884.frc2026.simulation.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/** Owns active simulated projectiles and advances them over time. */
public final class ProjectileManager {
  private final ShotSimulationConfig.PhysicsConfig physics;
  private final List<ProjectileState> activeProjectiles = new ArrayList<>();
  private int nextProjectileId = 1;
  private int spawnedCount = 0;

  public ProjectileManager(ShotSimulationConfig.PhysicsConfig physics) {
    this.physics = physics;
  }

  public ProjectileSnapshot spawn(SimulatedShot shot) {
    if (shot == null
        || shot.releasePose() == null
        || shot.initialVelocityMetersPerSecond() == null
        || !Double.isFinite(shot.initialVelocityMetersPerSecond().getNorm())
        || shot.initialVelocityMetersPerSecond().getNorm() <= 1e-9) {
      return null;
    }
    ProjectileState projectile =
        new ProjectileState(
            nextProjectileId++,
            shot.releasePose().getTranslation(),
            shot.initialVelocityMetersPerSecond(),
            Timer.getFPGATimestamp());
    activeProjectiles.add(projectile);
    spawnedCount++;
    return ProjectileSnapshot.from(projectile);
  }

  public void clear() {
    activeProjectiles.clear();
  }

  public List<ProjectileSnapshot> update(double dtSeconds) {
    List<ProjectileSnapshot> completed = new ArrayList<>();
    double nowSeconds = Timer.getFPGATimestamp();
    Iterator<ProjectileState> iterator = activeProjectiles.iterator();
    while (iterator.hasNext()) {
      ProjectileState projectile = iterator.next();
      projectile.advance(physics, dtSeconds, nowSeconds);
      if (!projectile.active()) {
        completed.add(ProjectileSnapshot.from(projectile));
        iterator.remove();
      }
    }
    return completed;
  }

  public Pose3d[] activeProjectilePoses() {
    return activeProjectiles.stream().map(ProjectileState::pose).toArray(Pose3d[]::new);
  }

  public List<ProjectileSnapshot> activeProjectileSnapshots() {
    return activeProjectiles.stream().map(ProjectileSnapshot::from).toList();
  }

  public int activeCount() {
    return activeProjectiles.size();
  }

  public int spawnedCount() {
    return spawnedCount;
  }

  public record ProjectileSnapshot(
      int id,
      Translation3d positionMeters,
      Translation3d velocityMetersPerSecond,
      double spawnTimestampSec,
      double ageSeconds,
      boolean active,
      boolean impacted,
      double impactTimestampSec,
      Translation3d impactPositionMeters,
      Translation3d impactVelocityMetersPerSecond) {
    private static ProjectileSnapshot from(ProjectileState projectile) {
      return new ProjectileSnapshot(
          projectile.id(),
          projectile.positionMeters(),
          projectile.velocityMetersPerSecond(),
          projectile.spawnTimestampSec(),
          projectile.ageSeconds(),
          projectile.active(),
          projectile.impacted(),
          projectile.impactTimestampSec(),
          projectile.impactPositionMeters(),
          projectile.impactVelocityMetersPerSecond());
    }
  }
}
