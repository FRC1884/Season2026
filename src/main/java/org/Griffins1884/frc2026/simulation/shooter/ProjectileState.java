package org.Griffins1884.frc2026.simulation.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Mutable projectile state used for predicted trajectories and live simulated shots. */
public final class ProjectileState {
  private final int id;
  private final double spawnTimestampSec;
  private double ageSeconds;
  private Translation3d positionMeters;
  private Translation3d velocityMetersPerSecond;
  private boolean active;
  private boolean impacted;
  private double impactTimestampSec = Double.NaN;
  private Translation3d impactPositionMeters = null;
  private Translation3d impactVelocityMetersPerSecond = null;

  public ProjectileState(
      Translation3d positionMeters,
      Translation3d velocityMetersPerSecond,
      double spawnTimestampSec) {
    this(0, positionMeters, velocityMetersPerSecond, spawnTimestampSec);
  }

  public ProjectileState(
      int id,
      Translation3d positionMeters,
      Translation3d velocityMetersPerSecond,
      double spawnTimestampSec) {
    this.id = id;
    this.positionMeters = positionMeters != null ? positionMeters : new Translation3d();
    this.velocityMetersPerSecond =
        velocityMetersPerSecond != null ? velocityMetersPerSecond : new Translation3d();
    this.spawnTimestampSec = spawnTimestampSec;
    active = true;
  }

  public int id() {
    return id;
  }

  public Translation3d positionMeters() {
    return positionMeters;
  }

  public Translation3d velocityMetersPerSecond() {
    return velocityMetersPerSecond;
  }

  public double spawnTimestampSec() {
    return spawnTimestampSec;
  }

  public double ageSeconds() {
    return ageSeconds;
  }

  public boolean active() {
    return active;
  }

  public boolean impacted() {
    return impacted;
  }

  public double impactTimestampSec() {
    return impactTimestampSec;
  }

  public Translation3d impactPositionMeters() {
    return impactPositionMeters;
  }

  public Translation3d impactVelocityMetersPerSecond() {
    return impactVelocityMetersPerSecond;
  }

  public Pose3d pose() {
    return new Pose3d(positionMeters, new Rotation3d());
  }

  public void deactivate() {
    active = false;
  }

  public void advance(ShotSimulationConfig.PhysicsConfig physics, double dtSeconds) {
    advance(physics, dtSeconds, Double.NaN);
  }

  public void advance(
      ShotSimulationConfig.PhysicsConfig physics, double dtSeconds, double currentTimestampSec) {
    if (!active || physics == null || dtSeconds <= 0.0) {
      return;
    }

    Translation3d acceleration = accelerationFor(physics);
    positionMeters =
        positionMeters
            .plus(velocityMetersPerSecond.times(dtSeconds))
            .plus(acceleration.times(0.5 * dtSeconds * dtSeconds));
    velocityMetersPerSecond = velocityMetersPerSecond.plus(acceleration.times(dtSeconds));
    ageSeconds += dtSeconds;
    if (positionMeters.getZ() <= 0.0 && velocityMetersPerSecond.getZ() <= 0.0) {
      positionMeters = new Translation3d(positionMeters.getX(), positionMeters.getY(), 0.0);
      impacted = true;
      impactTimestampSec =
          Double.isFinite(currentTimestampSec) ? currentTimestampSec + dtSeconds : ageSeconds;
      impactPositionMeters = positionMeters;
      impactVelocityMetersPerSecond = velocityMetersPerSecond;
      active = false;
    }
  }

  private Translation3d accelerationFor(ShotSimulationConfig.PhysicsConfig physics) {
    double speed = velocityMetersPerSecond.getNorm();
    Translation3d drag =
        velocityMetersPerSecond
            .times(-physics.linearDragPerSecond())
            .plus(velocityMetersPerSecond.times(-physics.quadraticDragPerMeter() * speed));
    Translation3d gravity = new Translation3d(0.0, 0.0, -physics.gravityMetersPerSecondSquared());
    return drag.plus(gravity);
  }
}
