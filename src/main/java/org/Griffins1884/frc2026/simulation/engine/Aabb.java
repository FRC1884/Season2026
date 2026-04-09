package org.Griffins1884.frc2026.simulation.engine;

import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;

/** Axis-aligned bounding box for deterministic broadphase. */
public record Aabb(Vec3 min, Vec3 max) {
  public boolean overlaps(Aabb other) {
    return min.x() <= other.max.x()
        && max.x() >= other.min.x()
        && min.y() <= other.max.y()
        && max.y() >= other.min.y()
        && min.z() <= other.max.z()
        && max.z() >= other.min.z();
  }
}
