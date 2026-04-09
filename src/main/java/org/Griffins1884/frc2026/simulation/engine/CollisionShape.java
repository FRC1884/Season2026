package org.Griffins1884.frc2026.simulation.engine;

/** Collision shapes supported by the local deterministic engine. */
public interface CollisionShape {
  ShapeType type();

  enum ShapeType {
    SPHERE,
    BOX,
    CYLINDER,
    PLANE
  }

  static CollisionShape sphere(double radius) {
    return new SphereShape(radius);
  }

  static CollisionShape box(double halfX, double halfY, double halfZ) {
    return new BoxShape(halfX, halfY, halfZ);
  }

  static CollisionShape cylinder(double radius, double halfHeight) {
    return new CylinderShape(radius, halfHeight);
  }

  static CollisionShape plane(PhysicsMath.Vec3 localNormal, double offset) {
    return new PlaneShape(localNormal, offset);
  }
}

record SphereShape(double radius) implements CollisionShape {
  @Override
  public ShapeType type() {
    return ShapeType.SPHERE;
  }
}

record BoxShape(double halfX, double halfY, double halfZ) implements CollisionShape {
  @Override
  public ShapeType type() {
    return ShapeType.BOX;
  }
}

record CylinderShape(double radius, double halfHeight) implements CollisionShape {
  @Override
  public ShapeType type() {
    return ShapeType.CYLINDER;
  }
}

record PlaneShape(PhysicsMath.Vec3 localNormal, double offset) implements CollisionShape {
  @Override
  public ShapeType type() {
    return ShapeType.PLANE;
  }
}
