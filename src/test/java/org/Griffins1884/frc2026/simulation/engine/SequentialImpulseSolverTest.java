package org.Griffins1884.frc2026.simulation.engine;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Quat;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;
import org.junit.jupiter.api.Test;

class SequentialImpulseSolverTest {
  @Test
  void sphereSettlesAbovePlaneWithoutExploding() {
    PhysicsWorld world = new PhysicsWorld(new Vec3(0.0, 0.0, -9.80665));
    RigidBody plane =
        new RigidBody(
            1,
            "plane",
            RigidBodyType.STATIC,
            CollisionShape.plane(Vec3.UNIT_Z, 0.0),
            PhysicsMaterial.DEFAULT,
            0.0,
            Vec3.ZERO,
            Vec3.ZERO,
            Quat.IDENTITY);
    RigidBody sphere =
        new RigidBody(
            2,
            "sphere",
            RigidBodyType.DYNAMIC,
            CollisionShape.sphere(0.2),
            PhysicsMaterial.DEFAULT,
            1.0,
            new Vec3(12.0, 12.0, 12.0),
            new Vec3(0.0, 0.0, 1.0),
            Quat.IDENTITY);
    world.addBody(plane);
    world.addBody(sphere);

    for (int i = 0; i < 240; i++) {
      world.step(0.005, null);
    }

    assertTrue(sphere.position().z() > 0.15);
    assertTrue(Math.abs(sphere.linearVelocity().z()) < 1.0);
  }
}
