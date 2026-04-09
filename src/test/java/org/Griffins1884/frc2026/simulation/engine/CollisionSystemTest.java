package org.Griffins1884.frc2026.simulation.engine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Quat;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;
import org.junit.jupiter.api.Test;

class CollisionSystemTest {
  @Test
  void detectsSpherePlaneAndBoxBoxContacts() {
    CollisionSystem collisionSystem = new CollisionSystem();
    RigidBody sphere =
        new RigidBody(
            1,
            "sphere",
            RigidBodyType.DYNAMIC,
            CollisionShape.sphere(0.25),
            PhysicsMaterial.DEFAULT,
            1.0,
            new Vec3(1.0, 1.0, 1.0),
            new Vec3(0.0, 0.0, 0.1),
            Quat.IDENTITY);
    RigidBody plane =
        new RigidBody(
            2,
            "plane",
            RigidBodyType.STATIC,
            CollisionShape.plane(Vec3.UNIT_Z, 0.0),
            PhysicsMaterial.DEFAULT,
            0.0,
            Vec3.ZERO,
            Vec3.ZERO,
            Quat.IDENTITY);
    RigidBody boxA =
        new RigidBody(
            3,
            "box-a",
            RigidBodyType.DYNAMIC,
            CollisionShape.box(0.5, 0.5, 0.5),
            PhysicsMaterial.DEFAULT,
            1.0,
            new Vec3(1.0, 1.0, 1.0),
            new Vec3(1.0, 0.0, 0.5),
            Quat.IDENTITY);
    RigidBody boxB =
        new RigidBody(
            4,
            "box-b",
            RigidBodyType.STATIC,
            CollisionShape.box(0.5, 0.5, 0.5),
            PhysicsMaterial.DEFAULT,
            0.0,
            Vec3.ZERO,
            new Vec3(1.6, 0.0, 0.5),
            Quat.IDENTITY);

    List<ContactState> contacts =
        collisionSystem.detect(List.of(sphere, plane, boxA, boxB), Set.of());

    assertEquals(2, contacts.size());
    assertTrue(
        contacts.stream().anyMatch(contact -> contact.bodyAId() == 1 && contact.bodyBId() == 2));
    assertTrue(
        contacts.stream().anyMatch(contact -> contact.bodyAId() == 3 && contact.bodyBId() == 4));
  }

  @Test
  void detectsCylinderPlaneContact() {
    CollisionSystem collisionSystem = new CollisionSystem();
    RigidBody cylinder =
        new RigidBody(
            1,
            "wheel",
            RigidBodyType.DYNAMIC,
            CollisionShape.cylinder(0.05, 0.02),
            PhysicsMaterial.DEFAULT,
            1.0,
            new Vec3(1.0, 1.0, 1.0),
            new Vec3(0.0, 0.0, 0.03),
            Quat.IDENTITY);
    RigidBody plane =
        new RigidBody(
            2,
            "plane",
            RigidBodyType.STATIC,
            CollisionShape.plane(Vec3.UNIT_Z, 0.0),
            PhysicsMaterial.DEFAULT,
            0.0,
            Vec3.ZERO,
            Vec3.ZERO,
            Quat.IDENTITY);

    List<ContactState> contacts = collisionSystem.detect(List.of(cylinder, plane), Set.of());

    assertEquals(1, contacts.size());
    assertTrue(contacts.get(0).penetrationDepth() > 0.0);
  }
}
