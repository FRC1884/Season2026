package org.Griffins1884.frc2026.simulation.engine;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;

/** Deterministic rigid-body world with broadphase, narrowphase, and impulse solve. */
public final class PhysicsWorld {
  private static final double POSITIONAL_CORRECTION_PERCENT = 0.8;
  private static final double POSITIONAL_SLOP_METERS = 0.001;
  private final List<RigidBody> bodies = new ArrayList<>();
  private final CollisionSystem collisionSystem = new CollisionSystem();
  private final SequentialImpulseSolver solver = new SequentialImpulseSolver();
  private final Vec3 gravity;
  private final Set<Long> ignoredPairs = new HashSet<>();
  private List<ContactState> lastContacts = List.of();

  public PhysicsWorld(Vec3 gravity) {
    this.gravity = gravity;
  }

  public void addBody(RigidBody body) {
    bodies.add(body);
    bodies.sort(Comparator.comparingInt(RigidBody::id));
  }

  public List<RigidBody> bodies() {
    return List.copyOf(bodies);
  }

  public RigidBody findBody(int id) {
    return bodies.stream().filter(body -> body.id() == id).findFirst().orElse(null);
  }

  public List<ContactState> lastContacts() {
    return lastContacts;
  }

  public void ignorePair(int bodyAId, int bodyBId) {
    int low = Math.min(bodyAId, bodyBId);
    int high = Math.max(bodyAId, bodyBId);
    ignoredPairs.add((((long) low) << 32) | (high & 0xffffffffL));
  }

  public void step(double dtSeconds, Consumer<PhysicsWorld> externalForces) {
    for (RigidBody body : bodies) {
      body.clearAccumulators();
    }
    if (externalForces != null) {
      externalForces.accept(this);
    }
    for (RigidBody body : bodies) {
      body.integrateForces(dtSeconds, gravity);
    }
    lastContacts = collisionSystem.detect(bodies, ignoredPairs);
    solver.solve(bodies, lastContacts, dtSeconds);
    for (RigidBody body : bodies) {
      body.integratePose(dtSeconds);
    }
    positionalCorrection(collisionSystem.detect(bodies, ignoredPairs));
  }

  private void positionalCorrection(List<ContactState> contacts) {
    for (ContactState contact : contacts) {
      RigidBody bodyA = findBody(contact.bodyAId());
      RigidBody bodyB = findBody(contact.bodyBId());
      if (bodyA == null || bodyB == null) {
        continue;
      }
      double inverseMassSum = bodyA.inverseMass() + bodyB.inverseMass();
      if (inverseMassSum < PhysicsMath.EPSILON) {
        continue;
      }
      double correctionMagnitude =
          Math.max(contact.penetrationDepth() - POSITIONAL_SLOP_METERS, 0.0)
              * POSITIONAL_CORRECTION_PERCENT
              / inverseMassSum;
      Vec3 correction = contact.normal().scale(correctionMagnitude);
      bodyA.translate(correction.negate().scale(bodyA.inverseMass()));
      bodyB.translate(correction.scale(bodyB.inverseMass()));
    }
  }
}
