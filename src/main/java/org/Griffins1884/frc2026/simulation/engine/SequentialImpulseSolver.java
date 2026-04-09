package org.Griffins1884.frc2026.simulation.engine;

import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;

/** Deterministic projected Gauss-Seidel impulse solver. */
public final class SequentialImpulseSolver {
  private static final double BAUMGARTE_BETA = 0.15;
  private static final double VELOCITY_SLOP = 0.01;
  private static final int SOLVER_ITERATIONS = 12;

  public void solve(List<RigidBody> bodies, List<ContactState> contacts, double dtSeconds) {
    Map<Integer, RigidBody> bodyById =
        bodies.stream().collect(Collectors.toMap(RigidBody::id, Function.identity()));

    for (int iteration = 0; iteration < SOLVER_ITERATIONS; iteration++) {
      for (ContactState contact : contacts) {
        RigidBody bodyA = bodyById.get(contact.bodyAId());
        RigidBody bodyB = bodyById.get(contact.bodyBId());
        solveNormalImpulse(bodyA, bodyB, contact, dtSeconds);
        solveTangentImpulse(bodyA, bodyB, contact, contact.tangent1(), true);
        solveTangentImpulse(bodyA, bodyB, contact, contact.tangent2(), false);
      }
    }
  }

  private void solveNormalImpulse(
      RigidBody bodyA, RigidBody bodyB, ContactState contact, double dtSeconds) {
    Vec3 ra = contact.point().subtract(bodyA.position());
    Vec3 rb = contact.point().subtract(bodyB.position());
    Vec3 relativeVelocity =
        bodyB.pointVelocity(contact.point()).subtract(bodyA.pointVelocity(contact.point()));
    double normalVelocity = relativeVelocity.dot(contact.normal());
    double bias =
        Math.max(0.0, contact.penetrationDepth() - VELOCITY_SLOP) * (BAUMGARTE_BETA / dtSeconds);
    double restitutionBias =
        normalVelocity < -0.5 ? -contact.restitutionCoefficient() * normalVelocity : 0.0;
    double denominator = effectiveMass(bodyA, bodyB, ra, rb, contact.normal());
    if (denominator < PhysicsMath.EPSILON) {
      return;
    }
    double lambda = (-(normalVelocity + bias + restitutionBias)) / denominator;
    double previousImpulse = contact.accumulatedNormalImpulse();
    double nextImpulse = Math.max(0.0, previousImpulse + lambda);
    double impulseDelta = nextImpulse - previousImpulse;
    contact.addNormalImpulse(impulseDelta);
    Vec3 impulse = contact.normal().scale(impulseDelta);
    bodyA.applyImpulseAtWorldPoint(impulse.negate(), contact.point());
    bodyB.applyImpulseAtWorldPoint(impulse, contact.point());
  }

  private void solveTangentImpulse(
      RigidBody bodyA, RigidBody bodyB, ContactState contact, Vec3 tangent, boolean firstTangent) {
    Vec3 ra = contact.point().subtract(bodyA.position());
    Vec3 rb = contact.point().subtract(bodyB.position());
    Vec3 relativeVelocity =
        bodyB.pointVelocity(contact.point()).subtract(bodyA.pointVelocity(contact.point()));
    double tangentVelocity = relativeVelocity.dot(tangent);
    double denominator = effectiveMass(bodyA, bodyB, ra, rb, tangent);
    if (denominator < PhysicsMath.EPSILON) {
      return;
    }
    double lambda = -tangentVelocity / denominator;
    double limit = contact.frictionCoefficient() * contact.accumulatedNormalImpulse();
    double previousImpulse =
        firstTangent ? contact.accumulatedTangentImpulse1() : contact.accumulatedTangentImpulse2();
    double nextImpulse = clamp(previousImpulse + lambda, -limit, limit);
    double impulseDelta = nextImpulse - previousImpulse;
    if (firstTangent) {
      contact.addTangentImpulse1(impulseDelta);
    } else {
      contact.addTangentImpulse2(impulseDelta);
    }
    Vec3 impulse = tangent.scale(impulseDelta);
    bodyA.applyImpulseAtWorldPoint(impulse.negate(), contact.point());
    bodyB.applyImpulseAtWorldPoint(impulse, contact.point());
  }

  private double effectiveMass(RigidBody bodyA, RigidBody bodyB, Vec3 ra, Vec3 rb, Vec3 direction) {
    Vec3 raCrossN = ra.cross(direction);
    Vec3 rbCrossN = rb.cross(direction);
    double angularA = raCrossN.dot(bodyA.inverseInertiaWorldMultiply(raCrossN));
    double angularB = rbCrossN.dot(bodyB.inverseInertiaWorldMultiply(rbCrossN));
    return bodyA.inverseMass() + bodyB.inverseMass() + angularA + angularB;
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
