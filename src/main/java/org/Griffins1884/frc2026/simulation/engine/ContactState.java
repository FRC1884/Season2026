package org.Griffins1884.frc2026.simulation.engine;

import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;

/** Solver contact state for one deterministic contact point. */
public final class ContactState {
  private final int bodyAId;
  private final int bodyBId;
  private final Vec3 point;
  private final Vec3 normal;
  private final double penetrationDepth;
  private final double frictionCoefficient;
  private final double restitutionCoefficient;
  private final Vec3 tangent1;
  private final Vec3 tangent2;

  private double accumulatedNormalImpulse;
  private double accumulatedTangentImpulse1;
  private double accumulatedTangentImpulse2;

  public ContactState(
      int bodyAId,
      int bodyBId,
      Vec3 point,
      Vec3 normal,
      double penetrationDepth,
      double frictionCoefficient,
      double restitutionCoefficient) {
    this.bodyAId = bodyAId;
    this.bodyBId = bodyBId;
    this.point = point;
    this.normal = normal.normalize();
    this.penetrationDepth = penetrationDepth;
    this.frictionCoefficient = frictionCoefficient;
    this.restitutionCoefficient = restitutionCoefficient;
    Vec3 axis = Math.abs(this.normal.z()) < 0.9 ? PhysicsMath.Vec3.UNIT_Z : PhysicsMath.Vec3.UNIT_X;
    tangent1 = this.normal.cross(axis).normalize();
    tangent2 = this.normal.cross(tangent1).normalize();
  }

  public int bodyAId() {
    return bodyAId;
  }

  public int bodyBId() {
    return bodyBId;
  }

  public Vec3 point() {
    return point;
  }

  public Vec3 normal() {
    return normal;
  }

  public double penetrationDepth() {
    return penetrationDepth;
  }

  public double frictionCoefficient() {
    return frictionCoefficient;
  }

  public double restitutionCoefficient() {
    return restitutionCoefficient;
  }

  public Vec3 tangent1() {
    return tangent1;
  }

  public Vec3 tangent2() {
    return tangent2;
  }

  public double accumulatedNormalImpulse() {
    return accumulatedNormalImpulse;
  }

  public double accumulatedTangentImpulse1() {
    return accumulatedTangentImpulse1;
  }

  public double accumulatedTangentImpulse2() {
    return accumulatedTangentImpulse2;
  }

  public void addNormalImpulse(double impulse) {
    accumulatedNormalImpulse += impulse;
  }

  public void addTangentImpulse1(double impulse) {
    accumulatedTangentImpulse1 += impulse;
  }

  public void addTangentImpulse2(double impulse) {
    accumulatedTangentImpulse2 += impulse;
  }
}
