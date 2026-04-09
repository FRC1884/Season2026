package org.Griffins1884.frc2026.simulation.engine;

import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Quat;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;

/** Mutable rigid body used by the local deterministic multibody engine. */
public final class RigidBody {
  private static final double MAX_LINEAR_SPEED_MPS = 25.0;
  private static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 80.0;
  private final int id;
  private final String name;
  private final RigidBodyType type;
  private final CollisionShape shape;
  private final PhysicsMaterial material;
  private final int collisionGroup;
  private final double massKg;
  private final double inverseMass;
  private final Vec3 inverseInertiaLocal;

  private Vec3 position;
  private Quat orientation;
  private Vec3 linearVelocity = Vec3.ZERO;
  private Vec3 angularVelocity = Vec3.ZERO;
  private Vec3 accumulatedForce = Vec3.ZERO;
  private Vec3 accumulatedTorque = Vec3.ZERO;

  public RigidBody(
      int id,
      String name,
      RigidBodyType type,
      CollisionShape shape,
      PhysicsMaterial material,
      double massKg,
      Vec3 inverseInertiaLocal,
      Vec3 position,
      Quat orientation) {
    this(id, name, type, shape, material, massKg, inverseInertiaLocal, position, orientation, id);
  }

  public RigidBody(
      int id,
      String name,
      RigidBodyType type,
      CollisionShape shape,
      PhysicsMaterial material,
      double massKg,
      Vec3 inverseInertiaLocal,
      Vec3 position,
      Quat orientation,
      int collisionGroup) {
    this.id = id;
    this.name = name;
    this.type = type;
    this.shape = shape;
    this.material = material != null ? material : PhysicsMaterial.DEFAULT;
    this.collisionGroup = collisionGroup;
    this.massKg = massKg;
    inverseMass =
        type == RigidBodyType.DYNAMIC && massKg > PhysicsMath.EPSILON ? 1.0 / massKg : 0.0;
    this.inverseInertiaLocal = inverseInertiaLocal != null ? inverseInertiaLocal : Vec3.ZERO;
    this.position = position != null ? position : Vec3.ZERO;
    this.orientation = orientation != null ? orientation.normalize() : Quat.IDENTITY;
  }

  public int id() {
    return id;
  }

  public String name() {
    return name;
  }

  public RigidBodyType type() {
    return type;
  }

  public CollisionShape shape() {
    return shape;
  }

  public int collisionGroup() {
    return collisionGroup;
  }

  public PhysicsMaterial material() {
    return material;
  }

  public double massKg() {
    return massKg;
  }

  public double inverseMass() {
    return inverseMass;
  }

  public Vec3 inverseInertiaLocal() {
    return inverseInertiaLocal;
  }

  public Vec3 position() {
    return position;
  }

  public Quat orientation() {
    return orientation;
  }

  public Vec3 linearVelocity() {
    return linearVelocity;
  }

  public Vec3 angularVelocity() {
    return angularVelocity;
  }

  public void setPose(Vec3 position, Quat orientation) {
    this.position = sanitizeVector(position);
    this.orientation = sanitizeQuat(orientation);
  }

  public void translate(Vec3 delta) {
    if (!isDynamic()) {
      return;
    }
    position = sanitizeVector(position.add(delta));
  }

  public void setVelocities(Vec3 linearVelocity, Vec3 angularVelocity) {
    this.linearVelocity = sanitizeVelocity(linearVelocity, MAX_LINEAR_SPEED_MPS);
    this.angularVelocity = sanitizeVelocity(angularVelocity, MAX_ANGULAR_SPEED_RAD_PER_SEC);
  }

  public boolean isDynamic() {
    return type == RigidBodyType.DYNAMIC;
  }

  public void clearAccumulators() {
    accumulatedForce = Vec3.ZERO;
    accumulatedTorque = Vec3.ZERO;
  }

  public void addForce(Vec3 force) {
    if (!isDynamic()) {
      return;
    }
    accumulatedForce = accumulatedForce.add(force);
  }

  public void addTorque(Vec3 torque) {
    if (!isDynamic()) {
      return;
    }
    accumulatedTorque = accumulatedTorque.add(torque);
  }

  public void addForceAtWorldPoint(Vec3 force, Vec3 worldPoint) {
    if (!isDynamic()) {
      return;
    }
    accumulatedForce = accumulatedForce.add(force);
    accumulatedTorque = accumulatedTorque.add(worldPoint.subtract(position).cross(force));
  }

  public Vec3 worldSpacePoint(Vec3 localPoint) {
    return position.add(orientation.rotate(localPoint));
  }

  public Vec3 worldSpaceVector(Vec3 localVector) {
    return orientation.rotate(localVector);
  }

  public Vec3 localSpaceVector(Vec3 worldVector) {
    return orientation.conjugate().rotate(worldVector);
  }

  public Vec3 pointVelocity(Vec3 worldPoint) {
    Vec3 r = worldPoint.subtract(position);
    return linearVelocity.add(angularVelocity.cross(r));
  }

  public Vec3 inverseInertiaWorldMultiply(Vec3 worldVector) {
    if (!isDynamic()) {
      return Vec3.ZERO;
    }
    Vec3 local = localSpaceVector(worldVector);
    Vec3 localResult = local.hadamard(inverseInertiaLocal);
    return worldSpaceVector(localResult);
  }

  public void integrateForces(double dtSeconds, Vec3 gravity) {
    if (!isDynamic()) {
      return;
    }
    linearVelocity =
        sanitizeVelocity(
            linearVelocity.add(gravity.add(accumulatedForce.scale(inverseMass)).scale(dtSeconds)),
            MAX_LINEAR_SPEED_MPS);
    angularVelocity =
        sanitizeVelocity(
            angularVelocity.add(inverseInertiaWorldMultiply(accumulatedTorque).scale(dtSeconds)),
            MAX_ANGULAR_SPEED_RAD_PER_SEC);
  }

  public void applyImpulseAtWorldPoint(Vec3 impulse, Vec3 worldPoint) {
    if (!isDynamic()) {
      return;
    }
    linearVelocity =
        sanitizeVelocity(linearVelocity.add(impulse.scale(inverseMass)), MAX_LINEAR_SPEED_MPS);
    Vec3 angularImpulse = worldPoint.subtract(position).cross(impulse);
    angularVelocity =
        sanitizeVelocity(
            angularVelocity.add(inverseInertiaWorldMultiply(angularImpulse)),
            MAX_ANGULAR_SPEED_RAD_PER_SEC);
  }

  public void integratePose(double dtSeconds) {
    if (!isDynamic()) {
      return;
    }
    position = sanitizeVector(position.add(linearVelocity.scale(dtSeconds)));
    orientation = sanitizeQuat(orientation.integrateAngularVelocity(angularVelocity, dtSeconds));
  }

  private Vec3 sanitizeVelocity(Vec3 velocity, double maxMagnitude) {
    if (!Double.isFinite(velocity.x())
        || !Double.isFinite(velocity.y())
        || !Double.isFinite(velocity.z())) {
      return Vec3.ZERO;
    }
    return PhysicsMath.clamp(velocity, maxMagnitude);
  }

  private Vec3 sanitizeVector(Vec3 vector) {
    if (!Double.isFinite(vector.x())
        || !Double.isFinite(vector.y())
        || !Double.isFinite(vector.z())) {
      return Vec3.ZERO;
    }
    return vector;
  }

  private Quat sanitizeQuat(Quat quat) {
    if (quat == null
        || !Double.isFinite(quat.w())
        || !Double.isFinite(quat.x())
        || !Double.isFinite(quat.y())
        || !Double.isFinite(quat.z())) {
      return Quat.IDENTITY;
    }
    return quat.normalize();
  }
}
