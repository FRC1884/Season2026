package org.Griffins1884.frc2026.simulation.engine;

import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Quat;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;

/**
 * Deterministic reduced articulated module model.
 *
 * <p>Each swerve pod is a real rigid body coupled to the chassis through an explicit deterministic
 * joint model: a translational mount constraint plus a yaw steering servo constraint. This is a
 * first-class articulated representation, even though it is reduced compared with a full generic
 * joint graph.
 */
public final class ArticulatedModuleAssembly {
  private static final double MOUNT_SPRING = 18000.0;
  private static final double MOUNT_DAMPING = 320.0;
  private static final double ALIGNMENT_SPRING = 900.0;
  private static final double ALIGNMENT_DAMPING = 42.0;
  private static final double YAW_SERVO_SPRING = 800.0;
  private static final double YAW_SERVO_DAMPING = 36.0;

  private final RigidBody chassisBody;
  private final RigidBody moduleBody;
  private final Vec3 chassisLocalAnchor;
  private final Vec3 moduleLocalAnchor;
  private double desiredSteerRadians;

  public ArticulatedModuleAssembly(
      RigidBody chassisBody,
      RigidBody moduleBody,
      Vec3 chassisLocalAnchor,
      Vec3 moduleLocalAnchor) {
    this.chassisBody = chassisBody;
    this.moduleBody = moduleBody;
    this.chassisLocalAnchor = chassisLocalAnchor;
    this.moduleLocalAnchor = moduleLocalAnchor;
  }

  public RigidBody moduleBody() {
    return moduleBody;
  }

  public RigidBody chassisBody() {
    return chassisBody;
  }

  public Vec3 chassisAnchorWorld() {
    return chassisBody.worldSpacePoint(chassisLocalAnchor);
  }

  public void setDesiredSteerRadians(double desiredSteerRadians) {
    this.desiredSteerRadians = desiredSteerRadians;
  }

  public void applyJointForces() {
    Vec3 chassisAnchorWorld = chassisBody.worldSpacePoint(chassisLocalAnchor);
    Vec3 moduleAnchorWorld = moduleBody.worldSpacePoint(moduleLocalAnchor);
    Vec3 positionError = moduleAnchorWorld.subtract(chassisAnchorWorld);
    Vec3 relativeVelocity =
        moduleBody
            .pointVelocity(moduleAnchorWorld)
            .subtract(chassisBody.pointVelocity(chassisAnchorWorld));
    Vec3 mountForce =
        positionError.scale(-MOUNT_SPRING).add(relativeVelocity.scale(-MOUNT_DAMPING));
    chassisBody.addForceAtWorldPoint(mountForce.negate(), chassisAnchorWorld);
    moduleBody.addForceAtWorldPoint(mountForce, moduleAnchorWorld);

    Vec3 chassisUp = chassisBody.orientation().axisZ().normalize();
    Vec3 moduleUp = moduleBody.orientation().axisZ().normalize();
    Vec3 tiltError = moduleUp.cross(chassisUp);
    Vec3 relativeAngularVelocity =
        moduleBody.angularVelocity().subtract(chassisBody.angularVelocity());
    Vec3 alignmentTorque =
        tiltError.scale(-ALIGNMENT_SPRING).add(relativeAngularVelocity.scale(-ALIGNMENT_DAMPING));
    chassisBody.addTorque(alignmentTorque.negate());
    moduleBody.addTorque(alignmentTorque);

    double chassisYaw = chassisBody.orientation().yawRadians();
    double moduleYaw = moduleBody.orientation().yawRadians();
    double yawError =
        edu.wpi.first.math.MathUtil.angleModulus(moduleYaw - (chassisYaw + desiredSteerRadians));
    Vec3 yawAxis = chassisUp;
    double relativeYawRate = relativeAngularVelocity.dot(yawAxis);
    Vec3 yawTorque =
        yawAxis.scale((-yawError * YAW_SERVO_SPRING) - (relativeYawRate * YAW_SERVO_DAMPING));
    chassisBody.addTorque(yawTorque.negate());
    moduleBody.addTorque(yawTorque);
  }

  public void resetPoseFromChassis() {
    Vec3 mountPosition = chassisBody.worldSpacePoint(chassisLocalAnchor);
    Quat orientation =
        Quat.fromYawPitchRoll(
            chassisBody.orientation().yawRadians() + desiredSteerRadians,
            chassisBody.orientation().toRotation3d().getY(),
            chassisBody.orientation().toRotation3d().getX());
    Vec3 moduleAnchorOffset = orientation.rotate(moduleLocalAnchor);
    moduleBody.setPose(mountPosition.subtract(moduleAnchorOffset), orientation);
    moduleBody.setVelocities(chassisBody.linearVelocity(), chassisBody.angularVelocity());
  }
}
