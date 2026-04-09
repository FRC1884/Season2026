package org.Griffins1884.frc2026.simulation.engine;

import java.util.ArrayList;
import java.util.List;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Quat;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;
import org.Griffins1884.frc2026.simulation.maple.Rebuilt2026FieldModel;

/** Physically grounded per-wheel contact patch model for articulated module pod bodies. */
public final class SwerveWheelContactModel {
  private static final double SUSPENSION_SPRING = 2400.0;
  private static final double SUSPENSION_DAMPING = 95.0;
  private static final double LONGITUDINAL_GRIP_GAIN = 4.0;
  private static final double LATERAL_GRIP_GAIN = 3.5;

  private final ArticulatedModuleAssembly[] modules;
  private final double wheelRadiusMeters;
  private final WheelCommand[] commands;
  private final List<WheelContactTelemetry> lastTelemetry = new ArrayList<>();

  public SwerveWheelContactModel(ArticulatedModuleAssembly[] modules, double wheelRadiusMeters) {
    this.modules = modules.clone();
    this.wheelRadiusMeters = wheelRadiusMeters;
    commands = new WheelCommand[this.modules.length];
    for (int i = 0; i < commands.length; i++) {
      commands[i] = new WheelCommand();
    }
  }

  public void setCommand(
      int wheelIndex, double steerRadians, double wheelAngularVelocityRadPerSec) {
    commands[wheelIndex].steerRadians = steerRadians;
    commands[wheelIndex].wheelAngularVelocityRadPerSec = wheelAngularVelocityRadPerSec;
  }

  public List<WheelContactTelemetry> lastTelemetry() {
    return List.copyOf(lastTelemetry);
  }

  public void apply(PhysicsWorld world, double dtSeconds) {
    lastTelemetry.clear();
    for (int wheelIndex = 0; wheelIndex < modules.length; wheelIndex++) {
      RigidBody moduleBody = modules[wheelIndex].moduleBody();
      Quat orientation = moduleBody.orientation();
      Vec3 wheelCenter = moduleBody.position();
      Rebuilt2026FieldModel.LocalTerrainSample terrainSample =
          Rebuilt2026FieldModel.sample(
              new edu.wpi.first.math.geometry.Pose2d(
                  wheelCenter.x(), wheelCenter.y(), new edu.wpi.first.math.geometry.Rotation2d()));
      Vec3 normal = terrainNormal(terrainSample);
      double penetration = terrainSample.heightMeters() + wheelRadiusMeters - wheelCenter.z();
      if (penetration <= 0.0) {
        lastTelemetry.add(
            new WheelContactTelemetry(
                wheelIndex, false, wheelCenter, normal, 0.0, 0.0, 0.0, Vec3.ZERO, Vec3.ZERO));
        continue;
      }

      Vec3 contactPoint = wheelCenter.subtract(normal.scale(wheelRadiusMeters));
      Vec3 contactVelocity = moduleBody.pointVelocity(contactPoint);
      Vec3 forward = orientation.axisX().normalize();
      Vec3 lateral = normal.cross(forward).normalize();
      double longitudinalVelocity = contactVelocity.dot(forward);
      double lateralVelocity = contactVelocity.dot(lateral);
      double normalVelocity = contactVelocity.dot(normal);

      double desiredSurfaceSpeed =
          commands[wheelIndex].wheelAngularVelocityRadPerSec * wheelRadiusMeters;
      double slipRatio =
          (desiredSurfaceSpeed - longitudinalVelocity)
              / Math.max(
                  Math.max(Math.abs(desiredSurfaceSpeed), Math.abs(longitudinalVelocity)), 0.5);
      double slipAngle = Math.atan2(lateralVelocity, Math.abs(longitudinalVelocity) + 0.5);

      double normalForceMagnitude =
          Math.max(0.0, (penetration * SUSPENSION_SPRING) - (normalVelocity * SUSPENSION_DAMPING));
      double tractionScale =
          Math.tanh(LONGITUDINAL_GRIP_GAIN * slipRatio)
              * moduleBody.material().frictionCoefficient();
      double scrubScale =
          Math.tanh(LATERAL_GRIP_GAIN * slipAngle)
              * moduleBody.material().lateralFrictionCoefficient();

      Vec3 normalForce = normal.scale(normalForceMagnitude);
      Vec3 longitudinalForce = forward.scale(normalForceMagnitude * tractionScale);
      Vec3 lateralForce = lateral.scale(-normalForceMagnitude * scrubScale);
      Vec3 rollingResistance =
          forward.scale(
              -Math.signum(longitudinalVelocity)
                  * moduleBody.material().rollingResistanceCoefficient()
                  * normalForceMagnitude);
      Vec3 netForce = normalForce.add(longitudinalForce).add(lateralForce).add(rollingResistance);
      moduleBody.addForceAtWorldPoint(netForce, contactPoint);
      modules[wheelIndex]
          .chassisBody()
          .addForceAtWorldPoint(netForce, modules[wheelIndex].chassisAnchorWorld());

      lastTelemetry.add(
          new WheelContactTelemetry(
              wheelIndex,
              true,
              contactPoint,
              normal,
              normalForceMagnitude,
              slipRatio,
              slipAngle,
              longitudinalForce,
              lateralForce));
    }
  }

  private Vec3 terrainNormal(Rebuilt2026FieldModel.LocalTerrainSample sample) {
    double pitch = sample.pitchRadians();
    double roll = sample.rollRadians();
    return new Vec3(-Math.tan(pitch), Math.tan(roll), 1.0).normalize();
  }

  private static final class WheelCommand {
    private double steerRadians;
    private double wheelAngularVelocityRadPerSec;
  }
}
