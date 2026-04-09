package org.Griffins1884.frc2026.simulation.contracts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.List;

/** Stable binary encoding for deterministic replay frames. */
public final class FrameBinaryCodec {
  private FrameBinaryCodec() {}

  public static byte peekFrameType(byte[] encoded) {
    if (encoded == null || encoded.length < 3) {
      throw new IllegalArgumentException("encoded frame too short");
    }
    return encoded[2];
  }

  public static byte[] encodeActuatorFrame(ActuatorFrame frame) {
    try {
      ByteArrayOutputStream buffer = new ByteArrayOutputStream();
      DataOutputStream out = new DataOutputStream(buffer);
      writeHeader(out, SimulationProtocolVersion.FRAME_TYPE_ACTUATOR);
      out.writeLong(frame.simTimeNanos());
      out.writeInt(frame.stepId());
      writePose2d(out, frame.robotPose());
      writeTranslation2d(out, frame.fieldVelocityMetersPerSecond());
      out.writeDouble(frame.turretYawRadians());
      out.writeDouble(frame.pivotMotorRotations());
      out.writeDouble(frame.shooterRpm());
      out.writeBoolean(frame.shooterArmed());
      out.flush();
      return buffer.toByteArray();
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  public static ActuatorFrame decodeActuatorFrame(byte[] encoded) {
    try {
      DataInputStream in = new DataInputStream(new ByteArrayInputStream(encoded));
      requireHeader(in, SimulationProtocolVersion.FRAME_TYPE_ACTUATOR);
      return new ActuatorFrame(
          in.readLong(),
          in.readInt(),
          readPose2d(in),
          readTranslation2d(in),
          in.readDouble(),
          in.readDouble(),
          in.readDouble(),
          in.readBoolean());
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  public static byte[] encodeSensorFrame(SensorFrame frame) {
    try {
      ByteArrayOutputStream buffer = new ByteArrayOutputStream();
      DataOutputStream out = new DataOutputStream(buffer);
      writeHeader(out, SimulationProtocolVersion.FRAME_TYPE_SENSOR);
      out.writeLong(frame.simTimeNanos());
      out.writeInt(frame.stepId());
      out.writeBoolean(frame.predictionAvailable());
      out.writeBoolean(frame.predictionFeasible());
      out.writeBoolean(frame.shotReleased());
      out.writeInt(frame.activeProjectileCount());
      out.writeInt(frame.projectileSpawnCount());
      out.writeDouble(frame.closestApproachErrorMeters());
      out.writeDouble(frame.timeOfFlightSeconds());
      out.flush();
      return buffer.toByteArray();
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  public static SensorFrame decodeSensorFrame(byte[] encoded) {
    try {
      DataInputStream in = new DataInputStream(new ByteArrayInputStream(encoded));
      requireHeader(in, SimulationProtocolVersion.FRAME_TYPE_SENSOR);
      return new SensorFrame(
          in.readLong(),
          in.readInt(),
          in.readBoolean(),
          in.readBoolean(),
          in.readBoolean(),
          in.readInt(),
          in.readInt(),
          in.readDouble(),
          in.readDouble());
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  public static byte[] encodeWorldSnapshot(WorldSnapshot snapshot) {
    try {
      ByteArrayOutputStream buffer = new ByteArrayOutputStream();
      DataOutputStream out = new DataOutputStream(buffer);
      writeHeader(out, SimulationProtocolVersion.FRAME_TYPE_WORLD_SNAPSHOT);
      out.writeLong(snapshot.simTimeNanos());
      out.writeInt(snapshot.stepId());
      out.writeBoolean(snapshot.predictionAvailable());
      out.writeBoolean(snapshot.predictionFeasible());
      writeNullablePose3d(out, snapshot.releasePose());
      writeNullablePose3d(out, snapshot.impactPose());
      out.writeInt(snapshot.activeProjectilePoses().size());
      for (Pose3d pose : snapshot.activeProjectilePoses()) {
        writeNullablePose3d(out, pose);
      }
      out.writeInt(snapshot.rigidBodyStates().size());
      for (RigidBodyState rigidBodyState : snapshot.rigidBodyStates()) {
        writeRigidBodyState(out, rigidBodyState);
      }
      out.writeInt(snapshot.contactPointStates().size());
      for (ContactPointState contactPointState : snapshot.contactPointStates()) {
        writeContactPointState(out, contactPointState);
      }
      out.writeInt(snapshot.wheelContactStates().size());
      for (WheelContactState wheelContactState : snapshot.wheelContactStates()) {
        writeWheelContactState(out, wheelContactState);
      }
      out.flush();
      return buffer.toByteArray();
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  public static WorldSnapshot decodeWorldSnapshot(byte[] encoded) {
    try {
      DataInputStream in = new DataInputStream(new ByteArrayInputStream(encoded));
      requireHeader(in, SimulationProtocolVersion.FRAME_TYPE_WORLD_SNAPSHOT);
      long simTimeNanos = in.readLong();
      int stepId = in.readInt();
      boolean predictionAvailable = in.readBoolean();
      boolean predictionFeasible = in.readBoolean();
      Pose3d releasePose = readNullablePose3d(in);
      Pose3d impactPose = readNullablePose3d(in);
      int projectileCount = in.readInt();
      List<Pose3d> projectilePoses = new ArrayList<>(projectileCount);
      for (int i = 0; i < projectileCount; i++) {
        projectilePoses.add(readNullablePose3d(in));
      }
      int rigidBodyCount = in.readInt();
      List<RigidBodyState> rigidBodyStates = new ArrayList<>(rigidBodyCount);
      for (int i = 0; i < rigidBodyCount; i++) {
        rigidBodyStates.add(readRigidBodyState(in));
      }
      int contactCount = in.readInt();
      List<ContactPointState> contactPointStates = new ArrayList<>(contactCount);
      for (int i = 0; i < contactCount; i++) {
        contactPointStates.add(readContactPointState(in));
      }
      int wheelContactCount = in.readInt();
      List<WheelContactState> wheelContactStates = new ArrayList<>(wheelContactCount);
      for (int i = 0; i < wheelContactCount; i++) {
        wheelContactStates.add(readWheelContactState(in));
      }
      return new WorldSnapshot(
          simTimeNanos,
          stepId,
          predictionAvailable,
          predictionFeasible,
          releasePose,
          impactPose,
          projectilePoses,
          rigidBodyStates,
          contactPointStates,
          wheelContactStates);
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  private static void writeHeader(DataOutputStream out, byte frameType) throws IOException {
    out.writeShort(SimulationProtocolVersion.VERSION);
    out.writeByte(frameType);
  }

  private static void requireHeader(DataInputStream in, byte expectedType) throws IOException {
    short version = in.readShort();
    byte frameType = in.readByte();
    if (version != SimulationProtocolVersion.VERSION) {
      throw new IOException("Unexpected frame version " + version);
    }
    if (frameType != expectedType) {
      throw new IOException("Unexpected frame type " + frameType);
    }
  }

  private static void writePose2d(DataOutputStream out, Pose2d pose) throws IOException {
    out.writeDouble(pose.getX());
    out.writeDouble(pose.getY());
    out.writeDouble(pose.getRotation().getRadians());
  }

  private static Pose2d readPose2d(DataInputStream in) throws IOException {
    return new Pose2d(in.readDouble(), in.readDouble(), Rotation2d.fromRadians(in.readDouble()));
  }

  private static void writeTranslation2d(DataOutputStream out, Translation2d translation)
      throws IOException {
    out.writeDouble(translation.getX());
    out.writeDouble(translation.getY());
  }

  private static Translation2d readTranslation2d(DataInputStream in) throws IOException {
    return new Translation2d(in.readDouble(), in.readDouble());
  }

  private static void writeNullablePose3d(DataOutputStream out, Pose3d pose) throws IOException {
    out.writeBoolean(pose != null);
    if (pose != null) {
      out.writeDouble(pose.getX());
      out.writeDouble(pose.getY());
      out.writeDouble(pose.getZ());
      Quaternion quaternion = pose.getRotation().getQuaternion();
      out.writeDouble(quaternion.getW());
      out.writeDouble(quaternion.getX());
      out.writeDouble(quaternion.getY());
      out.writeDouble(quaternion.getZ());
    }
  }

  private static Pose3d readNullablePose3d(DataInputStream in) throws IOException {
    if (!in.readBoolean()) {
      return null;
    }
    return new Pose3d(
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        new Rotation3d(
            new Quaternion(in.readDouble(), in.readDouble(), in.readDouble(), in.readDouble())));
  }

  private static void writeRigidBodyState(DataOutputStream out, RigidBodyState state)
      throws IOException {
    out.writeInt(state.id());
    out.writeUTF(state.name());
    out.writeUTF(state.shapeType());
    out.writeBoolean(state.dynamic());
    out.writeDouble(state.positionX());
    out.writeDouble(state.positionY());
    out.writeDouble(state.positionZ());
    out.writeDouble(state.quaternionW());
    out.writeDouble(state.quaternionX());
    out.writeDouble(state.quaternionY());
    out.writeDouble(state.quaternionZ());
    out.writeDouble(state.linearVelocityX());
    out.writeDouble(state.linearVelocityY());
    out.writeDouble(state.linearVelocityZ());
    out.writeDouble(state.angularVelocityX());
    out.writeDouble(state.angularVelocityY());
    out.writeDouble(state.angularVelocityZ());
  }

  private static RigidBodyState readRigidBodyState(DataInputStream in) throws IOException {
    return new RigidBodyState(
        in.readInt(),
        in.readUTF(),
        in.readUTF(),
        in.readBoolean(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble());
  }

  private static void writeContactPointState(DataOutputStream out, ContactPointState state)
      throws IOException {
    out.writeInt(state.bodyAId());
    out.writeInt(state.bodyBId());
    out.writeDouble(state.pointX());
    out.writeDouble(state.pointY());
    out.writeDouble(state.pointZ());
    out.writeDouble(state.normalX());
    out.writeDouble(state.normalY());
    out.writeDouble(state.normalZ());
    out.writeDouble(state.penetrationDepth());
    out.writeDouble(state.normalImpulse());
    out.writeDouble(state.tangentImpulse1());
    out.writeDouble(state.tangentImpulse2());
  }

  private static ContactPointState readContactPointState(DataInputStream in) throws IOException {
    return new ContactPointState(
        in.readInt(),
        in.readInt(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble());
  }

  private static void writeWheelContactState(DataOutputStream out, WheelContactState state)
      throws IOException {
    out.writeInt(state.wheelIndex());
    out.writeBoolean(state.contacting());
    out.writeDouble(state.pointX());
    out.writeDouble(state.pointY());
    out.writeDouble(state.pointZ());
    out.writeDouble(state.normalX());
    out.writeDouble(state.normalY());
    out.writeDouble(state.normalZ());
    out.writeDouble(state.normalForceNewtons());
    out.writeDouble(state.slipRatio());
    out.writeDouble(state.slipAngleRadians());
    out.writeDouble(state.longitudinalForceX());
    out.writeDouble(state.longitudinalForceY());
    out.writeDouble(state.longitudinalForceZ());
    out.writeDouble(state.lateralForceX());
    out.writeDouble(state.lateralForceY());
    out.writeDouble(state.lateralForceZ());
  }

  private static WheelContactState readWheelContactState(DataInputStream in) throws IOException {
    return new WheelContactState(
        in.readInt(),
        in.readBoolean(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble(),
        in.readDouble());
  }
}
