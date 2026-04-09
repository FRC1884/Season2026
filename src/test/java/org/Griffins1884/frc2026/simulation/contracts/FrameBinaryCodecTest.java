package org.Griffins1884.frc2026.simulation.contracts;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.junit.jupiter.api.Test;

class FrameBinaryCodecTest {
  @Test
  void actuatorFrameRoundTripsThroughCanonicalBinaryEncoding() {
    ActuatorFrame frame =
        new ActuatorFrame(
            42L,
            3,
            new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(15.0)),
            new Translation2d(0.4, -0.2),
            0.7,
            0.3,
            3600.0,
            true);

    ActuatorFrame decoded =
        FrameBinaryCodec.decodeActuatorFrame(FrameBinaryCodec.encodeActuatorFrame(frame));

    assertEquals(frame.simTimeNanos(), decoded.simTimeNanos());
    assertEquals(frame.stepId(), decoded.stepId());
    assertEquals(frame.robotPose().getX(), decoded.robotPose().getX());
    assertEquals(frame.robotPose().getY(), decoded.robotPose().getY());
    assertEquals(
        frame.robotPose().getRotation().getRadians(),
        decoded.robotPose().getRotation().getRadians());
    assertEquals(
        frame.fieldVelocityMetersPerSecond().getX(), decoded.fieldVelocityMetersPerSecond().getX());
    assertEquals(
        frame.fieldVelocityMetersPerSecond().getY(), decoded.fieldVelocityMetersPerSecond().getY());
    assertEquals(frame.turretYawRadians(), decoded.turretYawRadians());
    assertEquals(frame.pivotMotorRotations(), decoded.pivotMotorRotations());
    assertEquals(frame.shooterRpm(), decoded.shooterRpm());
    assertEquals(frame.shooterArmed(), decoded.shooterArmed());
  }

  @Test
  void worldSnapshotRoundTripsNullablePosesAndProjectileList() {
    WorldSnapshot snapshot =
        new WorldSnapshot(
            100L,
            7,
            true,
            false,
            new Pose3d(1.0, 2.0, 3.0, new Rotation3d(0.1, 0.2, 0.3)),
            null,
            List.of(new Pose3d(4.0, 5.0, 6.0, new Rotation3d()), new Pose3d()));

    WorldSnapshot decoded =
        FrameBinaryCodec.decodeWorldSnapshot(FrameBinaryCodec.encodeWorldSnapshot(snapshot));

    assertEquals(snapshot.simTimeNanos(), decoded.simTimeNanos());
    assertEquals(snapshot.stepId(), decoded.stepId());
    assertTrue(decoded.predictionAvailable());
    assertFalse(decoded.predictionFeasible());
    assertNotNull(decoded.releasePose());
    assertNull(decoded.impactPose());
    assertEquals(2, decoded.activeProjectilePoses().size());
    assertEquals(snapshot.releasePose().getX(), decoded.releasePose().getX());
    assertEquals(
        snapshot.releasePose().getRotation().getQuaternion().getY(),
        decoded.releasePose().getRotation().getQuaternion().getY(),
        1e-12);
  }
}
