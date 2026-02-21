package org.Griffins1884.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;

class VisionReliabilityHardeningTest {
  @Test
  void periodicLimelight_suppressedStillProcessesReconnectReset() throws Exception {
    StubVisionIO io = new StubVisionIO("vision-test");
    Vision vision = new Vision((pose, timestamp, stdDevs) -> {}, Pose2d::new, () -> 0.0, io);

    double[] lastAccepted = (double[]) readField(vision, "lastAcceptedTimestampsSec");
    boolean[] wasConnected = (boolean[]) readField(vision, "wasConnected");
    lastAccepted[0] = 42.0;
    wasConnected[0] = false;

    io.connected = true;
    vision.suppressVisionForSeconds(1.0);
    vision.periodic();

    assertEquals(Double.NEGATIVE_INFINITY, lastAccepted[0]);
    assertTrue(wasConnected[0]);
  }

  @Test
  void periodicLimelight_disconnectedCamera_setsRejectReason() throws Exception {
    StubVisionIO io = new StubVisionIO("vision-test");
    Vision vision = new Vision((pose, timestamp, stdDevs) -> {}, Pose2d::new, () -> 0.0, io);

    io.connected = false;
    vision.periodic();

    Object[] visionInputs = (Object[]) readField(vision, "inputs");
    Object input = visionInputs[0];
    Field rejectReasonField = input.getClass().getField("rejectReason");
    Object rejectReason = rejectReasonField.get(input);
    assertEquals(VisionIO.RejectReason.DISCONNECTED, rejectReason);
  }

  @Test
  void maxAcceptableFrameAge_isLooserThanYawGateFrameAge() {
    assertTrue(
        AprilTagVisionConstants.getLimelightMaxAcceptableFrameAgeSec()
            > AprilTagVisionConstants.getLimelightYawGateMaxFrameAgeSec());
    assertTrue(
        AprilTagVisionConstants.getLimelightMaxAcceptableFrameAgeSec(VisionIO.LimelightProfile.LL3)
            > AprilTagVisionConstants.getLimelightYawGateMaxFrameAgeSec(
                VisionIO.LimelightProfile.LL3));
    assertFalse(AprilTagVisionConstants.getLimelightMaxAcceptableFrameAgeSec() <= 0.0);
  }

  private static Object readField(Object target, String name) throws Exception {
    Field field = target.getClass().getDeclaredField(name);
    field.setAccessible(true);
    return field.get(target);
  }

  private static final class StubVisionIO implements VisionIO {
    private final CameraConstants constants;
    private boolean connected;

    private StubVisionIO(String name) {
      this.constants = new CameraConstants(name, new Transform3d(), CameraType.LIMELIGHT);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.connected = connected;
      inputs.fiducialObservations = new FiducialObservation[0];
      inputs.tagIds = new int[0];
      inputs.poseObservations = new PoseObservation[0];
      inputs.megatagPoseEstimate = null;
    }

    @Override
    public CameraConstants getCameraConstants() {
      return constants;
    }
  }
}
