package org.Griffins1884.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.lang.reflect.Method;
import org.junit.jupiter.api.Test;

class VisionObservationValidityTest {
  private static Vision createVision() {
    VisionIO dummyIO =
        new VisionIO() {
          @Override
          public CameraConstants getCameraConstants() {
            return new CameraConstants("test", new Transform3d(), CameraType.UNKNOWN);
          }
        };
    return new Vision((pose, timestamp, stdDevs) -> {}, dummyIO);
  }

  private static boolean invokeIsObservationValid(Vision vision, VisionIO.PoseObservation obs)
      throws Exception {
    Method method = Vision.class.getDeclaredMethod("isObservationValid", VisionIO.PoseObservation.class);
    method.setAccessible(true);
    return (boolean) method.invoke(vision, obs);
  }

  @Test
  void rejectsHighAmbiguity() throws Exception {
    Vision vision = createVision();
    VisionIO.PoseObservation observation =
        new VisionIO.PoseObservation(
            1.0,
            new Pose3d(0.0, 0.0, 0.0, new Rotation3d()),
            AprilTagVisionConstants.MAX_AMBIGUITY_CUTOFF + 0.05,
            1,
            2.0);

    assertFalse(invokeIsObservationValid(vision, observation));
  }

  @Test
  void rejectsLargeZError() throws Exception {
    Vision vision = createVision();
    VisionIO.PoseObservation observation =
        new VisionIO.PoseObservation(
            1.0,
            new Pose3d(0.0, 0.0, AprilTagVisionConstants.MAX_Z_ERROR + 0.1, new Rotation3d()),
            0.0,
            1,
            2.0);

    assertFalse(invokeIsObservationValid(vision, observation));
  }

  @Test
  void acceptsValidObservation() throws Exception {
    Vision vision = createVision();
    VisionIO.PoseObservation observation =
        new VisionIO.PoseObservation(
            1.0,
            new Pose3d(0.0, 0.0, 0.0, new Rotation3d()),
            0.0,
            1,
            2.0);

    assertTrue(invokeIsObservationValid(vision, observation));
  }
}
