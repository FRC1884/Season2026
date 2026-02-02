package org.Griffins1884.frc2026.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.Griffins1884.frc2026.subsystems.vision.VisionIO.CameraType;
import org.Griffins1884.frc2026.subsystems.vision.VisionIO.PoseObservation;

/**
 * This class provides utility methods and record classes for vision-related operations,
 * specifically for pose estimation using April Tags.
 */
public class AprilTagVisionHelpers {
  /**
   * Cool idea by the vision template & the PoseEstimator docs – dynamically update the vision
   * standard deviations based on the distance. In this case, we're scaling the deviations
   * quadratically with distance, so faraway tags are trusted <i>way</i> less than close ones.
   */
  public static Matrix<N3, N1> generateDynamicStdDevs(
      PoseObservation observation, CameraType type) {
    double poseAmbiguityFactor =
        observation.tagCount() > 1
            ? 1
            : Math.max(
                1,
                (observation.ambiguity() + AprilTagVisionConstants.POSE_AMBIGUITY_SHIFTER)
                    * AprilTagVisionConstants.POSE_AMBIGUITY_MULTIPLIER);

    double confidenceMultiplier =
        Math.max(
            1,
            (Math.max(
                        1,
                        Math.max(0, observation.averageTagDistance() - type.noisyDistance)
                            * AprilTagVisionConstants.DISTANCE_WEIGHT)
                    * poseAmbiguityFactor)
                / (1
                    + ((observation.tagCount() - 1)
                        * AprilTagVisionConstants.TAG_PRESENCE_WEIGHT)));

    return AprilTagVisionConstants.getVisionMeasurementStdDevs().times(confidenceMultiplier);
  }
}
