package org.Griffins1884.frc2026.subsystems.vision;

import java.util.Arrays;
import java.util.Objects;

/**
 * Represents an observation of a fiducial marker (AprilTag) with position and quality data.
 *
 * @param id The fiducial marker ID
 * @param txnc Normalized horizontal offset (-1 to 1)
 * @param tync Normalized vertical offset (-1 to 1)
 * @param ambiguity Pose ambiguity score (0 = confident, 1 = ambiguous)
 * @param area Target area as percentage of image
 */
public record FiducialObservation(int id, double txnc, double tync, double ambiguity, double area) {

  /** Converts a Limelight raw fiducial to a FiducialObservation. */
  public static FiducialObservation fromLimelight(LimelightHelpers.RawFiducial fiducial) {
    if (fiducial == null) {
      return null;
    }
    return new FiducialObservation(
        fiducial.id, fiducial.txnc, fiducial.tync, fiducial.ambiguity, fiducial.ta);
  }

  /** Converts an array of Limelight raw fiducials to FiducialObservation array. */
  public static FiducialObservation[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
    if (fiducials == null) {
      return new FiducialObservation[0];
    }
    return Arrays.stream(fiducials)
        .map(FiducialObservation::fromLimelight)
        .filter(Objects::nonNull)
        .toArray(FiducialObservation[]::new);
  }
}
