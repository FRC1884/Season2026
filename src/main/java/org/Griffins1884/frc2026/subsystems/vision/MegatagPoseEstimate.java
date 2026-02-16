package org.Griffins1884.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Represents a robot pose estimate using multiple AprilTags (Megatag).
 *
 * @param fieldToRobot The estimated robot pose on the field
 * @param timestampSeconds The timestamp when this estimate was captured
 * @param latency Processing latency in seconds
 * @param avgTagArea Average area of detected tags
 * @param avgTagDist Average distance to detected tags
 * @param quality Quality score of the pose estimate (0-1)
 * @param fiducialIds IDs of fiducials used for this estimate
 */
public record MegatagPoseEstimate(
    Pose2d fieldToRobot,
    double timestampSeconds,
    double latency,
    double avgTagArea,
    double avgTagDist,
    double quality,
    int[] fiducialIds,
    double residualTranslation) {

  public MegatagPoseEstimate {
    if (fieldToRobot == null) {
      fieldToRobot = new Pose2d();
    }
    if (fiducialIds == null) {
      fiducialIds = new int[0];
    }
  }

  /** Converts a Limelight pose estimate to a MegatagPoseEstimate. */
  public static MegatagPoseEstimate fromLimelight(LimelightHelpers.PoseEstimate poseEstimate) {
    Pose2d fieldToRobot = poseEstimate.pose;
    if (fieldToRobot == null) {
      fieldToRobot = new Pose2d();
    }
    LimelightHelpers.RawFiducial[] rawFiducials =
        poseEstimate.rawFiducials != null
            ? poseEstimate.rawFiducials
            : new LimelightHelpers.RawFiducial[0];
    int[] fiducialIds = new int[rawFiducials.length];
    for (int i = 0; i < rawFiducials.length; i++) {
      if (rawFiducials[i] != null) {
        fiducialIds[i] = rawFiducials[i].id;
      }
    }
    double quality;
    if (rawFiducials.length > 1) {
      quality = 1.0;
    } else if (rawFiducials.length == 1) {
      double ambiguity = rawFiducials[0].ambiguity;
      quality = Double.isFinite(ambiguity) ? 1.0 - ambiguity : 0.0;
    } else {
      quality = 0.0;
    }
    if (!Double.isFinite(quality)) {
      quality = 0.0;
    } else if (quality < 0.0) {
      quality = 0.0;
    } else if (quality > 1.0) {
      quality = 1.0;
    }
    return new MegatagPoseEstimate(
        fieldToRobot,
        poseEstimate.timestampSeconds,
        poseEstimate.latency,
        poseEstimate.avgTagArea,
        poseEstimate.avgTagDist,
        quality,
        fiducialIds,
        poseEstimate.residualTranslation);
  }
}
