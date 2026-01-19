package org.Griffins1884.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Represents a robot pose estimate using multiple AprilTags (Megatag).
 *
 * @param fieldToRobot The estimated robot pose on the field
 * @param timestampSeconds The timestamp when this estimate was captured
 * @param latency Processing latency in seconds
 * @param avgTagArea Average area of detected tags
 * @param quality Quality score of the pose estimate (0-1)
 * @param fiducialIds IDs of fiducials used for this estimate
 */
public record MegatagPoseEstimate(
    Pose2d fieldToRobot,
    double timestampSeconds,
    double latency,
    double avgTagArea,
    double quality,
    int[] fiducialIds) {

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
    int[] fiducialIds = new int[poseEstimate.rawFiducials.length];
    for (int i = 0; i < poseEstimate.rawFiducials.length; i++) {
      if (poseEstimate.rawFiducials[i] != null) {
        fiducialIds[i] = poseEstimate.rawFiducials[i].id;
      }
    }
    return new MegatagPoseEstimate(
        fieldToRobot,
        poseEstimate.timestampSeconds,
        poseEstimate.latency,
        poseEstimate.avgTagArea,
        fiducialIds.length > 1 ? 1.0 : 1.0 - poseEstimate.rawFiducials[0].ambiguity,
        fiducialIds);
  }
}
