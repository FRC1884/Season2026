package org.Griffins1884.frc2026.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record VisionFieldPoseEstimate(
    Pose2d visionRobotPoseMeters,
    double timestampSeconds,
    Matrix<N3, N1> visionMeasurementStdDevs,
    int numTags) {
  public VisionFieldPoseEstimate {
    if (visionRobotPoseMeters == null) {
      visionRobotPoseMeters = new Pose2d();
    }
  }
}
