package org.Griffins1884.frc2026.util;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.PhotonPoseEstimator;

/** Purely for experimentation, DNT */
@Deprecated
public final class PhotonVisionHelpers {
  private static Pose2d robotPose = null;
  private static PhotonPoseEstimator photonPoseEstimator;
  private static SwerveDrivePoseEstimator poseEstimator;
  private static Kinematics kinematics;

  static enum CameraType {
    LIMELIGHT,
    LIMELIGHT_3G;
  }

  public static record Camera(String cameraName) {}

  public static void use(SwerveDrivePoseEstimator test, Camera... cameras) {
    // poseEstimator = new SwerveDrivePoseEstimator();
  }

  /**
   * Use this method to get the final, filtered robot pose.
   *
   * @return the filtered robot pose on the field
   */
  public static Pose2d getRobotPose() {
    try {
      return robotPose;
    } catch (NullPointerException e) {
      DriverStation.reportError(
          "The robot pose could not be found! " + "Did you call PhotonVisionHelpers.____()?", true);
      return null;
    }
  }

  private PhotonVisionHelpers() {}
}
