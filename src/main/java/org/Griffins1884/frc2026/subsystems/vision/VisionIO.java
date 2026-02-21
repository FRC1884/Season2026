package org.Griffins1884.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  /**
   * An enum containing every type of camera we might have on our robot, and thresholds of distances
   * to tags at which we think they will provide meaningful data on a target.
   */
  enum CameraType {
    OV2311(7.5, 100),
    OV9281(4.5, 100),
    LIMELIGHT(12.5, 100),
    LIMELIGHT_3G(12.0, 100),
    A1920(8.0, 100),
    TELEPHOTO_OV2311(9.0, 100),
    TELEPHOTO_OV9281(9.0, 100),
    TELEPHOTO_A1920(10.0, 100),
    TELEPHOTO_LIMELIGHT(7.5, 100),
    TELEPHOTO_LIMELIGHT_3G(15.0, 100),
    UNKNOWN(0.0, 3);

    public final double noisyDistance;
    public final double noisySpeed;

    CameraType(double noisyDistance, double noisySpeed) {
      this.noisyDistance = noisyDistance;
      this.noisySpeed = noisySpeed;
    }
  }

  enum LimelightProfile {
    AUTO,
    LL3,
    LL4
  }

  public static enum RejectReason {
    VISION_DISABLED,
    DISCONNECTED,
    NO_MEGATAG,
    POSE_NONFINITE,
    NO_TAGS,
    LOW_SINGLE_TAG_QUALITY,
    OUT_OF_FIELD,
    STDDEV_NONFINITE,
    RESIDUAL_OUTLIER,
    LARGE_ROTATION_RESIDUAL,
    LARGE_TRANSLATION_RESIDUAL,
    UNKNOWN,
    ACCEPTED
  }

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public boolean seesTarget = false;
    public int megatagCount = 0;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public Pose3d pose3d = null;
    public MegatagPoseEstimate megatagPoseEstimate = null;
    public FiducialObservation[] fiducialObservations = new FiducialObservation[0];
    public double[] standardDeviations = new double[0];
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public double residualTranslationMeters = 0.0;
    public RejectReason rejectReason = RejectReason.UNKNOWN;
    public String limelightProfile = LimelightProfile.LL4.name();
    public String limelightProfileSource = "DEFAULT";
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default CameraConstants getCameraConstants() {
    return new CameraConstants("UNKNOWN", new Transform3d(), CameraType.UNKNOWN);
  }

  /**
   * Represents the angle to a tag, not used for pose estimation.
   *
   * @param tx the horizontal angle (yaw) to the target, in degrees. Positive-right, center-zero.
   * @param ty the vertical angle (pitch) to the target, in degrees. Positive-down, center-zero.
   */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /**
   * Represents a robot pose sample used for pose estimation.
   *
   * @param timestamp the timestamp of the estimate, in seconds.
   * @param pose the estimated pose, in meters.
   * @param ambiguity the ambiguity factor of the pose estimate.
   * @param tagCount the total number of tags seen.
   * @param averageTagDistance the average distance to all tags in frame, in meters.
   */
  public static record PoseObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

  /**
   * A data class representing all the information we need on a camera to get us from it being
   * connected and streaming, to a filtered pose estimate we can fuse with other cameras' estimates
   * and the drivetrain's odometry.
   *
   * @param cameraName the NetworkTables name for this camera. This is what will help us identify
   *     each camera on AdvantageScope and Elastic, to help with debugging.
   * @param robotToCamera the transformation representing the camera's position relative to the
   *     center of the robot. Translations are in meters.
   * @param cameraType the type of hardware camera we are using. See {@link CameraType}.
   */
  public static record CameraConstants(
      String cameraName, Transform3d robotToCamera, CameraType cameraType) {}
}
