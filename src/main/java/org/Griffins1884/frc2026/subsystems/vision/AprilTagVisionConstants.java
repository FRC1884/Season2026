package org.Griffins1884.frc2026.subsystems.vision;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static org.Griffins1884.frc2026.GlobalConstants.ROBOT;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

// TODO tune all of these!! Make sure that the robot-relative coords are correct - this will cause
// rapid pose oscillation
public final class AprilTagVisionConstants {
  public static final boolean IS_LIMELIGHT = true;
  private static final VisionIO.CameraType LIMELIGHT_TYPE_HINT = VisionIO.CameraType.LIMELIGHT;

  private static VisionIO.CameraType getPrimaryCameraType() {
    return IS_LIMELIGHT ? LIMELIGHT_TYPE_HINT : VisionIO.CameraType.OV9281;
  }

  public static final boolean LEFT_CAM_ENABLED = true;
  public static final VisionIO.CameraConstants LEFT_CAM_CONSTANTS =
      switch (ROBOT) {
        case DEVBOT ->
            new VisionIO.CameraConstants(
                (IS_LIMELIGHT) ? "limelight-left" : "lefttagcam",
                new Transform3d(
                    0.290868,
                    -0.288925,
                    0.205,
                    new Rotation3d(
                        degreesToRadians(180), degreesToRadians(5), degreesToRadians(0))),
                getPrimaryCameraType());
        case COMPBOT ->
            new VisionIO.CameraConstants(
                (IS_LIMELIGHT) ? "limelight-left" : "lefttagcam",
                new Transform3d(
                    0.30368,
                    -0.29575,
                    0.22454,
                    new Rotation3d(
                        degreesToRadians(180), degreesToRadians(15), degreesToRadians(20))),
                getPrimaryCameraType());
        case SIMBOT, CRESCENDO ->
            new VisionIO.CameraConstants(
                (IS_LIMELIGHT) ? "limelight-left" : "lefttagcam",
                new Transform3d(
                    0.3006,
                    0.3056,
                    0.245,
                    new Rotation3d(0, degreesToRadians(-5), degreesToRadians(-20))),
                getPrimaryCameraType());
      };

  public static final boolean RIGHT_CAM_ENABLED = true;
  public static final VisionIO.CameraConstants RIGHT_CAM_CONSTANTS =
      switch (ROBOT) {
        case DEVBOT ->
            new VisionIO.CameraConstants(
                (IS_LIMELIGHT) ? "limelight-right" : "righttagcam",
                new Transform3d(
                    0.290868,
                    0.288922,
                    0.205,
                    new Rotation3d(
                        degreesToRadians(180), degreesToRadians(5), degreesToRadians(0))),
                getPrimaryCameraType());
        case COMPBOT ->
            new VisionIO.CameraConstants(
                (IS_LIMELIGHT) ? "limelight-right" : "righttagcam",
                new Transform3d(
                    0.30368,
                    0.29575,
                    0.22454,
                    new Rotation3d(
                        degreesToRadians(180), degreesToRadians(15), degreesToRadians(-20))),
                getPrimaryCameraType());
        case SIMBOT, CRESCENDO ->
            new VisionIO.CameraConstants(
                (IS_LIMELIGHT) ? "limelight-right" : "righttagcam",
                new Transform3d(
                    0.3006,
                    -0.3056,
                    0.245,
                    new Rotation3d(0, degreesToRadians(-5), degreesToRadians(20))),
                getPrimaryCameraType());
      };

  public static final boolean BACK_CAM_ENABLED = false;
  public static final VisionIO.CameraConstants BACK_CAM_CONSTANTS =
      switch (ROBOT) {
        case DEVBOT ->
            new VisionIO.CameraConstants(
                (IS_LIMELIGHT) ? "limelight-back" : "backtagcam",
                new Transform3d(
                    -0.29176,
                    0.289464,
                    0.205,
                    new Rotation3d(
                        degreesToRadians(180), degreesToRadians(5), degreesToRadians(180))),
                getPrimaryCameraType());
        case COMPBOT, SIMBOT, CRESCENDO ->
            new VisionIO.CameraConstants(
                (IS_LIMELIGHT) ? "limelight-back" : "backtagcam",
                new Transform3d(
                    -0.3006,
                    0.3056,
                    0.245,
                    new Rotation3d(0, degreesToRadians(-20), degreesToRadians(180))),
                getPrimaryCameraType());
      };

  private static final LoggedTunableNumber VISION_STDDEV_X =
      new LoggedTunableNumber("AprilTagVision/StdDev/X", 2.0);
  private static final LoggedTunableNumber VISION_STDDEV_Y =
      new LoggedTunableNumber("AprilTagVision/StdDev/Y", 2.0);
  private static final LoggedTunableNumber VISION_STDDEV_THETA =
      new LoggedTunableNumber("AprilTagVision/StdDev/Theta", 1.0);

  private static final LoggedTunableNumber LIMELIGHT_LARGE_VARIANCE =
      new LoggedTunableNumber("AprilTagVision/Limelight/LargeVariance", 1e6);
  private static final LoggedTunableNumber LIMELIGHT_IGNORE_MEGATAG2_ROTATION =
      new LoggedTunableNumber("AprilTagVision/Limelight/IgnoreMegaTag2Rotation", 1.0);
  private static final LoggedTunableNumber LIMELIGHT_PROFILE_OVERRIDE =
      new LoggedTunableNumber("AprilTagVision/Limelight/ProfileOverride", 0.0);
  private static final LoggedTunableNumber MEGATAG2_SINGLE_TAG_QUALITY_CUTOFF =
      new LoggedTunableNumber("AprilTagVision/Limelight/SingleTagQualityCutoff", 0.3);
  private static final LoggedTunableNumber MEGATAG2_SINGLE_TAG_QUALITY_CUTOFF_LL3 =
      new LoggedTunableNumber("AprilTagVision/Limelight/SingleTagQualityCutoffLL3", 0.45);
  private static final LoggedTunableNumber LIMELIGHT_MAX_YAW_RATE_DEG_PER_SEC =
      new LoggedTunableNumber("AprilTagVision/Limelight/MaxYawRateDegPerSec", 180.0);
  private static final LoggedTunableNumber FIELD_BORDER_MARGIN_METERS =
      new LoggedTunableNumber("AprilTagVision/FieldBorderMarginMeters", 0.5);
  public static final int LIMELIGHT_MEGATAG2_X_STDDEV_INDEX = 3;
  public static final int LIMELIGHT_MEGATAG2_Y_STDDEV_INDEX = 4;
  public static final int LIMELIGHT_MEGATAG2_YAW_STDDEV_INDEX = 5;
  private static final LoggedTunableNumber LIMELIGHT_MT1_STDDEV_X =
      new LoggedTunableNumber("AprilTagVision/Limelight/StdDev/MT1/X", 1.2);
  private static final LoggedTunableNumber LIMELIGHT_MT1_STDDEV_Y =
      new LoggedTunableNumber("AprilTagVision/Limelight/StdDev/MT1/Y", 1.2);
  private static final LoggedTunableNumber LIMELIGHT_MT1_STDDEV_YAW =
      new LoggedTunableNumber("AprilTagVision/Limelight/StdDev/MT1/YawDeg", 6.0);
  private static final LoggedTunableNumber LIMELIGHT_MT2_STDDEV_X =
      new LoggedTunableNumber("AprilTagVision/Limelight/StdDev/MT2/X", 0.7);
  private static final LoggedTunableNumber LIMELIGHT_MT2_STDDEV_Y =
      new LoggedTunableNumber("AprilTagVision/Limelight/StdDev/MT2/Y", 0.7);
  private static final LoggedTunableNumber LIMELIGHT_MT2_STDDEV_YAW =
      new LoggedTunableNumber("AprilTagVision/Limelight/StdDev/MT2/YawDeg", 4.0);
  private static final LoggedTunableNumber LIMELIGHT_MT2_STDDEV_SCALE_LL3 =
      new LoggedTunableNumber("AprilTagVision/Limelight/StdDev/MT2/ScaleLL3", 1.3);

  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_ENABLED =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/Enabled", 1.0);
  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_MAX_DIST_METERS =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/MaxDistanceMeters", 2.5);
  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_MAX_DIST_METERS_LL3 =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/MaxDistanceMetersLL3", 2.0);
  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_MAX_YAW_RATE_DEG_PER_SEC =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/MaxYawRateDegPerSec", 30.0);
  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_MAX_YAW_RATE_DEG_PER_SEC_LL3 =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/MaxYawRateDegPerSecLL3", 25.0);
  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_MAX_RESIDUAL_METERS =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/MaxResidualMeters", 0.25);
  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_MAX_YAW_RESIDUAL_DEG =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/MaxYawResidualDeg", 5.0);
  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_MAX_YAW_RESIDUAL_DEG_LL3 =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/MaxYawResidualDegLL3", 4.0);
  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_MAX_FRAME_AGE_SEC =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/MaxFrameAgeSec", 0.08);
  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_MAX_FRAME_AGE_SEC_LL3 =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/MaxFrameAgeSecLL3", 0.07);
  private static final LoggedTunableNumber LIMELIGHT_MAX_ACCEPTABLE_FRAME_AGE_SEC =
      new LoggedTunableNumber("AprilTagVision/Limelight/MaxAcceptableFrameAgeSec", 0.2);
  private static final LoggedTunableNumber LIMELIGHT_MAX_ACCEPTABLE_FRAME_AGE_SEC_LL3 =
      new LoggedTunableNumber("AprilTagVision/Limelight/MaxAcceptableFrameAgeSecLL3", 0.18);
  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_STABLE_WINDOW =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/StableWindow", 5.0);
  private static final LoggedTunableNumber LIMELIGHT_YAW_GATE_STABLE_DELTA_DEG =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/StableDeltaDeg", 2.0);
  private static final LoggedTunableNumber LIMELIGHT_YAW_STDDEV_STABLE_DEG =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/StableYawStdDevDeg", 8.0);
  private static final LoggedTunableNumber LIMELIGHT_YAW_STDDEV_UNSTABLE =
      new LoggedTunableNumber("AprilTagVision/Limelight/YawGate/UnstableYawStdDev", 1e6);

  public static final double TAG_PRESENCE_WEIGHT = 10;
  public static final double DISTANCE_WEIGHT = 7;
  public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
  public static final double POSE_AMBIGUITY_MULTIPLIER = 4;

  public static final LoggedTunableNumber LIMELIGHT_MAX_TRANSLATION_RESIDUAL_METERS =
      new LoggedTunableNumber("AprilTagVision/Limelight/MaxTranslationResidualMeters", 2.5);
  private static final LoggedTunableNumber LIMELIGHT_MAX_TRANSLATION_RESIDUAL_METERS_LL3 =
      new LoggedTunableNumber("AprilTagVision/Limelight/MaxTranslationResidualMetersLL3", 1.0);
  public static final LoggedTunableNumber LIMELIGHT_REJECT_OUTLIERS =
      new LoggedTunableNumber("AprilTagVision/Limelight/RejectOutliers", 1.0);

  public static Matrix<N3, N1> getVisionMeasurementStdDevs() {
    return MatBuilder.fill(
        Nat.N3(),
        Nat.N1(),
        VISION_STDDEV_X.get(),
        VISION_STDDEV_Y.get(),
        VISION_STDDEV_THETA.get());
  }

  public static double[] getLimelightStandardDeviations() {
    return getLimelightStandardDeviations(VisionIO.LimelightProfile.LL4);
  }

  public static double[] getLimelightStandardDeviations(VisionIO.LimelightProfile profile) {
    double mt2Scale =
        profile == VisionIO.LimelightProfile.LL3 ? LIMELIGHT_MT2_STDDEV_SCALE_LL3.get() : 1.0;
    return new double[] {
      LIMELIGHT_MT1_STDDEV_X.get(),
      LIMELIGHT_MT1_STDDEV_Y.get(),
      Math.toRadians(LIMELIGHT_MT1_STDDEV_YAW.get()),
      LIMELIGHT_MT2_STDDEV_X.get() * mt2Scale,
      LIMELIGHT_MT2_STDDEV_Y.get() * mt2Scale,
      Math.toRadians(LIMELIGHT_MT2_STDDEV_YAW.get()) * mt2Scale
    };
  }

  public static boolean ignoreMegatag2Rotation() {
    return LIMELIGHT_IGNORE_MEGATAG2_ROTATION.get() > 0.5;
  }

  public static double getMegatag2SingleTagQualityCutoff() {
    return getMegatag2SingleTagQualityCutoff(VisionIO.LimelightProfile.LL4);
  }

  public static double getMegatag2SingleTagQualityCutoff(VisionIO.LimelightProfile profile) {
    if (profile == VisionIO.LimelightProfile.LL3) {
      return MEGATAG2_SINGLE_TAG_QUALITY_CUTOFF_LL3.get();
    }
    return MEGATAG2_SINGLE_TAG_QUALITY_CUTOFF.get();
  }

  public static double getLimelightMaxYawRateDegPerSec() {
    return LIMELIGHT_MAX_YAW_RATE_DEG_PER_SEC.get();
  }

  public static double getFieldBorderMarginMeters() {
    return FIELD_BORDER_MARGIN_METERS.get();
  }

  public static double getLimelightLargeVariance() {
    return LIMELIGHT_LARGE_VARIANCE.get();
  }

  public static boolean isLimelightYawGateEnabled() {
    return LIMELIGHT_YAW_GATE_ENABLED.get() > 0.5;
  }

  public static double getLimelightYawGateMaxDistMeters() {
    return getLimelightYawGateMaxDistMeters(VisionIO.LimelightProfile.LL4);
  }

  public static double getLimelightYawGateMaxDistMeters(VisionIO.LimelightProfile profile) {
    if (profile == VisionIO.LimelightProfile.LL3) {
      return LIMELIGHT_YAW_GATE_MAX_DIST_METERS_LL3.get();
    }
    return LIMELIGHT_YAW_GATE_MAX_DIST_METERS.get();
  }

  public static double getLimelightYawGateMaxYawRateDegPerSec() {
    return getLimelightYawGateMaxYawRateDegPerSec(VisionIO.LimelightProfile.LL4);
  }

  public static double getLimelightYawGateMaxYawRateDegPerSec(VisionIO.LimelightProfile profile) {
    if (profile == VisionIO.LimelightProfile.LL3) {
      return LIMELIGHT_YAW_GATE_MAX_YAW_RATE_DEG_PER_SEC_LL3.get();
    }
    return LIMELIGHT_YAW_GATE_MAX_YAW_RATE_DEG_PER_SEC.get();
  }

  public static double getLimelightYawGateMaxResidualMeters() {
    return LIMELIGHT_YAW_GATE_MAX_RESIDUAL_METERS.get();
  }

  public static double getLimelightYawGateMaxYawResidualDeg() {
    return getLimelightYawGateMaxYawResidualDeg(VisionIO.LimelightProfile.LL4);
  }

  public static double getLimelightYawGateMaxYawResidualDeg(VisionIO.LimelightProfile profile) {
    if (profile == VisionIO.LimelightProfile.LL3) {
      return LIMELIGHT_YAW_GATE_MAX_YAW_RESIDUAL_DEG_LL3.get();
    }
    return LIMELIGHT_YAW_GATE_MAX_YAW_RESIDUAL_DEG.get();
  }

  public static double getLimelightYawGateMaxFrameAgeSec() {
    return getLimelightYawGateMaxFrameAgeSec(VisionIO.LimelightProfile.LL4);
  }

  public static double getLimelightYawGateMaxFrameAgeSec(VisionIO.LimelightProfile profile) {
    if (profile == VisionIO.LimelightProfile.LL3) {
      return LIMELIGHT_YAW_GATE_MAX_FRAME_AGE_SEC_LL3.get();
    }
    return LIMELIGHT_YAW_GATE_MAX_FRAME_AGE_SEC.get();
  }

  public static double getLimelightMaxAcceptableFrameAgeSec() {
    return getLimelightMaxAcceptableFrameAgeSec(VisionIO.LimelightProfile.LL4);
  }

  public static double getLimelightMaxAcceptableFrameAgeSec(VisionIO.LimelightProfile profile) {
    if (profile == VisionIO.LimelightProfile.LL3) {
      return LIMELIGHT_MAX_ACCEPTABLE_FRAME_AGE_SEC_LL3.get();
    }
    return LIMELIGHT_MAX_ACCEPTABLE_FRAME_AGE_SEC.get();
  }

  public static int getLimelightYawGateStableWindow() {
    return Math.max(1, (int) Math.round(LIMELIGHT_YAW_GATE_STABLE_WINDOW.get()));
  }

  public static double getLimelightYawGateStableDeltaDeg() {
    return LIMELIGHT_YAW_GATE_STABLE_DELTA_DEG.get();
  }

  public static double getLimelightYawStdDevStableRad() {
    return Math.toRadians(LIMELIGHT_YAW_STDDEV_STABLE_DEG.get());
  }

  public static double getLimelightYawStdDevUnstable() {
    return LIMELIGHT_YAW_STDDEV_UNSTABLE.get();
  }

  public static double getLimelightMaxTranslationResidualMeters(VisionIO.LimelightProfile profile) {
    if (profile == VisionIO.LimelightProfile.LL3) {
      return LIMELIGHT_MAX_TRANSLATION_RESIDUAL_METERS_LL3.get();
    }
    return LIMELIGHT_MAX_TRANSLATION_RESIDUAL_METERS.get();
  }

  public static VisionIO.LimelightProfile getLimelightProfileOverride() {
    return LimelightProfileResolver.fromOverrideValue(LIMELIGHT_PROFILE_OVERRIDE.get());
  }
}
