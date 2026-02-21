package org.Griffins1884.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.Griffins1884.frc2026.GlobalConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class AprilTagVisionProfileConstantsTest {
  @BeforeAll
  static void useDefaultTunableValues() {
    GlobalConstants.TUNING_MODE = false;
  }

  @Test
  void ll4NoArgMethods_matchExplicitLl4Profile() {
    double[] implicitLl4 = AprilTagVisionConstants.getLimelightStandardDeviations();
    double[] explicitLl4 =
        AprilTagVisionConstants.getLimelightStandardDeviations(VisionIO.LimelightProfile.LL4);

    assertArrayEquals(explicitLl4, implicitLl4, 1e-9);
    assertEquals(
        AprilTagVisionConstants.getMegatag2SingleTagQualityCutoff(),
        AprilTagVisionConstants.getMegatag2SingleTagQualityCutoff(VisionIO.LimelightProfile.LL4),
        1e-9);
    assertEquals(
        AprilTagVisionConstants.getLimelightYawGateMaxDistMeters(),
        AprilTagVisionConstants.getLimelightYawGateMaxDistMeters(VisionIO.LimelightProfile.LL4),
        1e-9);
  }

  @Test
  void ll3Thresholds_areStricterThanLl4Defaults() {
    double ll3Quality =
        AprilTagVisionConstants.getMegatag2SingleTagQualityCutoff(VisionIO.LimelightProfile.LL3);
    double ll4Quality =
        AprilTagVisionConstants.getMegatag2SingleTagQualityCutoff(VisionIO.LimelightProfile.LL4);
    assertTrue(ll3Quality > ll4Quality, "LL3 should demand stronger single-tag quality");

    assertTrue(
        AprilTagVisionConstants.getLimelightYawGateMaxDistMeters(VisionIO.LimelightProfile.LL3)
            < AprilTagVisionConstants.getLimelightYawGateMaxDistMeters(
                VisionIO.LimelightProfile.LL4));
    assertTrue(
        AprilTagVisionConstants.getLimelightYawGateMaxYawRateDegPerSec(
                VisionIO.LimelightProfile.LL3)
            < AprilTagVisionConstants.getLimelightYawGateMaxYawRateDegPerSec(
                VisionIO.LimelightProfile.LL4));
    assertTrue(
        AprilTagVisionConstants.getLimelightYawGateMaxYawResidualDeg(VisionIO.LimelightProfile.LL3)
            < AprilTagVisionConstants.getLimelightYawGateMaxYawResidualDeg(
                VisionIO.LimelightProfile.LL4));
    assertTrue(
        AprilTagVisionConstants.getLimelightYawGateMaxFrameAgeSec(VisionIO.LimelightProfile.LL3)
            < AprilTagVisionConstants.getLimelightYawGateMaxFrameAgeSec(
                VisionIO.LimelightProfile.LL4));
    assertTrue(
        AprilTagVisionConstants.getLimelightMaxTranslationResidualMeters(
                VisionIO.LimelightProfile.LL3)
            < AprilTagVisionConstants.getLimelightMaxTranslationResidualMeters(
                VisionIO.LimelightProfile.LL4));
  }

  @Test
  void ll3Megatag2Stddevs_areInflatedComparedToLl4() {
    double[] ll3 =
        AprilTagVisionConstants.getLimelightStandardDeviations(VisionIO.LimelightProfile.LL3);
    double[] ll4 =
        AprilTagVisionConstants.getLimelightStandardDeviations(VisionIO.LimelightProfile.LL4);

    assertEquals(6, ll3.length);
    assertEquals(6, ll4.length);
    assertTrue(ll3[3] > ll4[3], "LL3 MT2 X stddev should be scaled up");
    assertTrue(ll3[4] > ll4[4], "LL3 MT2 Y stddev should be scaled up");
    assertTrue(ll3[5] > ll4[5], "LL3 MT2 yaw stddev should be scaled up");
  }

  @Test
  void defaultProfileOverride_isAuto() {
    assertEquals(
        VisionIO.LimelightProfile.AUTO, AprilTagVisionConstants.getLimelightProfileOverride());
  }

  @Test
  void limelightCameraConstants_useLimelightTypeHints() {
    if (!AprilTagVisionConstants.IS_LIMELIGHT) {
      return;
    }

    assertTrue(isLimelightType(AprilTagVisionConstants.LEFT_CAM_CONSTANTS.cameraType()));
    assertTrue(isLimelightType(AprilTagVisionConstants.RIGHT_CAM_CONSTANTS.cameraType()));
    assertTrue(isLimelightType(AprilTagVisionConstants.BACK_CAM_CONSTANTS.cameraType()));
  }

  private static boolean isLimelightType(VisionIO.CameraType cameraType) {
    return switch (cameraType) {
      case LIMELIGHT, LIMELIGHT_3G, TELEPHOTO_LIMELIGHT, TELEPHOTO_LIMELIGHT_3G -> true;
      default -> false;
    };
  }
}
