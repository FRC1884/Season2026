package org.Griffins1884.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

class LimelightProfileResolverTest {
  @Test
  void fromOverrideValue_mapsExpectedValues() {
    assertEquals(VisionIO.LimelightProfile.AUTO, LimelightProfileResolver.fromOverrideValue(0.0));
    assertEquals(VisionIO.LimelightProfile.LL3, LimelightProfileResolver.fromOverrideValue(3.0));
    assertEquals(VisionIO.LimelightProfile.LL4, LimelightProfileResolver.fromOverrideValue(4.0));
    assertEquals(
        VisionIO.LimelightProfile.AUTO,
        LimelightProfileResolver.fromOverrideValue(9.0),
        "Unsupported override values should keep AUTO behavior");
  }

  @Test
  void resolve_prefersExplicitOverride() {
    VisionIO.LimelightProfile resolved =
        LimelightProfileResolver.resolve(
            VisionIO.LimelightProfile.LL3, VisionIO.CameraType.LIMELIGHT, "Limelight 4");

    assertEquals(VisionIO.LimelightProfile.LL3, resolved);
  }

  @Test
  void resolve_usesCameraIdBeforeFallback() {
    VisionIO.LimelightProfile resolved =
        LimelightProfileResolver.resolve(
            VisionIO.LimelightProfile.AUTO, VisionIO.CameraType.LIMELIGHT, "My LL3G camera");

    assertEquals(VisionIO.LimelightProfile.LL3, resolved);
  }

  @Test
  void resolve_fallsBackToCameraTypeWhenIdUnknown() {
    VisionIO.LimelightProfile resolved =
        LimelightProfileResolver.resolve(
            VisionIO.LimelightProfile.AUTO,
            VisionIO.CameraType.TELEPHOTO_LIMELIGHT_3G,
            "mystery-camera");

    assertEquals(VisionIO.LimelightProfile.LL3, resolved);
  }

  @Test
  void detectFromCameraId_matchesCommonPatterns() {
    assertEquals(
        VisionIO.LimelightProfile.LL3,
        LimelightProfileResolver.detectFromCameraId("LIMELIGHT 3G SN123"));
    assertEquals(
        VisionIO.LimelightProfile.LL3, LimelightProfileResolver.detectFromCameraId("foo-ll3-bar"));
    assertEquals(
        VisionIO.LimelightProfile.LL4, LimelightProfileResolver.detectFromCameraId("LIMELIGHT-4"));
    assertNull(LimelightProfileResolver.detectFromCameraId("unknown-cam"));
    assertNull(LimelightProfileResolver.detectFromCameraId("   "));
    assertNull(LimelightProfileResolver.detectFromCameraId(null));
  }

  @Test
  void fallbackFromCameraType_handlesKnownTypes() {
    assertEquals(
        VisionIO.LimelightProfile.LL3,
        LimelightProfileResolver.fallbackFromCameraType(VisionIO.CameraType.LIMELIGHT_3G));
    assertEquals(
        VisionIO.LimelightProfile.LL3,
        LimelightProfileResolver.fallbackFromCameraType(
            VisionIO.CameraType.TELEPHOTO_LIMELIGHT_3G));
    assertEquals(
        VisionIO.LimelightProfile.LL4,
        LimelightProfileResolver.fallbackFromCameraType(VisionIO.CameraType.LIMELIGHT));
    assertEquals(
        VisionIO.LimelightProfile.LL4, LimelightProfileResolver.fallbackFromCameraType(null));
  }
}
