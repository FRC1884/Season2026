package org.Griffins1884.frc2026.subsystems.vision;

import java.util.Locale;

/** Resolves which Limelight profile should be used for a camera. */
public final class LimelightProfileResolver {
  private LimelightProfileResolver() {}

  public static VisionIO.LimelightProfile fromOverrideValue(double value) {
    long rounded = Math.round(value);
    if (rounded == 3L) {
      return VisionIO.LimelightProfile.LL3;
    }
    if (rounded == 4L) {
      return VisionIO.LimelightProfile.LL4;
    }
    return VisionIO.LimelightProfile.AUTO;
  }

  public static VisionIO.LimelightProfile resolve(
      VisionIO.LimelightProfile override, VisionIO.CameraType typeHint, String cameraId) {
    if (override != null && override != VisionIO.LimelightProfile.AUTO) {
      return override;
    }
    VisionIO.LimelightProfile detected = detectFromCameraId(cameraId);
    if (detected != null) {
      return detected;
    }
    return fallbackFromCameraType(typeHint);
  }

  static VisionIO.LimelightProfile detectFromCameraId(String cameraId) {
    if (cameraId == null) {
      return null;
    }
    String normalized = cameraId.trim().toUpperCase(Locale.ROOT);
    if (normalized.isEmpty()) {
      return null;
    }
    if (normalized.contains("LL3")
        || normalized.contains("3G")
        || normalized.contains("LIMELIGHT 3")
        || normalized.contains("LIMELIGHT-3")) {
      return VisionIO.LimelightProfile.LL3;
    }
    if (normalized.contains("LL4")
        || normalized.contains("LIMELIGHT 4")
        || normalized.contains("LIMELIGHT-4")) {
      return VisionIO.LimelightProfile.LL4;
    }
    return null;
  }

  static VisionIO.LimelightProfile fallbackFromCameraType(VisionIO.CameraType cameraType) {
    if (cameraType == null) {
      return VisionIO.LimelightProfile.LL4;
    }
    return switch (cameraType) {
      case LIMELIGHT_3G, TELEPHOTO_LIMELIGHT_3G -> VisionIO.LimelightProfile.LL3;
      default -> VisionIO.LimelightProfile.LL4;
    };
  }
}
