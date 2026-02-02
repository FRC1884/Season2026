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
import java.util.function.DoubleSupplier;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

// TODO tune all of these!! Make sure that the robot-relative coords are correct - this will cause
// rapid pose oscillation
public final class AprilTagVisionConstants {
  public static final boolean IS_LIMELIGHT = true;
  public static final boolean usingMegaTag = true;
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
                VisionIO.CameraType.OV9281);
        case COMPBOT, SIMBOT, CRESCENDO ->
            new VisionIO.CameraConstants(
                (IS_LIMELIGHT) ? "limelight-left" : "lefttagcam",
                new Transform3d(
                    0.3006,
                    0.3056,
                    0.245,
                    new Rotation3d(0, degreesToRadians(-5), degreesToRadians(-20))),
                VisionIO.CameraType.OV9281);
      };

  public static final boolean RIGHT_CAM_ENABLED = false;
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
                VisionIO.CameraType.OV9281);
        case COMPBOT, SIMBOT, CRESCENDO ->
            new VisionIO.CameraConstants(
                (IS_LIMELIGHT) ? "limelight-right" : "righttagcam",
                new Transform3d(
                    0.3006,
                    -0.3056,
                    0.245,
                    new Rotation3d(0, degreesToRadians(-5), degreesToRadians(20))),
                VisionIO.CameraType.OV9281);
      };

  public static final boolean BACK_CAM_ENABLED = true;
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
                VisionIO.CameraType.OV9281);
        case COMPBOT, SIMBOT, CRESCENDO ->
            new VisionIO.CameraConstants(
                "backtagcam",
                new Transform3d(
                    -0.3006,
                    0.3056,
                    0.245,
                    new Rotation3d(0, degreesToRadians(-20), degreesToRadians(180))),
                VisionIO.CameraType.OV9281);
      };

  public static final DoubleSupplier TRANSLATION_EULER_MULTIPLIER =
      new LoggedTunableNumber("AprilTagVision/EulerMultipliers/Translation", 0.02);
  public static final DoubleSupplier ROTATION_EULER_MULTIPLIER =
      new LoggedTunableNumber("AprilTagVision/EulerMultipliers/Rotation", 0.06);

  public static final double MAX_AMBIGUITY_CUTOFF = 0.3;
  public static final double MAX_Z_ERROR = 0.75;

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
  private static final LoggedTunableNumber MEGATAG2_SINGLE_TAG_QUALITY_CUTOFF =
      new LoggedTunableNumber("AprilTagVision/Limelight/SingleTagQualityCutoff", 0.3);
  private static final LoggedTunableNumber FIELD_BORDER_MARGIN_METERS =
      new LoggedTunableNumber("AprilTagVision/FieldBorderMarginMeters", 0.5);
  public static final int LIMELIGHT_MEGATAG1_X_STDDEV_INDEX = 0;
  public static final int LIMELIGHT_MEGATAG1_Y_STDDEV_INDEX = 1;
  public static final int LIMELIGHT_MEGATAG1_YAW_STDDEV_INDEX = 2;
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

  public static final double TAG_PRESENCE_WEIGHT = 10;
  public static final double DISTANCE_WEIGHT = 7;
  public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
  public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
  public static Matrix<N3, N1> getVisionMeasurementStdDevs() {
    return MatBuilder.fill(
        Nat.N3(), Nat.N1(), VISION_STDDEV_X.get(), VISION_STDDEV_Y.get(), VISION_STDDEV_THETA.get());
  }

  public static double[] getLimelightStandardDeviations() {
    return new double[] {
      LIMELIGHT_MT1_STDDEV_X.get(),
      LIMELIGHT_MT1_STDDEV_Y.get(),
      Math.toRadians(LIMELIGHT_MT1_STDDEV_YAW.get()),
      LIMELIGHT_MT2_STDDEV_X.get(),
      LIMELIGHT_MT2_STDDEV_Y.get(),
      Math.toRadians(LIMELIGHT_MT2_STDDEV_YAW.get())
    };
  }

  public static boolean ignoreMegatag2Rotation() {
    return LIMELIGHT_IGNORE_MEGATAG2_ROTATION.get() > 0.5;
  }

  public static double getMegatag2SingleTagQualityCutoff() {
    return MEGATAG2_SINGLE_TAG_QUALITY_CUTOFF.get();
  }

  public static double getFieldBorderMarginMeters() {
    return FIELD_BORDER_MARGIN_METERS.get();
  }

  public static double getLimelightLargeVariance() {
    return LIMELIGHT_LARGE_VARIANCE.get();
  }
}
