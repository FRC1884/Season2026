package org.Griffins1884.frc2026.subsystems.vision;

import static edu.wpi.first.math.util.Units.radiansToDegrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Arrays;
import java.util.Objects;
import java.util.Optional;
import lombok.Getter;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.util.RobotLogging;
import org.littletonrobotics.junction.Logger;

/** Hardware implementation of VisionIO using Limelight cameras. */
public class AprilTagVisionIOLimelight implements VisionIO {
  // Timeout window for considering the Limelight disconnected, in FPGA microseconds.
  private static final double DISCONNECT_TIMEOUT_MICROS = 250_000.0;

  // MegaTag2 covariance scaling constants.
  private static final double BASE_STD_DEV = 1.0;
  private static final double DISTANCE_SCALE = 0.15;
  private static final double TAG_BONUS = 0.20;

  private final NetworkTable table;

  private final String limelightName;
  private final SwerveSubsystem drive;
  @Getter private final CameraConstants cameraConstants;
  private int imuMode = -1;

  /** Creates a new Limelight vision IO instance. */
  public AprilTagVisionIOLimelight(CameraConstants cameraConstants, SwerveSubsystem drive) {
    this.cameraConstants = cameraConstants;
    this.drive = drive;
    this.limelightName = cameraConstants.cameraName();
    this.table = NetworkTableInstance.getDefault().getTable(this.limelightName);
    setLLSettings();
  }

  /** Configures Limelight camera poses in robot coordinate system. */
  private void setLLSettings() {
    LimelightHelpers.setCameraPose_RobotSpace(
        limelightName,
        cameraConstants.robotToCamera().getX(),
        cameraConstants.robotToCamera().getY(),
        cameraConstants.robotToCamera().getZ(),
        radiansToDegrees(cameraConstants.robotToCamera().getRotation().getX()),
        radiansToDegrees(cameraConstants.robotToCamera().getRotation().getY()),
        radiansToDegrees(cameraConstants.robotToCamera().getRotation().getZ()));
  }

  private void applyImuMode(int desiredMode) {
    if (imuMode == desiredMode) {
      return;
    }
    LimelightHelpers.SetIMUMode(limelightName, desiredMode);
    imuMode = desiredMode;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    resetPerCycleOutputs(inputs);

    int desiredImuMode = DriverStation.isDisabled() ? 1 : 4;
    applyImuMode(desiredImuMode);

    double gyroYawDeg = drive.getRawestGyroRotation().getDegrees();
    double gyroYawRateDegPerSec = drive.getYawRateDegreesPerSec();

    // MegaTag2 requires current robot orientation to be pushed every loop before reading the
    // estimate.
    LimelightHelpers.SetRobotOrientation(
        limelightName, gyroYawDeg, 0.0, 0.0, gyroYawRateDegPerSec, 0.0, 0.0);

    long lastChange = table.getEntry("tl").getLastChange();
    long now = RobotController.getFPGATime();
    inputs.connected = lastChange > 0 && (now - lastChange) < DISCONNECT_TIMEOUT_MICROS;
    inputs.seesTarget = LimelightHelpers.getTV(limelightName);
    if (inputs.seesTarget) {
      inputs.latestTargetObservation =
          new TargetObservation(
              Rotation2d.fromDegrees(LimelightHelpers.getTX(limelightName)),
              Rotation2d.fromDegrees(LimelightHelpers.getTY(limelightName)));
    }

    inputs.standardDeviations = AprilTagVisionConstants.getLimelightStandardDeviations();
    if (!inputs.connected) {
      inputs.rejectReason = RejectReason.DISCONNECTED;
      return;
    }

    try {
      LimelightHelpers.PoseEstimate mt2 = getMegaTag2Estimate();
      if (mt2 != null) {
        inputs.megatagCount = mt2.tagCount;
        inputs.fiducialObservations = FiducialObservation.fromLimelight(mt2.rawFiducials);
        inputs.tagIds = extractTagIds(mt2.rawFiducials);
      }

      ValidationResult validation = validate(mt2, gyroYawRateDegPerSec);
      inputs.rejectReason = toRejectReason(validation);
      logValidation(validation, gyroYawRateDegPerSec, mt2);
      if (!validation.accepted()) {
        return;
      }

      inputs.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(mt2);
      inputs.pose3d = new Pose3d(mt2.pose);
      inputs.residualTranslationMeters = mt2.residualTranslation;
      inputs.standardDeviations = buildDynamicStandardDeviations(mt2);

      double ambiguity = calculateAverageAmbiguity(mt2.rawFiducials);
      inputs.poseObservations =
          new PoseObservation[] {
            new PoseObservation(
                mt2.timestampSeconds, new Pose3d(mt2.pose), ambiguity, mt2.tagCount, mt2.avgTagDist)
          };
    } catch (Exception e) {
      RobotLogging.error("Error processing Limelight MegaTag2 data", e);
    }
  }

  private void resetPerCycleOutputs(VisionIOInputs inputs) {
    inputs.megatagPoseEstimate = null;
    inputs.pose3d = null;
    inputs.fiducialObservations = new FiducialObservation[0];
    inputs.poseObservations = new PoseObservation[0];
    inputs.tagIds = new int[0];
    inputs.megatagCount = 0;
    inputs.residualTranslationMeters = 0.0;
    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    inputs.rejectReason = RejectReason.UNKNOWN;
  }

  private LimelightHelpers.PoseEstimate getMegaTag2Estimate() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
    }
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
  }

  private ValidationResult validate(LimelightHelpers.PoseEstimate mt2, double gyroRateDegPerSec) {
    if (mt2 == null) {
      return ValidationResult.NULL_ESTIMATE;
    }
    if (mt2.pose == null) {
      return ValidationResult.NULL_POSE;
    }
    if (!Double.isFinite(mt2.timestampSeconds)) {
      return ValidationResult.INVALID_TIMESTAMP;
    }
    if (mt2.tagCount <= 0) {
      return ValidationResult.NO_TAGS;
    }
    if (Math.abs(gyroRateDegPerSec) > getMaxAngularVelocityDegPerSec()) {
      return ValidationResult.HIGH_ANGULAR_VELOCITY;
    }
    if (!Double.isFinite(mt2.avgTagDist) || mt2.avgTagDist > getMaxDistanceMeters()) {
      return ValidationResult.TOO_FAR;
    }
    return ValidationResult.ACCEPTED;
  }

  private double[] buildDynamicStandardDeviations(LimelightHelpers.PoseEstimate mt2) {
    double[] stdDevs = AprilTagVisionConstants.getLimelightStandardDeviations();

    double trust = BASE_STD_DEV;
    trust *= (1.0 + mt2.avgTagDist * DISTANCE_SCALE);
    trust /= (1.0 + mt2.tagCount * TAG_BONUS);

    if (!Double.isFinite(trust) || trust <= 0.0) {
      return stdDevs;
    }

    stdDevs[AprilTagVisionConstants.LIMELIGHT_MEGATAG2_X_STDDEV_INDEX] *= trust;
    stdDevs[AprilTagVisionConstants.LIMELIGHT_MEGATAG2_Y_STDDEV_INDEX] *= trust;
    stdDevs[AprilTagVisionConstants.LIMELIGHT_MEGATAG2_YAW_STDDEV_INDEX] *= trust;
    return stdDevs;
  }

  private double getMaxAngularVelocityDegPerSec() {
    return Math.min(
        AprilTagVisionConstants.getLimelightMaxYawRateDegPerSec(),
        cameraConstants.cameraType().noisySpeed);
  }

  private double getMaxDistanceMeters() {
    return cameraConstants.cameraType().noisyDistance;
  }

  private void logValidation(
      ValidationResult validation, double gyroRateDegPerSec, LimelightHelpers.PoseEstimate mt2) {
    String prefix = "AprilTagVision/" + limelightName + "/IOValidation";
    Logger.recordOutput(prefix + "/Accepted", validation.accepted());
    Logger.recordOutput(prefix + "/Reason", validation.name());
    Logger.recordOutput(prefix + "/GyroRateDegPerSec", gyroRateDegPerSec);
    Logger.recordOutput(prefix + "/MaxAngularVelocityDegPerSec", getMaxAngularVelocityDegPerSec());
    Logger.recordOutput(prefix + "/MaxDistanceMeters", getMaxDistanceMeters());
    Logger.recordOutput(prefix + "/TagCount", mt2 != null ? mt2.tagCount : 0);
    Logger.recordOutput(prefix + "/AvgTagDist", mt2 != null ? mt2.avgTagDist : Double.NaN);
  }

  private static RejectReason toRejectReason(ValidationResult validation) {
    return switch (validation) {
      case ACCEPTED -> RejectReason.ACCEPTED;
      case NULL_ESTIMATE, NULL_POSE, INVALID_TIMESTAMP -> RejectReason.NO_MEGATAG;
      case NO_TAGS -> RejectReason.NO_TAGS;
      case HIGH_ANGULAR_VELOCITY, TOO_FAR -> RejectReason.UNKNOWN;
    };
  }

  private static int[] extractTagIds(LimelightHelpers.RawFiducial[] rawFiducials) {
    if (rawFiducials == null || rawFiducials.length == 0) {
      return new int[0];
    }
    return Arrays.stream(rawFiducials)
        .filter(Objects::nonNull)
        .mapToInt(fiducial -> fiducial.id)
        .distinct()
        .sorted()
        .toArray();
  }

  // Compute the mean ambiguity across all detected fiducials; default to 1.0 when none exist.
  private static double calculateAverageAmbiguity(LimelightHelpers.RawFiducial[] fiducials) {
    if (fiducials == null || fiducials.length == 0) {
      return 1.0;
    }
    double totalAmbiguity = 0.0;
    int count = 0;
    for (LimelightHelpers.RawFiducial fiducial : fiducials) {
      if (fiducial == null) {
        continue;
      }
      double ambiguity = fiducial.ambiguity;
      if (!Double.isFinite(ambiguity)) {
        continue;
      }
      totalAmbiguity += ambiguity;
      count++;
    }
    return count == 0 ? 1.0 : totalAmbiguity / count;
  }

  private enum ValidationResult {
    ACCEPTED,
    NULL_ESTIMATE,
    NULL_POSE,
    INVALID_TIMESTAMP,
    NO_TAGS,
    HIGH_ANGULAR_VELOCITY,
    TOO_FAR;

    private boolean accepted() {
      return this == ACCEPTED;
    }
  }
}
