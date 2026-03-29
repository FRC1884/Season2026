package org.Griffins1884.frc2026.subsystems.vision;

import static org.Griffins1884.frc2026.GlobalConstants.FieldConstants.aprilTagWidth;
import static org.Griffins1884.frc2026.GlobalConstants.FieldConstants.defaultAprilTagType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import lombok.Getter;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;

/** Hardware implementation of VisionIO using a Northstar coprocessor. */
public class AprilTagVisionIONorthstar implements VisionIO {
  private static final double DISCONNECT_TIMEOUT_SEC = 0.5;
  private static final double BASE_STD_DEV = 1.0;
  private static final double DISTANCE_SCALE = 0.15;
  private static final double TAG_BONUS = 0.20;
  private static final double SINGLE_TAG_ERROR_DOMINANCE_RATIO = 0.8;

  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final String deviceId;
  private final DoubleArraySubscriber observationSubscriber;
  private final IntegerSubscriber fpsAprilTagsSubscriber;
  private final StringPublisher cameraIdPublisher;
  private final IntegerPublisher cameraWidthPublisher;
  private final IntegerPublisher cameraHeightPublisher;
  private final IntegerPublisher cameraAutoExposurePublisher;
  private final IntegerPublisher cameraExposurePublisher;
  private final DoublePublisher cameraGainPublisher;
  private final DoublePublisher cameraDenoisePublisher;
  private final DoublePublisher fiducialSizePublisher;
  private final StringPublisher tagLayoutPublisher;
  private final StringPublisher eventNamePublisher;
  private final IntegerPublisher matchTypePublisher;
  private final IntegerPublisher matchNumberPublisher;
  private final IntegerPublisher timestampPublisher;
  private final BooleanPublisher isRecordingPublisher;
  private final SwerveSubsystem drive;
  @Getter private final CameraConstants cameraConstants;
  private final NorthstarConfig northstarConfig;
  private double lastFrameNtTimestampSec = Double.NEGATIVE_INFINITY;

  public AprilTagVisionIONorthstar(
      CameraConstants cameraConstants, NorthstarConfig northstarConfig, SwerveSubsystem drive) {
    this.cameraConstants = cameraConstants;
    this.northstarConfig = northstarConfig;
    this.drive = drive;
    this.deviceId = northstarConfig.deviceId();

    var northstarTable = ntInstance.getTable(this.deviceId);
    var configTable = northstarTable.getSubTable("config");
    cameraIdPublisher = configTable.getStringTopic("camera_id").publish();
    cameraWidthPublisher = configTable.getIntegerTopic("camera_resolution_width").publish();
    cameraHeightPublisher = configTable.getIntegerTopic("camera_resolution_height").publish();
    cameraAutoExposurePublisher = configTable.getIntegerTopic("camera_auto_exposure").publish();
    cameraExposurePublisher = configTable.getIntegerTopic("camera_exposure").publish();
    cameraGainPublisher = configTable.getDoubleTopic("camera_gain").publish();
    cameraDenoisePublisher = configTable.getDoubleTopic("camera_denoise").publish();
    fiducialSizePublisher = configTable.getDoubleTopic("fiducial_size_m").publish();
    tagLayoutPublisher = configTable.getStringTopic("tag_layout").publish();
    eventNamePublisher = configTable.getStringTopic("event_name").publish();
    matchTypePublisher = configTable.getIntegerTopic("match_type").publish();
    matchNumberPublisher = configTable.getIntegerTopic("match_number").publish();
    timestampPublisher = configTable.getIntegerTopic("timestamp").publish();
    isRecordingPublisher = configTable.getBooleanTopic("is_recording").publish();

    publishStaticConfig();

    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01));
    fpsAprilTagsSubscriber = outputTable.getIntegerTopic("fps_apriltags").subscribe(0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    resetPerCycleOutputs(inputs);
    publishRuntimeConfig();

    var aprilTagQueue = observationSubscriber.readQueue();
    if (aprilTagQueue.length > 0) {
      var latestFrame = aprilTagQueue[aprilTagQueue.length - 1];
      lastFrameNtTimestampSec = latestFrame.timestamp / 1_000_000.0;
      parseObservation(latestFrame.value, toFpgaTimestampSec(lastFrameNtTimestampSec), inputs);
    }

    boolean ntConnected = isNtConnected();
    boolean recentFrame =
        Double.isFinite(lastFrameNtTimestampSec)
            && ((WPIUtilJNI.getSystemTime() / 1_000_000.0) - lastFrameNtTimestampSec)
                < DISCONNECT_TIMEOUT_SEC;
    inputs.connected = ntConnected && (recentFrame || fpsAprilTagsSubscriber.get() > 0);
    if (!inputs.connected) {
      inputs.rejectReason = RejectReason.DISCONNECTED;
    } else if (!inputs.seesTarget) {
      inputs.rejectReason = RejectReason.NO_TAGS;
    } else if (inputs.megatagPoseEstimate == null) {
      inputs.rejectReason = RejectReason.NO_MEGATAG;
    } else {
      inputs.rejectReason = RejectReason.ACCEPTED;
    }
  }

  private double toFpgaTimestampSec(double ntTimestampSec) {
    double nowSystemSec = WPIUtilJNI.getSystemTime() / 1_000_000.0;
    double frameAgeSec = Math.max(0.0, nowSystemSec - ntTimestampSec);
    return Math.max(0.0, Timer.getFPGATimestamp() - frameAgeSec);
  }

  private void publishStaticConfig() {
    cameraIdPublisher.set(northstarConfig.cameraId());
    cameraWidthPublisher.set(northstarConfig.width());
    cameraHeightPublisher.set(northstarConfig.height());
    cameraAutoExposurePublisher.set(northstarConfig.autoExposure());
    cameraExposurePublisher.set(northstarConfig.exposure());
    cameraGainPublisher.set(northstarConfig.gain());
    cameraDenoisePublisher.set(northstarConfig.denoise());
    fiducialSizePublisher.set(aprilTagWidth);
    tagLayoutPublisher.set(defaultAprilTagType.getLayoutString());
    isRecordingPublisher.set(false);
  }

  private void publishRuntimeConfig() {
    timestampPublisher.set(WPIUtilJNI.getSystemTime() / 1_000_000);
    eventNamePublisher.set(DriverStation.getEventName());
    matchTypePublisher.set(DriverStation.getMatchType().ordinal());
    matchNumberPublisher.set(DriverStation.getMatchNumber());
  }

  private boolean isNtConnected() {
    for (var connection : ntInstance.getConnections()) {
      if (connection.remote_id.startsWith(deviceId)) {
        return true;
      }
    }
    return false;
  }

  private void resetPerCycleOutputs(VisionIOInputs inputs) {
    inputs.connected = false;
    inputs.seesTarget = false;
    inputs.megatagCount = 0;
    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    inputs.pose3d = null;
    inputs.megatagPoseEstimate = null;
    inputs.fiducialObservations = new FiducialObservation[0];
    inputs.standardDeviations = AprilTagVisionConstants.getLimelightStandardDeviations();
    inputs.poseObservations = new PoseObservation[0];
    inputs.tagIds = new int[0];
    inputs.residualTranslationMeters = Double.NaN;
    inputs.rejectReason = RejectReason.UNKNOWN;
    inputs.limelightProfile = "NORTHSTAR";
    inputs.limelightProfileSource = "NORTHSTAR";
  }

  private void parseObservation(double[] frame, double timestampSec, VisionIOInputs inputs) {
    if (frame == null || frame.length == 0) {
      return;
    }

    int poseCount = (int) Math.round(frame[0]);
    inputs.seesTarget = poseCount > 0;
    if (poseCount <= 0) {
      return;
    }

    int tagStartIndex =
        switch (poseCount) {
          case 1 -> 9;
          case 2 -> 17;
          default -> -1;
        };
    if (tagStartIndex < 0 || frame.length < tagStartIndex) {
      return;
    }

    ParsedPose pose0 = parsePose(frame, 1);
    ParsedPose pose1 = poseCount == 2 && frame.length >= 17 ? parsePose(frame, 9) : null;
    List<NorthstarTagObservation> tagObservations = parseTagObservations(frame, tagStartIndex);
    if (tagObservations.isEmpty()) {
      return;
    }

    PoseSelection selection = selectPose(pose0, pose1);
    if (selection == null || selection.robotPose() == null) {
      return;
    }

    inputs.tagIds =
        tagObservations.stream().mapToInt(NorthstarTagObservation::tagId).distinct().toArray();
    inputs.megatagCount = inputs.tagIds.length;
    inputs.latestTargetObservation =
        tagObservations.stream()
            .min(Comparator.comparingDouble(NorthstarTagObservation::distanceMeters))
            .map(NorthstarTagObservation::targetObservation)
            .orElse(new TargetObservation(new Rotation2d(), new Rotation2d()));
    inputs.fiducialObservations =
        tagObservations.stream()
            .map(
                observation ->
                    new FiducialObservation(
                        observation.tagId(),
                        observation.targetObservation().tx().getRadians(),
                        observation.targetObservation().ty().getRadians(),
                        selection.ambiguity(),
                        Double.NaN))
            .toArray(FiducialObservation[]::new);

    double avgTagDist =
        tagObservations.stream()
            .mapToDouble(NorthstarTagObservation::distanceMeters)
            .average()
            .orElse(Double.NaN);
    double quality =
        selection.ambiguity() >= 0.0 && Double.isFinite(selection.ambiguity())
            ? Math.max(0.0, Math.min(1.0, 1.0 - selection.ambiguity()))
            : 1.0;
    inputs.pose3d = selection.robotPose();
    Pose2d fieldToRobot2d = selection.robotPose().toPose2d();
    inputs.megatagPoseEstimate =
        new MegatagPoseEstimate(
            fieldToRobot2d,
            timestampSec,
            0.0,
            Double.NaN,
            avgTagDist,
            quality,
            Arrays.copyOf(inputs.tagIds, inputs.tagIds.length),
            Double.NaN);
    inputs.standardDeviations = buildDynamicStandardDeviations(avgTagDist, inputs.megatagCount);
    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              timestampSec,
              selection.robotPose(),
              selection.ambiguity(),
              inputs.megatagCount,
              avgTagDist)
        };
  }

  private ParsedPose parsePose(double[] frame, int baseIndex) {
    if (frame.length < baseIndex + 8) {
      return null;
    }
    return new ParsedPose(
        frame[baseIndex],
        new Pose3d(
            frame[baseIndex + 1],
            frame[baseIndex + 2],
            frame[baseIndex + 3],
            new Rotation3d(
                new Quaternion(
                    frame[baseIndex + 4],
                    frame[baseIndex + 5],
                    frame[baseIndex + 6],
                    frame[baseIndex + 7]))));
  }

  private List<NorthstarTagObservation> parseTagObservations(double[] frame, int tagStartIndex) {
    List<NorthstarTagObservation> observations = new ArrayList<>();
    for (int index = tagStartIndex; index + 9 < frame.length; index += 10) {
      int tagId = (int) Math.round(frame[index]);
      double[] txCorners = {frame[index + 1], frame[index + 3], frame[index + 5], frame[index + 7]};
      double[] tyCorners = {frame[index + 2], frame[index + 4], frame[index + 6], frame[index + 8]};
      double tx = Arrays.stream(txCorners).average().orElse(0.0);
      double ty = Arrays.stream(tyCorners).average().orElse(0.0);
      observations.add(
          new NorthstarTagObservation(
              tagId,
              frame[index + 9],
              new TargetObservation(new Rotation2d(tx), new Rotation2d(ty))));
    }
    return observations;
  }

  private PoseSelection selectPose(ParsedPose pose0, ParsedPose pose1) {
    if (pose0 == null) {
      return null;
    }

    Transform3d cameraToRobot = cameraConstants.robotToCamera().inverse();
    Pose3d robotPose0 = pose0.pose().transformBy(cameraToRobot);
    if (pose1 == null) {
      return new PoseSelection(robotPose0, 0.0);
    }

    Pose3d robotPose1 = pose1.pose().transformBy(cameraToRobot);
    double error0 = sanitizedError(pose0.error());
    double error1 = sanitizedError(pose1.error());
    double ambiguity = computeAmbiguity(error0, error1);

    if (error0 < error1 * SINGLE_TAG_ERROR_DOMINANCE_RATIO) {
      return new PoseSelection(robotPose0, ambiguity);
    }
    if (error1 < error0 * SINGLE_TAG_ERROR_DOMINANCE_RATIO) {
      return new PoseSelection(robotPose1, ambiguity);
    }

    Rotation2d currentHeading = drive.getPose().getRotation();
    double headingError0 =
        Math.abs(currentHeading.minus(robotPose0.toPose2d().getRotation()).getRadians());
    double headingError1 =
        Math.abs(currentHeading.minus(robotPose1.toPose2d().getRotation()).getRadians());
    return headingError0 <= headingError1
        ? new PoseSelection(robotPose0, ambiguity)
        : new PoseSelection(robotPose1, ambiguity);
  }

  private double[] buildDynamicStandardDeviations(double avgTagDistanceMeters, int tagCount) {
    double[] stdDevs = AprilTagVisionConstants.getLimelightStandardDeviations();
    if (!Double.isFinite(avgTagDistanceMeters) || tagCount <= 0) {
      return stdDevs;
    }

    double trust = BASE_STD_DEV;
    trust *= (1.0 + avgTagDistanceMeters * DISTANCE_SCALE);
    trust /= (1.0 + tagCount * TAG_BONUS);
    if (!Double.isFinite(trust) || trust <= 0.0) {
      return stdDevs;
    }

    stdDevs[AprilTagVisionConstants.LIMELIGHT_MEGATAG2_X_STDDEV_INDEX] *= trust;
    stdDevs[AprilTagVisionConstants.LIMELIGHT_MEGATAG2_Y_STDDEV_INDEX] *= trust;
    stdDevs[AprilTagVisionConstants.LIMELIGHT_MEGATAG2_YAW_STDDEV_INDEX] *= trust;
    return stdDevs;
  }

  private static double sanitizedError(double error) {
    return Double.isFinite(error) && error >= 0.0 ? error : Double.POSITIVE_INFINITY;
  }

  private static double computeAmbiguity(double error0, double error1) {
    if (!Double.isFinite(error0) || !Double.isFinite(error1) || error0 + error1 <= 0.0) {
      return 1.0;
    }
    return Math.max(0.0, Math.min(1.0, Math.min(error0, error1) / (error0 + error1)));
  }

  private record ParsedPose(double error, Pose3d pose) {}

  private record PoseSelection(Pose3d robotPose, double ambiguity) {}

  private record NorthstarTagObservation(
      int tagId, double distanceMeters, TargetObservation targetObservation) {}
}
