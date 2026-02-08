package org.Griffins1884.frc2026.subsystems.vision;

import static org.Griffins1884.frc2026.GlobalConstants.FieldConstants.defaultAprilTagType;
import static org.Griffins1884.frc2026.subsystems.vision.AprilTagVisionHelpers.generateDynamicStdDevs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Setter;
import org.Griffins1884.frc2026.GlobalConstants;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase implements VisionTargetProvider {
  private static final double HISTORY_WINDOW_SEC = 1.5;

  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final boolean useLimelightFusion;
  private final PoseHistory poseHistory;
  @Setter private boolean useVision = true;
  private final DoubleSupplier yawRateRadPerSecSupplier;

  /**
   * Creates a Vision system for PhotonVision inputs.
   *
   * @param consumer an object that processes the vision pose estimate (should be the drivetrain)
   * @param io the collection of {@link VisionIO}s instances that represent the cameras in the
   *     system.
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this(consumer, null, null, io);
  }

  /**
   * Creates a Vision system for Limelight inputs using pose-fusion.
   *
   * @param consumer an object that processes the vision pose estimate (should be the drivetrain)
   * @param poseSupplier robot pose supplier for pose-history alignment
   * @param yawRateRadPerSecSupplier yaw-rate supplier for alignment gating
   * @param io the collection of {@link VisionIO}s instances that represent the cameras in the
   *     system.
   */
  public Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier yawRateRadPerSecSupplier,
      VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.useLimelightFusion = poseSupplier != null && yawRateRadPerSecSupplier != null;
    this.poseHistory =
        useLimelightFusion
            ? new PoseHistory(HISTORY_WINDOW_SEC, poseSupplier, yawRateRadPerSecSupplier)
            : null;
    this.yawRateRadPerSecSupplier = yawRateRadPerSecSupplier;

    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera \"" + io[i].getCameraConstants().cameraName() + "\" is disconnected.",
              Alert.AlertType.kWarning);
    }
  }

  /**
   * Returns the yaw (horizontal angle) to the best detected AprilTag if available. If no tags are
   * detected, an empty {@link Optional} is returned.
   *
   * @param cameraIndex The index of the camera to retrieve yaw data from.
   * @return An {@link Optional} containing the yaw as a {@link Rotation2d}, or empty if no tag is
   *     detected.
   */
  public Optional<Rotation2d> getTargetX(int cameraIndex) {
    return inputs[cameraIndex].tagIds.length == 0
        ? Optional.empty()
        : Optional.of(inputs[cameraIndex].latestTargetObservation.tx());
  }

  /**
   * Returns the pitch (vertical angle) to the best detected AprilTag if available. If no tags are
   * detected, an empty {@link Optional} is returned.
   *
   * @param cameraIndex The index of the camera to retrieve pitch data from.
   * @return An {@link Optional} containing the pitch as a {@link Rotation2d}, or empty if no tag is
   *     detected.
   */
  public Optional<Rotation2d> getTargetY(int cameraIndex) {
    return inputs[cameraIndex].tagIds.length == 0
        ? Optional.empty()
        : Optional.of(inputs[cameraIndex].latestTargetObservation.ty());
  }

  /**
   * Returns the field translation of the closest visible AprilTag. This is intended for
   * field-relative targeting, using the robot pose estimate for distance selection.
   */
  @Override
  public Optional<Translation2d> getBestTargetTranslation(Pose2d robotPose) {
    Translation2d bestTranslation = null;
    double bestDistance = Double.POSITIVE_INFINITY;

    for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {
      if (!inputs[cameraIndex].connected) {
        continue;
      }
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = defaultAprilTagType.getLayout().getTagPose(tagId);
        if (tagPose.isEmpty()) {
          continue;
        }
        Translation2d translation = tagPose.get().toPose2d().getTranslation();
        double distance = translation.getDistance(robotPose.getTranslation());
        if (distance < bestDistance) {
          bestDistance = distance;
          bestTranslation = translation;
        }
      }
    }

    return Optional.ofNullable(bestTranslation);
  }

  /**
   * Updates vision inputs and logs relevant pose data. Uses either PhotonVision observations or
   * Limelight Megatag fusion depending on construction.
   */
  @Override
  public void periodic() {
    if (useLimelightFusion) {
      periodicLimelight();
    } else {
      periodicPhotonVision();
    }
  }

  private void periodicPhotonVision() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("AprilTagVision/" + io[i].getCameraConstants().cameraName(), inputs[i]);
    }

    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();

      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = defaultAprilTagType.getLayout().getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      for (var observation : inputs[cameraIndex].poseObservations) {
        if (observation == null || observation.pose() == null) {
          continue;
        }
        robotPoses.add(observation.pose());
        if (!isObservationValid(observation)) {
          continue;
        }

        robotPosesAccepted.add(observation.pose());

        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            generateDynamicStdDevs(observation, io[cameraIndex].getCameraConstants().cameraType()));
      }

      Logger.recordOutput(
          "AprilTagVision/" + io[cameraIndex].getCameraConstants().cameraName() + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "AprilTagVision/" + io[cameraIndex].getCameraConstants().cameraName() + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "AprilTagVision/"
              + io[cameraIndex].getCameraConstants().cameraName()
              + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "AprilTagVision/" + io[cameraIndex].getCameraConstants().cameraName() + "/CameraPose",
          io[cameraIndex].getCameraConstants().robotToCamera());
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
    }

    Logger.recordOutput(
        "AprilTagVision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "AprilTagVision/Summary/RobotPoses",
        allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "AprilTagVision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
  }

  private boolean isObservationValid(VisionIO.PoseObservation observation) {
    if (observation.tagCount() <= 0) {
      return false;
    }
    if (!isFinite(observation.timestamp()) || !isFinite(observation.averageTagDistance())) {
      return false;
    }
    Pose3d pose = observation.pose();
    if (!isFinite(pose.getX())
        || !isFinite(pose.getY())
        || !isFinite(pose.getZ())
        || !isFinite(pose.getRotation().getX())
        || !isFinite(pose.getRotation().getY())
        || !isFinite(pose.getRotation().getZ())) {
      return false;
    }
    Rotation2d rotation = pose.toPose2d().getRotation();
    double cos = rotation.getCos();
    double sin = rotation.getSin();
    return isFinite(cos) && isFinite(sin) && !(Math.abs(cos) < 1e-9 && Math.abs(sin) < 1e-9);
  }

  private boolean isFinite(double value) {
    return Double.isFinite(value);
  }

  private void periodicLimelight() {
    double startTime = Timer.getFPGATimestamp();
    if (poseHistory != null) {
      poseHistory.update(startTime);
    }

    double yawRateDegPerSec =
        yawRateRadPerSecSupplier != null
            ? Math.toDegrees(yawRateRadPerSecSupplier.getAsDouble())
            : 0.0;
    boolean yawRateOk =
        DriverStation.isDisabled()
            || Math.abs(yawRateDegPerSec)
                <= AprilTagVisionConstants.getLimelightMaxYawRateDegPerSec();
    Logger.recordOutput("Vision/yawRateDegPerSec", yawRateDegPerSec);
    Logger.recordOutput("Vision/yawRateAccept", yawRateOk);

    List<Optional<VisionFieldPoseEstimate>> estimates = new ArrayList<>();

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("LimelightVision/" + io[i].getCameraConstants().cameraName(), inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);

      String cameraLabel = io[i].getCameraConstants().cameraName();
      logCameraInputs("Vision/" + cameraLabel, inputs[i]);
      logLimelightDiagnostics(cameraLabel, inputs[i]);
      estimates.add(buildLimelightEstimate(inputs[i]));
    }

    if (!useVision) {
      Logger.recordOutput("Vision/usingVision", false);
      Logger.recordOutput("Vision/rejectReason", "VISION_DISABLED");
      Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
      return;
    }

    if (!yawRateOk) {
      Logger.recordOutput("Vision/usingVision", true);
      Logger.recordOutput("Vision/rejectReason", "HIGH_YAW_RATE");
      Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
      return;
    }

    Logger.recordOutput("Vision/usingVision", true);
    Logger.recordOutput("Vision/rejectReason", "ACCEPTED");

    Optional<VisionFieldPoseEstimate> accepted = Optional.empty();
    for (Optional<VisionFieldPoseEstimate> estimate : estimates) {
      if (estimate.isEmpty()) {
        continue;
      }
      if (accepted.isEmpty()) {
        accepted = estimate;
      } else {
        accepted = Optional.of(fuseEstimates(accepted.get(), estimate.get()));
      }
    }

    accepted.ifPresent(
        est -> {
          Logger.recordOutput("Vision/fusedAccepted", est.visionRobotPoseMeters());
          consumer.accept(
              est.visionRobotPoseMeters(), est.timestampSeconds(), est.visionMeasurementStdDevs());
        });

    Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
  }

  private Optional<VisionFieldPoseEstimate> buildLimelightEstimate(VisionIO.VisionIOInputs cam) {
    if (!cam.connected || cam.megatagPoseEstimate == null) {
      return Optional.empty();
    }

    Pose2d fieldToRobot = cam.megatagPoseEstimate.fieldToRobot();
    if (!isFinitePose(fieldToRobot) || !isWithinFieldBounds(fieldToRobot)) {
      return Optional.empty();
    }

    int tagCount = cam.megatagPoseEstimate.fiducialIds().length;
    if (tagCount <= 0) {
      return Optional.empty();
    }
    int indexBase = AprilTagVisionConstants.LIMELIGHT_MEGATAG2_X_STDDEV_INDEX;

    double qualityUsed = sanitizeQuality(cam.megatagPoseEstimate.quality());
    if (tagCount == 1
        && qualityUsed < AprilTagVisionConstants.getMegatag2SingleTagQualityCutoff()) {
      return Optional.empty();
    }
    LimelightStdDevs stdDevs = computeLimelightStdDevs(cam, indexBase, qualityUsed);
    if (stdDevs == null || !stdDevs.finite()) {
      return Optional.empty();
    }

    double thetaStd = stdDevs.theta();
    if (AprilTagVisionConstants.ignoreMegatag2Rotation()) {
      thetaStd = AprilTagVisionConstants.getLimelightLargeVariance();
    }
    Matrix<N3, N1> visionStdDevs = VecBuilder.fill(stdDevs.x(), stdDevs.y(), thetaStd);

    return Optional.of(
        new VisionFieldPoseEstimate(
            fieldToRobot, cam.megatagPoseEstimate.timestampSeconds(), visionStdDevs, tagCount));
  }

  private VisionFieldPoseEstimate fuseEstimates(
      VisionFieldPoseEstimate a, VisionFieldPoseEstimate b) {
    if (poseHistory == null) {
      return b;
    }
    if (b.timestampSeconds() < a.timestampSeconds()) {
      VisionFieldPoseEstimate tmp = a;
      a = b;
      b = tmp;
    }

    Optional<Pose2d> poseAAtTime = poseHistory.getFieldToRobot(a.timestampSeconds());
    Optional<Pose2d> poseBAtTime = poseHistory.getFieldToRobot(b.timestampSeconds());
    if (poseAAtTime.isEmpty() || poseBAtTime.isEmpty()) {
      return b;
    }

    var a_T_b = poseBAtTime.get().minus(poseAAtTime.get());
    Pose2d poseA = a.visionRobotPoseMeters().transformBy(a_T_b);
    Pose2d poseB = b.visionRobotPoseMeters();

    var varianceA = a.visionMeasurementStdDevs().elementTimes(a.visionMeasurementStdDevs());
    var varianceB = b.visionMeasurementStdDevs().elementTimes(b.visionMeasurementStdDevs());

    Rotation2d fusedHeading =
        new Rotation2d(
            poseA.getRotation().getCos() / varianceA.get(2, 0)
                + poseB.getRotation().getCos() / varianceB.get(2, 0),
            poseA.getRotation().getSin() / varianceA.get(2, 0)
                + poseB.getRotation().getSin() / varianceB.get(2, 0));

    double weightAx = 1.0 / varianceA.get(0, 0);
    double weightAy = 1.0 / varianceA.get(1, 0);
    double weightBx = 1.0 / varianceB.get(0, 0);
    double weightBy = 1.0 / varianceB.get(1, 0);

    Pose2d fusedPose =
        new Pose2d(
            new Translation2d(
                (poseA.getTranslation().getX() * weightAx
                        + poseB.getTranslation().getX() * weightBx)
                    / (weightAx + weightBx),
                (poseA.getTranslation().getY() * weightAy
                        + poseB.getTranslation().getY() * weightBy)
                    / (weightAy + weightBy)),
            fusedHeading);

    Matrix<N3, N1> fusedStdDev =
        VecBuilder.fill(
            Math.sqrt(1.0 / (weightAx + weightBx)),
            Math.sqrt(1.0 / (weightAy + weightBy)),
            Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

    int numTags = a.numTags() + b.numTags();
    double time = b.timestampSeconds();

    return new VisionFieldPoseEstimate(fusedPose, time, fusedStdDev, numTags);
  }

  private void logCameraInputs(String prefix, VisionIO.VisionIOInputs cam) {
    Logger.recordOutput(prefix + "/SeesTarget", cam.seesTarget);
    Logger.recordOutput(prefix + "/MegatagCount", cam.megatagCount);

    if (DriverStation.isDisabled()) {
      SmartDashboard.putBoolean(prefix + "/SeesTarget", cam.seesTarget);
      SmartDashboard.putNumber(prefix + "/MegatagCount", cam.megatagCount);
    }

    if (cam.pose3d != null) {
      Logger.recordOutput(prefix + "/Pose3d", cam.pose3d);
    }

    if (cam.megatagPoseEstimate != null) {
      Logger.recordOutput(prefix + "/MegatagPoseEstimate", cam.megatagPoseEstimate.fieldToRobot());
      Logger.recordOutput(prefix + "/Quality", cam.megatagPoseEstimate.quality());
      Logger.recordOutput(prefix + "/AvgTagArea", cam.megatagPoseEstimate.avgTagArea());
    }

    if (cam.fiducialObservations != null) {
      Logger.recordOutput(prefix + "/FiducialCount", cam.fiducialObservations.length);
    }
  }

  private double getStdDev(VisionIO.VisionIOInputs cam, int index) {
    double[] stdDevs =
        cam.standardDeviations == null || cam.standardDeviations.length <= index
            ? AprilTagVisionConstants.getLimelightStandardDeviations()
            : cam.standardDeviations;
    if (stdDevs == null || stdDevs.length <= index) {
      return 0.0;
    }
    double value = stdDevs[index];
    return Double.isFinite(value) ? value : 0.0;
  }

  private static double sanitizeQuality(double quality) {
    if (!Double.isFinite(quality)) {
      return 0.0;
    }
    if (quality < 0.0) {
      return 0.0;
    }
    if (quality > 1.0) {
      return 1.0;
    }
    return quality;
  }

  private LimelightStdDevs computeLimelightStdDevs(
      VisionIO.VisionIOInputs cam, int indexBase, double qualityUsed) {
    double scaleFactor = 1.0 / Math.max(qualityUsed, 1e-6);
    double xStd = getStdDev(cam, indexBase) * scaleFactor;
    double yStd = getStdDev(cam, indexBase + 1) * scaleFactor;
    double rotStd = getStdDev(cam, indexBase + 2) * scaleFactor;
    boolean finite = isFinite(xStd) && isFinite(yStd) && isFinite(rotStd);
    return new LimelightStdDevs(xStd, yStd, rotStd, finite);
  }

  private void logLimelightDiagnostics(String cameraLabel, VisionIO.VisionIOInputs cam) {
    String prefix = "AprilTagVision/" + cameraLabel + "/LimelightDiagnostics";
    boolean connected = cam.connected;
    boolean seesTarget = cam.seesTarget;
    boolean hasMegatag = cam.megatagPoseEstimate != null;
    int fiducialCount = cam.fiducialObservations == null ? 0 : cam.fiducialObservations.length;
    int tagCount = hasMegatag ? cam.megatagPoseEstimate.fiducialIds().length : 0;
    double qualityRaw = hasMegatag ? cam.megatagPoseEstimate.quality() : Double.NaN;
    double qualityUsed = sanitizeQuality(qualityRaw);
    boolean qualityFinite = Double.isFinite(qualityRaw);
    boolean poseFinite = hasMegatag && isFinitePose(cam.megatagPoseEstimate.fieldToRobot());
    boolean tagCountValid = tagCount > 0;
    boolean singleTagQualityPass =
        !(tagCount == 1
            && qualityUsed < AprilTagVisionConstants.getMegatag2SingleTagQualityCutoff());
    boolean inFieldBounds =
        hasMegatag && isWithinFieldBounds(cam.megatagPoseEstimate.fieldToRobot());

    LimelightStdDevs stdDevs =
        hasMegatag
            ? computeLimelightStdDevs(
                cam, AprilTagVisionConstants.LIMELIGHT_MEGATAG2_X_STDDEV_INDEX, qualityUsed)
            : null;
    boolean stdDevsFinite = stdDevs != null && stdDevs.finite();

    boolean wouldAccept =
        useVision
            && connected
            && hasMegatag
            && poseFinite
            && stdDevsFinite
            && tagCountValid
            && singleTagQualityPass
            && inFieldBounds;

    String rejectReason;
    if (!useVision) {
      rejectReason = "VISION_DISABLED";
    } else if (!connected) {
      rejectReason = "DISCONNECTED";
    } else if (!hasMegatag) {
      rejectReason = "NO_MEGATAG";
    } else if (!poseFinite) {
      rejectReason = "POSE_NONFINITE";
    } else if (!tagCountValid) {
      rejectReason = "NO_TAGS";
    } else if (!singleTagQualityPass) {
      rejectReason = "LOW_SINGLE_TAG_QUALITY";
    } else if (!inFieldBounds) {
      rejectReason = "OUT_OF_FIELD";
    } else if (!stdDevsFinite) {
      rejectReason = "STDDEV_NONFINITE";
    } else {
      rejectReason = "ACCEPTED";
    }

    Logger.recordOutput(prefix + "/Connected", connected);
    Logger.recordOutput(prefix + "/SeesTarget", seesTarget);
    Logger.recordOutput(prefix + "/HasMegaTagPose", hasMegatag);
    Logger.recordOutput(prefix + "/TagCount", tagCount);
    Logger.recordOutput(prefix + "/FiducialCount", fiducialCount);
    Logger.recordOutput(prefix + "/PoseFinite", poseFinite);
    Logger.recordOutput(prefix + "/TagCountValid", tagCountValid);
    Logger.recordOutput(prefix + "/SingleTagQualityPass", singleTagQualityPass);
    Logger.recordOutput(prefix + "/InFieldBounds", inFieldBounds);
    Logger.recordOutput(prefix + "/QualityRaw", qualityRaw);
    Logger.recordOutput(prefix + "/QualityUsed", qualityUsed);
    Logger.recordOutput(prefix + "/QualityFinite", qualityFinite);
    Logger.recordOutput(prefix + "/StdDevX", stdDevs != null ? stdDevs.x() : Double.NaN);
    Logger.recordOutput(prefix + "/StdDevY", stdDevs != null ? stdDevs.y() : Double.NaN);
    Logger.recordOutput(prefix + "/StdDevTheta", stdDevs != null ? stdDevs.theta() : Double.NaN);
    Logger.recordOutput(prefix + "/StdDevsFinite", stdDevsFinite);
    Logger.recordOutput(prefix + "/WouldAccept", wouldAccept);
    Logger.recordOutput(prefix + "/RejectReason", rejectReason);
  }

  private boolean isFinitePose(Pose2d pose) {
    if (pose == null) {
      return false;
    }
    if (!isFinite(pose.getX()) || !isFinite(pose.getY())) {
      return false;
    }
    Rotation2d rotation = pose.getRotation();
    if (rotation == null) {
      return false;
    }
    double cos = rotation.getCos();
    double sin = rotation.getSin();
    if (!isFinite(cos) || !isFinite(sin)) {
      return false;
    }
    return !(Math.abs(cos) < 1e-9 && Math.abs(sin) < 1e-9);
  }

  private boolean isWithinFieldBounds(Pose2d pose) {
    double margin = AprilTagVisionConstants.getFieldBorderMarginMeters();
    double x = pose.getX();
    double y = pose.getY();
    return x >= -margin
        && x <= GlobalConstants.FieldConstants.fieldLength + margin
        && y >= -margin
        && y <= GlobalConstants.FieldConstants.fieldWidth + margin;
  }

  private record LimelightStdDevs(double x, double y, double theta, boolean finite) {}

  /** Functional interface defining a consumer that processes vision-based pose estimates. */
  @FunctionalInterface
  public interface VisionConsumer {
    /**
     * Accepts a vision pose estimate for processing.
     *
     * @param visionRobotPoseMeters The estimated robot pose, in meters.
     * @param timestampSeconds The timestamp of the observation for latency compensation, in
     *     seconds.
     * @param visionMeasurementStdDevs The standard deviations of the measurement.
     */
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  private static final class PoseHistory {
    private final double historyWindowSec;
    private final Supplier<Pose2d> poseSupplier;
    private final DoubleSupplier yawRateRadPerSecSupplier;
    private final ArrayDeque<Sample> samples = new ArrayDeque<>();

    private PoseHistory(
        double historyWindowSec,
        Supplier<Pose2d> poseSupplier,
        DoubleSupplier yawRateRadPerSecSupplier) {
      this.historyWindowSec = historyWindowSec;
      this.poseSupplier = poseSupplier;
      this.yawRateRadPerSecSupplier = yawRateRadPerSecSupplier;
    }

    private void update(double timestampSeconds) {
      samples.addLast(
          new Sample(timestampSeconds, poseSupplier.get(), yawRateRadPerSecSupplier.getAsDouble()));
      while (!samples.isEmpty()
          && timestampSeconds - samples.getFirst().timestampSeconds > historyWindowSec) {
        samples.removeFirst();
      }
    }

    private Optional<Pose2d> getFieldToRobot(double timestampSeconds) {
      if (samples.isEmpty()) {
        return Optional.empty();
      }
      if (timestampSeconds < samples.getFirst().timestampSeconds
          || timestampSeconds > samples.getLast().timestampSeconds) {
        return Optional.empty();
      }

      Sample previous = null;
      for (Sample sample : samples) {
        if (sample.timestampSeconds >= timestampSeconds) {
          if (previous == null) {
            return Optional.of(sample.pose);
          }
          double t =
              (timestampSeconds - previous.timestampSeconds)
                  / (sample.timestampSeconds - previous.timestampSeconds);
          return Optional.of(previous.pose.interpolate(sample.pose, t));
        }
        previous = sample;
      }
      return Optional.empty();
    }
  }

  private record Sample(double timestampSeconds, Pose2d pose, double yawRateRadPerSec) {}
}
