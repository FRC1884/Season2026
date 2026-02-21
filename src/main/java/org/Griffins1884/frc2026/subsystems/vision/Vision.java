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
import java.util.Arrays;
import java.util.Deque;
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
  private final Alert[] outlierAlerts;
  private final Deque<YawSample>[] yawHistory;
  private final double[] lastAcceptedTimestampsSec;
  private final boolean useLimelightFusion;
  private final PoseHistory poseHistory;
  @Setter private boolean useVision = true;
  private final DoubleSupplier yawRateRadPerSecSupplier;
  private Integer exclusiveTagId = null;
  private double ignoreVisionUntilTimestamp = 0.0;

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

    @SuppressWarnings("unchecked")
    Deque<YawSample>[] yawHistoryInit = new ArrayDeque[io.length];
    for (int i = 0; i < yawHistoryInit.length; i++) {
      yawHistoryInit[i] = new ArrayDeque<>();
    }
    yawHistory = yawHistoryInit;

    lastAcceptedTimestampsSec = new double[io.length];
    Arrays.fill(lastAcceptedTimestampsSec, Double.NEGATIVE_INFINITY);

    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera \"" + io[i].getCameraConstants().cameraName() + "\" is disconnected.",
              Alert.AlertType.kWarning);
    }
    this.outlierAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      outlierAlerts[i] =
          new Alert(
              "Vision Outlier detected on camera \""
                  + io[i].getCameraConstants().cameraName()
                  + "\".",
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

  private static double wrapDegrees(double degrees) {
    double wrapped = degrees % 360.0;
    if (wrapped > 180.0) {
      wrapped -= 360.0;
    }
    if (wrapped < -180.0) {
      wrapped += 360.0;
    }
    return wrapped;
  }

  private void periodicLimelight() {
    double startTime = Timer.getFPGATimestamp();
    if (poseHistory != null) {
      poseHistory.update(startTime);
    }

    if (startTime < ignoreVisionUntilTimestamp) {
      Logger.recordOutput("Vision/usingVision", false);
      Logger.recordOutput("Vision/rejectReason", "RESET_SUPPRESS");
      Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
      return;
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
      updateResiduals(inputs[i]);
      logLimelightDiagnostics(cameraLabel, inputs[i]);
      estimates.add(buildLimelightEstimate(i, cameraLabel, inputs[i]));

      boolean isOutlier =
          inputs[i].rejectReason == VisionIO.RejectReason.LARGE_TRANSLATION_RESIDUAL
              || inputs[i].rejectReason == VisionIO.RejectReason.LARGE_ROTATION_RESIDUAL;
      outlierAlerts[i].set(isOutlier);
    }

    if (!useVision) {
      Logger.recordOutput("Vision/usingVision", false);
      Logger.recordOutput("Vision/rejectReason", "VISION_DISABLED");
      Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
      return;
    }

    if (!yawRateOk) {
      Logger.recordOutput("Vision/usingVision", false);
      Logger.recordOutput("Vision/rejectReason", "HIGH_YAW_RATE");
      Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
      return;
    }

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

    boolean hasAccepted = accepted.isPresent();
    Logger.recordOutput("Vision/usingVision", hasAccepted);
    Logger.recordOutput("Vision/rejectReason", hasAccepted ? "ACCEPTED" : "NO_ACCEPTED_ESTIMATE");

    accepted.ifPresent(
        est -> {
          Logger.recordOutput("Vision/fusedAccepted", est.visionRobotPoseMeters());
          consumer.accept(
              est.visionRobotPoseMeters(), est.timestampSeconds(), est.visionMeasurementStdDevs());
        });

    Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
  }

  private Optional<VisionFieldPoseEstimate> buildLimelightEstimate(
      int cameraIndex, String cameraLabel, VisionIO.VisionIOInputs cam) {
    if (!cam.connected || cam.megatagPoseEstimate == null) {
      return Optional.empty();
    }

    if (exclusiveTagId != null
        && !containsFiducialId(cam.megatagPoseEstimate.fiducialIds(), exclusiveTagId)) {
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
    if (!DriverStation.isDisabled()) {
      if (tagCount == 1
          && qualityUsed < AprilTagVisionConstants.getMegatag2SingleTagQualityCutoff()) {
        return Optional.empty();
      }
    }
    LimelightStdDevs stdDevs = computeLimelightStdDevs(cam, indexBase, qualityUsed);
    if (stdDevs == null || !stdDevs.finite()) {
      return Optional.empty();
    }

    double timestampSeconds = cam.megatagPoseEstimate.timestampSeconds();
    if (!isTimestampInOrder(cameraIndex, timestampSeconds)) {
      if (cameraLabel != null) {
        String prefix = "AprilTagVision/" + cameraLabel + "/Timestamp";
        Logger.recordOutput(prefix + "/OutOfOrder", true);
        Logger.recordOutput(prefix + "/LastAccepted", lastAcceptedTimestampsSec[cameraIndex]);
        Logger.recordOutput(prefix + "/Current", timestampSeconds);
      }
      return Optional.empty();
    }

    boolean allowYawGate = !AprilTagVisionConstants.ignoreMegatag2Rotation();
    double thetaStd =
        allowYawGate ? stdDevs.theta() : AprilTagVisionConstants.getLimelightLargeVariance();

    double frameAgeSec = Math.max(0.0, Timer.getFPGATimestamp() - timestampSeconds);
    double gyroYawDeg =
        getReferencePose(timestampSeconds)
            .map(pose -> Math.toDegrees(pose.getRotation().getRadians()))
            .orElse(0.0);
    double visionYawDeg = Math.toDegrees(fieldToRobot.getRotation().getRadians());
    double yawRateDegPerSec =
        yawRateRadPerSecSupplier != null
            ? Math.toDegrees(yawRateRadPerSecSupplier.getAsDouble())
            : 0.0;
    boolean yawGate = false;
    YawGateResult yawGateResult = YawGateResult.disabled();
    if (allowYawGate) {
      yawGateResult =
          evaluateYawGate(
              cameraIndex,
              cam,
              visionYawDeg,
              gyroYawDeg,
              yawRateDegPerSec,
              frameAgeSec,
              cam.residualTranslationMeters);
      yawGate = yawGateResult.passed();
      if (yawGate) {
        thetaStd = AprilTagVisionConstants.getLimelightYawStdDevStableRad();
      } else {
        thetaStd = AprilTagVisionConstants.getLimelightYawStdDevUnstable();
      }
    }
    if (cameraLabel != null) {
      String prefix = "AprilTagVision/" + cameraLabel + "/YawGate";
      Logger.recordOutput(prefix + "/Pass", yawGate);
      Logger.recordOutput(prefix + "/ResidualDeg", yawGateResult.residualDeg());
      Logger.recordOutput(prefix + "/FrameAgeSec", frameAgeSec);
      Logger.recordOutput(prefix + "/TagCountOk", yawGateResult.tagCountOk());
      Logger.recordOutput(prefix + "/DistanceOk", yawGateResult.distanceOk());
      Logger.recordOutput(prefix + "/YawRateOk", yawGateResult.yawRateOk());
      Logger.recordOutput(prefix + "/FrameAgeOk", yawGateResult.frameAgeOk());
      Logger.recordOutput(prefix + "/ResidualOk", yawGateResult.residualOk());
      Logger.recordOutput(prefix + "/YawResidualOk", yawGateResult.yawResidualOk());
      Logger.recordOutput(prefix + "/StableWindowOk", yawGateResult.stableOk());
      Logger.recordOutput(prefix + "/ThetaStdUsed", thetaStd);
    }
    Matrix<N3, N1> visionStdDevs = VecBuilder.fill(stdDevs.x(), stdDevs.y(), thetaStd);
    if (!DriverStation.isDisabled()) {
      if (AprilTagVisionConstants.LIMELIGHT_REJECT_OUTLIERS.get() > 0.5
          && Double.isFinite(cam.residualTranslationMeters)
          && (cam.residualTranslationMeters
              > AprilTagVisionConstants.LIMELIGHT_MAX_TRANSLATION_RESIDUAL_METERS.get())) {
        return Optional.empty();
      }
    }

    markTimestampAccepted(cameraIndex, timestampSeconds);
    return Optional.of(
        new VisionFieldPoseEstimate(fieldToRobot, timestampSeconds, visionStdDevs, tagCount));
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

  private void recordYawResidual(int cameraIndex, double timestampSeconds, double residualDeg) {
    if (cameraIndex < 0 || cameraIndex >= yawHistory.length) {
      return;
    }
    Deque<YawSample> history = yawHistory[cameraIndex];
    history.addLast(new YawSample(timestampSeconds, residualDeg));
    int maxSamples = Math.max(1, AprilTagVisionConstants.getLimelightYawGateStableWindow());
    while (history.size() > maxSamples) {
      history.removeFirst();
    }
  }

  private boolean isYawStable(int cameraIndex) {
    if (cameraIndex < 0 || cameraIndex >= yawHistory.length) {
      return false;
    }
    Deque<YawSample> history = yawHistory[cameraIndex];
    int required = Math.max(1, AprilTagVisionConstants.getLimelightYawGateStableWindow());
    if (history.size() < required) {
      return false;
    }
    double maxDelta = AprilTagVisionConstants.getLimelightYawGateStableDeltaDeg();
    Double first = null;
    for (YawSample sample : history) {
      if (first == null) {
        first = sample.residualDeg;
        continue;
      }
      if (Math.abs(sample.residualDeg - first) > maxDelta) {
        return false;
      }
    }
    return true;
  }

  private YawGateResult evaluateYawGate(
      int cameraIndex,
      VisionIO.VisionIOInputs cam,
      double visionYawDeg,
      double gyroYawDeg,
      double yawRateDegPerSec,
      double frameAgeSec,
      double residualTranslationMeters) {
    if (!AprilTagVisionConstants.isLimelightYawGateEnabled() || cam.megatagPoseEstimate == null) {
      return YawGateResult.disabled();
    }
    int tagCount = cam.megatagPoseEstimate.fiducialIds().length;
    boolean tagCountOk = tagCount >= 2;
    boolean distOk =
        cam.megatagPoseEstimate.avgTagDist()
            <= AprilTagVisionConstants.getLimelightYawGateMaxDistMeters();
    boolean yawRateOk =
        Math.abs(yawRateDegPerSec)
            <= AprilTagVisionConstants.getLimelightYawGateMaxYawRateDegPerSec();
    boolean frameAgeOk = frameAgeSec <= AprilTagVisionConstants.getLimelightYawGateMaxFrameAgeSec();
    boolean residualOk =
        residualTranslationMeters <= AprilTagVisionConstants.getLimelightYawGateMaxResidualMeters();
    double yawResidual = wrapDegrees(visionYawDeg - gyroYawDeg);
    boolean yawResidualOk =
        Math.abs(yawResidual) <= AprilTagVisionConstants.getLimelightYawGateMaxYawResidualDeg();

    boolean prereqOk =
        tagCountOk && distOk && yawRateOk && frameAgeOk && residualOk && yawResidualOk;
    boolean stableOk = false;
    if (prereqOk) {
      recordYawResidual(cameraIndex, cam.megatagPoseEstimate.timestampSeconds(), yawResidual);
      stableOk = isYawStable(cameraIndex);
    }
    boolean passed = prereqOk && stableOk;
    return new YawGateResult(
        passed,
        tagCountOk,
        distOk,
        yawRateOk,
        frameAgeOk,
        residualOk,
        yawResidualOk,
        stableOk,
        yawResidual);
  }

  private boolean isTimestampInOrder(int cameraIndex, double timestampSeconds) {
    if (cameraIndex < 0 || cameraIndex >= lastAcceptedTimestampsSec.length) {
      return true;
    }
    return timestampSeconds > lastAcceptedTimestampsSec[cameraIndex];
  }

  private void markTimestampAccepted(int cameraIndex, double timestampSeconds) {
    if (cameraIndex < 0 || cameraIndex >= lastAcceptedTimestampsSec.length) {
      return;
    }
    lastAcceptedTimestampsSec[cameraIndex] = timestampSeconds;
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

  private void updateResiduals(VisionIO.VisionIOInputs cam) {
    cam.residualTranslationMeters = Double.NaN;
    if (cam.megatagPoseEstimate == null) {
      return;
    }

    Optional<Pose2d> referencePose = getReferencePose(cam.megatagPoseEstimate.timestampSeconds());
    if (referencePose.isEmpty()) {
      return;
    }

    Pose2d visionPose = cam.megatagPoseEstimate.fieldToRobot();
    cam.residualTranslationMeters =
        referencePose.get().getTranslation().getDistance(visionPose.getTranslation());
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
    boolean exclusiveTagPass =
        exclusiveTagId == null
            || (hasMegatag
                && containsFiducialId(cam.megatagPoseEstimate.fiducialIds(), exclusiveTagId));

    boolean wouldAccept =
        useVision
            && connected
            && hasMegatag
            && poseFinite
            && stdDevsFinite
            && tagCountValid
            && singleTagQualityPass
            && inFieldBounds
            && exclusiveTagPass;

    VisionIO.RejectReason rejectReason;

    boolean residualFinite = Double.isFinite(cam.residualTranslationMeters);
    boolean residualsOk =
        !(AprilTagVisionConstants.LIMELIGHT_REJECT_OUTLIERS.get() > 0.5
            && residualFinite
            && (cam.residualTranslationMeters
                > AprilTagVisionConstants.LIMELIGHT_MAX_TRANSLATION_RESIDUAL_METERS.get()));

    if (!useVision) {
      rejectReason = VisionIO.RejectReason.VISION_DISABLED;
    } else if (!connected) {
      rejectReason = VisionIO.RejectReason.DISCONNECTED;
    } else if (!hasMegatag) {
      rejectReason = VisionIO.RejectReason.NO_MEGATAG;
    } else if (!poseFinite) {
      rejectReason = VisionIO.RejectReason.POSE_NONFINITE;
    } else if (!tagCountValid) {
      rejectReason = VisionIO.RejectReason.NO_TAGS;
    } else if (!singleTagQualityPass) {
      rejectReason = VisionIO.RejectReason.LOW_SINGLE_TAG_QUALITY;
    } else if (!inFieldBounds) {
      rejectReason = VisionIO.RejectReason.OUT_OF_FIELD;
    } else if (!exclusiveTagPass) {
      rejectReason = VisionIO.RejectReason.EXCLUSIVE_ID_MISMATCH;
    } else if (!stdDevsFinite) {
      rejectReason = VisionIO.RejectReason.STDDEV_NONFINITE;
    } else if (!residualsOk) {
      rejectReason = VisionIO.RejectReason.RESIDUAL_OUTLIER;
    } else {
      rejectReason = VisionIO.RejectReason.UNKNOWN;
    }

    wouldAccept = wouldAccept && residualsOk;

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
    Logger.recordOutput(prefix + "/ExclusiveTagPass", exclusiveTagPass);
    Logger.recordOutput(prefix + "/WouldAccept", wouldAccept);
    Logger.recordOutput(prefix + "/ResidualFinite", residualFinite);
    Logger.recordOutput(prefix + "/ResidualTranslationMeters", cam.residualTranslationMeters);
    Logger.recordOutput("ExclusiveTagActive", exclusiveTagId != null);

    cam.rejectReason = rejectReason;
  }

  private Optional<Pose2d> getReferencePose(double timestamp) {
    if (poseHistory == null) {
      return Optional.empty();
    }

    return poseHistory.getFieldToRobot(timestamp);
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

    private void clear() {
      samples.clear();
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

  private record YawSample(double timestampSeconds, double residualDeg) {}

  private record YawGateResult(
      boolean passed,
      boolean tagCountOk,
      boolean distanceOk,
      boolean yawRateOk,
      boolean frameAgeOk,
      boolean residualOk,
      boolean yawResidualOk,
      boolean stableOk,
      double residualDeg) {
    private static YawGateResult disabled() {
      return new YawGateResult(false, false, false, false, false, false, false, false, 0.0);
    }
  }

  public void setExclusiveTagId(int id) {
    exclusiveTagId = id;
  }

  public void clearExclusiveTagId() {
    exclusiveTagId = null;
  }

  public void resetPoseHistory() {
    if (poseHistory != null) {
      poseHistory.clear();
    }
  }

  public void suppressVisionForSeconds(double seconds) {
    ignoreVisionUntilTimestamp = Timer.getFPGATimestamp() + Math.max(0.0, seconds);
  }

  private boolean containsFiducialId(int[] ids, int target) {
    if (ids == null) {
      return false;
    }
    for (int id : ids) {
      if (id == target) {
        return true;
      }
    }
    return false;
  }

  private record Sample(double timestampSeconds, Pose2d pose, double yawRateRadPerSec) {}
}
