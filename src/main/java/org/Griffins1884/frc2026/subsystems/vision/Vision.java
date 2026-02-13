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
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
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
  private final Set<String> lastArbitrationPairKeys = new HashSet<>();

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

    List<LimelightEstimateCandidate> candidates = new ArrayList<>();

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("LimelightVision/" + io[i].getCameraConstants().cameraName(), inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);

      String cameraLabel = io[i].getCameraConstants().cameraName();
      logCameraInputs("Vision/" + cameraLabel, inputs[i]);
      logLimelightDiagnostics(cameraLabel, inputs[i]);
      buildLimelightCandidate(cameraLabel, inputs[i]).ifPresent(candidates::add);
    }

    if (!useVision) {
      logArbitration(
          new ArbitrationResult(
              ArbitrationMode.NONE,
              candidates.size(),
              0,
              "VISION_DISABLED",
              Optional.empty(),
              List.of()));
      Logger.recordOutput("Vision/usingVision", false);
      Logger.recordOutput("Vision/rejectReason", "VISION_DISABLED");
      Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
      return;
    }

    if (!yawRateOk) {
      logArbitration(
          new ArbitrationResult(
              ArbitrationMode.NONE,
              candidates.size(),
              0,
              "HIGH_YAW_RATE",
              Optional.empty(),
              List.of()));
      Logger.recordOutput("Vision/usingVision", true);
      Logger.recordOutput("Vision/rejectReason", "HIGH_YAW_RATE");
      Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
      return;
    }

    Logger.recordOutput("Vision/usingVision", true);
    Logger.recordOutput("Vision/rejectReason", "ACCEPTED");

    ArbitrationResult arbitration = arbitrateCandidates(candidates);
    logArbitration(arbitration);
    arbitration
        .estimate()
        .ifPresent(
            est -> {
              Logger.recordOutput("Vision/fusedAccepted", est.visionRobotPoseMeters());
              consumer.accept(
                  est.visionRobotPoseMeters(),
                  est.timestampSeconds(),
                  est.visionMeasurementStdDevs());
            });

    Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
  }

  private Optional<LimelightEstimateCandidate> buildLimelightCandidate(
      String cameraLabel, VisionIO.VisionIOInputs cam) {
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
    int numTags = cam.megatagPoseEstimate.fiducialIds().length;
    double score =
        computeCandidateScore(
            numTags,
            qualityUsed,
            cam.megatagPoseEstimate.avgTagArea(),
            stdDevs.x(),
            stdDevs.y(),
            thetaStd);

    return Optional.of(
        new LimelightEstimateCandidate(
            cameraLabel,
            new VisionFieldPoseEstimate(
                fieldToRobot, cam.megatagPoseEstimate.timestampSeconds(), visionStdDevs, numTags),
            score));
  }

  private ArbitrationResult arbitrateCandidates(List<LimelightEstimateCandidate> candidates) {
    if (candidates.isEmpty()) {
      return new ArbitrationResult(ArbitrationMode.NONE, 0, 0, "", Optional.empty(), List.of());
    }
    if (candidates.size() == 1) {
      LimelightEstimateCandidate only = candidates.get(0);
      return new ArbitrationResult(
          ArbitrationMode.SINGLE,
          1,
          1,
          only.cameraLabel(),
          Optional.of(only.estimate()),
          List.of());
    }

    List<CameraPairDelta> pairDeltas = new ArrayList<>();
    boolean allConsistent = true;
    for (int i = 0; i < candidates.size(); i++) {
      for (int j = i + 1; j < candidates.size(); j++) {
        CameraPairDelta delta = compareCandidates(candidates.get(i), candidates.get(j), i, j);
        pairDeltas.add(delta);
        allConsistent &= delta.consistent();
      }
    }

    int consistentGroupSize = computeLargestConsistentGroupSize(candidates, pairDeltas);
    if (allConsistent) {
      List<LimelightEstimateCandidate> ordered = new ArrayList<>(candidates);
      ordered.sort(
          Comparator.comparingDouble(candidate -> candidate.estimate().timestampSeconds()));
      VisionFieldPoseEstimate fused = ordered.get(0).estimate();
      for (int i = 1; i < ordered.size(); i++) {
        fused = fuseEstimates(fused, ordered.get(i).estimate());
      }
      return new ArbitrationResult(
          ArbitrationMode.FUSED,
          candidates.size(),
          consistentGroupSize,
          "FUSED",
          Optional.of(fused),
          pairDeltas);
    }

    LimelightEstimateCandidate best = chooseBestCandidate(candidates);
    return new ArbitrationResult(
        ArbitrationMode.FALLBACK_SINGLE,
        candidates.size(),
        consistentGroupSize,
        best.cameraLabel(),
        Optional.of(best.estimate()),
        pairDeltas);
  }

  private LimelightEstimateCandidate chooseBestCandidate(
      List<LimelightEstimateCandidate> candidates) {
    return candidates.stream()
        .min(
            Comparator.comparingDouble(LimelightEstimateCandidate::score)
                .thenComparing(
                    candidate -> candidate.estimate().numTags(), Comparator.reverseOrder())
                .thenComparing(LimelightEstimateCandidate::cameraLabel))
        .orElseThrow();
  }

  private int computeLargestConsistentGroupSize(
      List<LimelightEstimateCandidate> candidates, List<CameraPairDelta> pairDeltas) {
    int n = candidates.size();
    if (n == 0) {
      return 0;
    }
    if (n == 1) {
      return 1;
    }
    boolean[][] consistent = new boolean[n][n];
    for (int i = 0; i < n; i++) {
      consistent[i][i] = true;
    }
    for (CameraPairDelta pairDelta : pairDeltas) {
      int a = pairDelta.cameraAIndex();
      int b = pairDelta.cameraBIndex();
      consistent[a][b] = pairDelta.consistent();
      consistent[b][a] = pairDelta.consistent();
    }

    int best = 1;
    int maxMask = 1 << n;
    for (int mask = 1; mask < maxMask; mask++) {
      int size = Integer.bitCount(mask);
      if (size <= best) {
        continue;
      }
      boolean isClique = true;
      for (int i = 0; i < n && isClique; i++) {
        if ((mask & (1 << i)) == 0) {
          continue;
        }
        for (int j = i + 1; j < n; j++) {
          if ((mask & (1 << j)) == 0) {
            continue;
          }
          if (!consistent[i][j]) {
            isClique = false;
            break;
          }
        }
      }
      if (isClique) {
        best = size;
      }
    }
    return best;
  }

  private CameraPairDelta compareCandidates(
      LimelightEstimateCandidate a,
      LimelightEstimateCandidate b,
      int cameraAIndex,
      int cameraBIndex) {
    PoseComparison comparison = comparePosesWithAlignment(a.estimate(), b.estimate());
    boolean consistent =
        comparison.translationDeltaMeters()
                <= AprilTagVisionConstants.getLimelightMultiCamMaxDeltaMeters()
            && comparison.headingDeltaDeg()
                <= AprilTagVisionConstants.getLimelightMultiCamMaxDeltaDeg();
    return new CameraPairDelta(
        a.cameraLabel(),
        b.cameraLabel(),
        comparison.translationDeltaMeters(),
        comparison.headingDeltaDeg(),
        consistent,
        cameraAIndex,
        cameraBIndex);
  }

  private PoseComparison comparePosesWithAlignment(
      VisionFieldPoseEstimate a, VisionFieldPoseEstimate b) {
    if (a.timestampSeconds() <= b.timestampSeconds()) {
      Pose2d poseAAtB =
          alignEstimateToTimestamp(a, b.timestampSeconds()).orElse(a.visionRobotPoseMeters());
      Pose2d poseB = b.visionRobotPoseMeters();
      return new PoseComparison(
          poseAAtB.getTranslation().getDistance(poseB.getTranslation()),
          Math.abs(poseAAtB.getRotation().minus(poseB.getRotation()).getDegrees()));
    }

    Pose2d poseBAtA =
        alignEstimateToTimestamp(b, a.timestampSeconds()).orElse(b.visionRobotPoseMeters());
    Pose2d poseA = a.visionRobotPoseMeters();
    return new PoseComparison(
        poseA.getTranslation().getDistance(poseBAtA.getTranslation()),
        Math.abs(poseA.getRotation().minus(poseBAtA.getRotation()).getDegrees()));
  }

  private Optional<Pose2d> alignEstimateToTimestamp(
      VisionFieldPoseEstimate estimate, double timestampSeconds) {
    if (Math.abs(estimate.timestampSeconds() - timestampSeconds) < 1e-6) {
      return Optional.of(estimate.visionRobotPoseMeters());
    }
    if (poseHistory == null) {
      return Optional.empty();
    }
    Optional<Pose2d> fromPose = poseHistory.getFieldToRobot(estimate.timestampSeconds());
    Optional<Pose2d> toPose = poseHistory.getFieldToRobot(timestampSeconds);
    if (fromPose.isEmpty() || toPose.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(
        estimate.visionRobotPoseMeters().transformBy(toPose.get().minus(fromPose.get())));
  }

  private VisionFieldPoseEstimate fuseEstimates(
      VisionFieldPoseEstimate a, VisionFieldPoseEstimate b) {
    Pose2d poseB = b.visionRobotPoseMeters();
    Pose2d poseA =
        alignEstimateToTimestamp(a, b.timestampSeconds()).orElse(a.visionRobotPoseMeters());

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

  private void logArbitration(ArbitrationResult arbitration) {
    Set<String> currentPairKeys = new HashSet<>();
    Logger.recordOutput("Vision/Arbitration/CandidateCount", arbitration.candidateCount());
    Logger.recordOutput(
        "Vision/Arbitration/ConsistentGroupSize", arbitration.consistentGroupSize());
    Logger.recordOutput("Vision/Arbitration/SelectedCameraName", arbitration.selectedCameraName());
    Logger.recordOutput("Vision/Arbitration/Mode", arbitration.mode().name());
    Logger.recordOutput("Vision/Arbitration/PairCount", arbitration.pairDeltas().size());
    for (CameraPairDelta pair : arbitration.pairDeltas()) {
      String key = sanitizeLogKey(pair.cameraA()) + "__" + sanitizeLogKey(pair.cameraB());
      currentPairKeys.add(key);
      String prefix = "Vision/Arbitration/Pairs/" + key;
      Logger.recordOutput(prefix + "/DeltaMeters", pair.translationDeltaMeters());
      Logger.recordOutput(prefix + "/DeltaDeg", pair.headingDeltaDeg());
      Logger.recordOutput(prefix + "/Consistent", pair.consistent());
    }
    for (String staleKey : lastArbitrationPairKeys) {
      if (currentPairKeys.contains(staleKey)) {
        continue;
      }
      String prefix = "Vision/Arbitration/Pairs/" + staleKey;
      Logger.recordOutput(prefix + "/DeltaMeters", Double.NaN);
      Logger.recordOutput(prefix + "/DeltaDeg", Double.NaN);
      Logger.recordOutput(prefix + "/Consistent", false);
    }
    lastArbitrationPairKeys.clear();
    lastArbitrationPairKeys.addAll(currentPairKeys);
  }

  private static String sanitizeLogKey(String key) {
    if (key == null || key.isBlank()) {
      return "UNKNOWN";
    }
    return key.replace('/', '_').replace(' ', '_');
  }

  private double computeCandidateScore(
      int tagCount,
      double qualityUsed,
      double avgTagArea,
      double xStd,
      double yStd,
      double thetaStd) {
    double stdComponent = xStd + yStd + (0.25 * thetaStd);
    double qualityPenalty = 1.0 - qualityUsed;
    double areaPenalty = 1.0 / (1.0 + Math.max(avgTagArea, 0.0));
    double tagPenalty = 1.0 / Math.max(1, tagCount);
    double score = stdComponent + qualityPenalty + (0.35 * areaPenalty) + tagPenalty;
    if (!Double.isFinite(score)) {
      return Double.MAX_VALUE;
    }
    return Math.max(score, 0.0);
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

  private enum ArbitrationMode {
    NONE,
    SINGLE,
    FUSED,
    FALLBACK_SINGLE
  }

  private record LimelightEstimateCandidate(
      String cameraLabel, VisionFieldPoseEstimate estimate, double score) {}

  private record PoseComparison(double translationDeltaMeters, double headingDeltaDeg) {}

  private record CameraPairDelta(
      String cameraA,
      String cameraB,
      double translationDeltaMeters,
      double headingDeltaDeg,
      boolean consistent,
      int cameraAIndex,
      int cameraBIndex) {}

  private record ArbitrationResult(
      ArbitrationMode mode,
      int candidateCount,
      int consistentGroupSize,
      String selectedCameraName,
      Optional<VisionFieldPoseEstimate> estimate,
      List<CameraPairDelta> pairDeltas) {}

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
