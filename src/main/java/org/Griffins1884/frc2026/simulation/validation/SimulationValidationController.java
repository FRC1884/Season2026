package org.Griffins1884.frc2026.simulation.validation;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;
import org.Griffins1884.frc2026.simulation.drive.Season2026DriveSimulation;
import org.Griffins1884.frc2026.simulation.maple.Rebuilt2026FieldModel;
import org.Griffins1884.frc2026.simulation.visualization.RobotStateVisualizer.RenderTelemetry;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.util.RobotLogging;

/**
 * Aggressive unattended bump-validation controller for Season2026's real simulateJava path.
 *
 * <p>This runner executes multiple deterministic bump variants and classifies whether the sim can
 * represent support loss/unloading/jump behavior.
 */
public final class SimulationValidationController implements SimulationValidationRoutine {
  private static final ObjectMapper JSON =
      new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);

  private static final double MAX_IDLE_DISPLACEMENT_METERS = 0.03;
  private static final double MAX_IDLE_ROTATION_RAD = 0.03;
  private static final double MIN_BUMP_Z_GAIN_METERS = 0.05;
  private static final double MIN_BUMP_PITCH_RAD = 0.08;
  private static final double MIN_BUMP_VERTICAL_SPEED_MPS = 0.05;
  private static final double AIRBORNE_MIN_DURATION_SEC = 0.08;
  private static final double MINOR_UNLOAD_GAP_METERS = 0.003;
  private static final double MEANINGFUL_AIRBORNE_GAP_METERS = 0.01;
  private static final double RECONTACT_LINEAR_SPEED_LIMIT = 0.35;
  private static final double RECONTACT_ANGULAR_SPEED_LIMIT = 0.75;
  private static final double RENDER_POSE_MATCH_TOLERANCE_METERS = 1e-3;
  private static final double RENDER_POSE_MATCH_TOLERANCE_RAD = 1e-3;
  private static final double SIM_LOOP_PERIOD_SECONDS = 0.02;
  private static final int RESET_SETTLE_CAPTURE_CYCLES = 8;

  private static final List<RunVariant> VARIANTS =
      List.of(
          new RunVariant(
              "medium_speed",
              new Pose2d(3.48, 2.49, new Rotation2d()),
              0.0,
              -0.40,
              0.0,
              0.60,
              0.95,
              0.90,
              0.80,
              false),
          new RunVariant(
              "high_speed",
              new Pose2d(2.15, 2.49, new Rotation2d()),
              0.0,
              -1.00,
              0.0,
              0.60,
              2.00,
              1.20,
              0.80,
              false),
          new RunVariant(
              "max_practical",
              new Pose2d(2.10, 2.49, new Rotation2d()),
              0.0,
              -1.00,
              0.0,
              0.60,
              2.05,
              1.20,
              0.90,
              false),
          new RunVariant(
              "turn_moderate",
              new Pose2d(2.15, 2.49, new Rotation2d()),
              0.0,
              -1.00,
              0.25,
              0.60,
              2.00,
              1.20,
              0.80,
              false),
          new RunVariant(
              "turn_moderate_flat_control",
              new Pose2d(2.15, 2.49, new Rotation2d()),
              0.0,
              -1.00,
              0.25,
              0.60,
              2.00,
              1.20,
              0.80,
              true),
          new RunVariant(
              "turn_aggressive",
              new Pose2d(2.15, 2.49, new Rotation2d()),
              0.0,
              -1.00,
              0.28,
              0.60,
              2.00,
              1.20,
              0.80,
              false),
          new RunVariant(
              "turn_aggressive_flat_control",
              new Pose2d(2.15, 2.49, new Rotation2d()),
              0.0,
              -1.00,
              0.28,
              0.60,
              2.00,
              1.20,
              0.80,
              true),
          new RunVariant(
              "diagonal_turn",
              new Pose2d(2.45, 2.49, new Rotation2d()),
              -0.15,
              -1.00,
              0.30,
              0.60,
              2.00,
              1.20,
              0.80,
              false),
          new RunVariant(
              "diagonal_turn_flat_control",
              new Pose2d(2.45, 2.49, new Rotation2d()),
              -0.15,
              -1.00,
              0.30,
              0.60,
              2.00,
              1.20,
              0.80,
              true),
          new RunVariant(
              "high_speed_flat_control",
              new Pose2d(2.15, 2.49, new Rotation2d()),
              0.0,
              -1.00,
              0.0,
              0.60,
              2.00,
              1.20,
              0.80,
              true),
          new RunVariant(
              "max_practical_flat_control",
              new Pose2d(2.10, 2.49, new Rotation2d()),
              0.0,
              -1.00,
              0.0,
              0.60,
              2.05,
              1.20,
              0.90,
              true));

  private final SwerveSubsystem drive;
  private final Season2026DriveSimulation terrainSimulation;
  private final Supplier<RenderTelemetry> renderTelemetrySupplier;
  private final Consumer<Pose2d> simulationReset;
  private final XboxControllerSim controllerSim = new XboxControllerSim(0);
  private final Path outputPath;
  private final List<TraceSample> trace = new ArrayList<>();
  private final List<Segment> schedule;
  private double elapsedSimulationSeconds = 0.0;
  private int activeSegmentIndex = -1;
  private int suppressedCaptureCycles = 0;
  private boolean initialized;
  private boolean wroteTrace;

  private SimulationValidationController(
      SwerveSubsystem drive,
      Season2026DriveSimulation terrainSimulation,
      Supplier<RenderTelemetry> renderTelemetrySupplier,
      Consumer<Pose2d> simulationReset,
      Path outputPath) {
    this.drive = drive;
    this.terrainSimulation = terrainSimulation;
    this.renderTelemetrySupplier = renderTelemetrySupplier;
    this.simulationReset = simulationReset;
    this.outputPath = outputPath;
    this.schedule = buildSchedule(VARIANTS);
  }

  public static SimulationValidationController create(
      SwerveSubsystem drive,
      Season2026DriveSimulation terrainSimulation,
      Supplier<RenderTelemetry> renderTelemetrySupplier,
      Consumer<Pose2d> simulationReset) {
    String outputDirProperty =
        System.getProperty("season2026.simValidation.outputDir", "build/sim-validation");
    Path outputDir = Path.of(outputDirProperty);
    return new SimulationValidationController(
        drive,
        terrainSimulation,
        renderTelemetrySupplier,
        simulationReset,
        outputDir.resolve("simulatejava-trace.json"));
  }

  public void applyInputs() {
    if (wroteTrace) {
      return;
    }
    ensureInitialized();
    Segment segment = updateActiveSegment(elapsedSeconds());

    controllerSim.setAxisCount(6);
    controllerSim.setButtonCount(10);
    controllerSim.setPOVCount(1);
    DriverStationSim.setJoystickType(0, GenericHID.HIDType.kXInputGamepad.value);
    DriverStationSim.setJoystickIsXbox(0, true);
    DriverStationSim.setJoystickName(0, "Season2026 Aggressive Bump Validation Xbox");
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(segment.enabled());

    controllerSim.setLeftY(segment.leftY());
    controllerSim.setLeftX(segment.leftX());
    controllerSim.setRightX(segment.rightX());
    controllerSim.setRightY(0.0);
    controllerSim.setLeftTriggerAxis(0.0);
    controllerSim.setRightTriggerAxis(0.0);
    controllerSim.setAButton(false);
    controllerSim.setBButton(false);
    controllerSim.setXButton(false);
    controllerSim.setYButton(false);
    controllerSim.setBackButton(false);
    controllerSim.setStartButton(false);
    controllerSim.setLeftBumperButton(false);
    controllerSim.setRightBumperButton(false);
    controllerSim.setPOV(-1);
    controllerSim.notifyNewData();
  }

  public void captureStepAndMaybeExit() {
    if (wroteTrace) {
      return;
    }
    ensureInitialized();
    double nowSeconds = elapsedSeconds();
    Segment segment = updateActiveSegment(nowSeconds);
    if (suppressedCaptureCycles > 0) {
      suppressedCaptureCycles--;
      elapsedSimulationSeconds += SIM_LOOP_PERIOD_SECONDS;
      return;
    }
    trace.add(capture(nowSeconds, segment));
    elapsedSimulationSeconds += SIM_LOOP_PERIOD_SECONDS;

    if (nowSeconds < totalDurationSeconds()) {
      return;
    }

    DriverStationSim.setEnabled(false);
    controllerSim.setLeftY(0.0);
    controllerSim.setLeftX(0.0);
    controllerSim.setRightX(0.0);
    controllerSim.notifyNewData();
    writeTraceAndExit();
  }

  private void ensureInitialized() {
    if (initialized) {
      return;
    }
    controllerSim.setAxisCount(6);
    controllerSim.setButtonCount(10);
    controllerSim.setPOVCount(1);
    DriverStationSim.setJoystickType(0, GenericHID.HIDType.kXInputGamepad.value);
    DriverStationSim.setJoystickIsXbox(0, true);
    DriverStationSim.setJoystickName(0, "Season2026 Aggressive Bump Validation Xbox");
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    elapsedSimulationSeconds = 0.0;
    initialized = true;
    RobotLogging.info("[SIM VALIDATION] Enabled aggressive bump-validation script.");
  }

  private double elapsedSeconds() {
    return elapsedSimulationSeconds;
  }

  private Segment updateActiveSegment(double elapsedSeconds) {
    Segment segment = schedule.get(schedule.size() - 1);
    int segmentIndex = schedule.size() - 1;
    for (int index = 0; index < schedule.size(); index++) {
      Segment candidate = schedule.get(index);
      if (elapsedSeconds < candidate.endSeconds()) {
        segment = candidate;
        segmentIndex = index;
        break;
      }
    }

    if (segmentIndex != activeSegmentIndex) {
      activeSegmentIndex = segmentIndex;
      Rebuilt2026FieldModel.setValidationFlatTerrainOverride(
          segment.variant().flatTerrainControl());
      if (segment.phase() == Phase.SETTLE_START) {
        DriverStationSim.setEnabled(false);
        controllerSim.setLeftY(0.0);
        controllerSim.setLeftX(0.0);
        controllerSim.setRightX(0.0);
        controllerSim.notifyNewData();
        simulationReset.accept(segment.variant().startPose());
        suppressedCaptureCycles = RESET_SETTLE_CAPTURE_CYCLES;
      }
    }
    return segment;
  }

  private TraceSample capture(double elapsedSeconds, Segment segment) {
    Pose2d pose = drive.getPose();
    ChassisSpeeds robotRelative = drive.getRobotRelativeSpeeds();
    ChassisSpeeds commandedRobotRelative = drive.getCommandedRobotRelativeSpeeds();
    Translation2d fieldVelocity = drive.getFieldVelocity();
    Translation2d fieldAcceleration = drive.getFieldAcceleration();
    Season2026DriveSimulation.ValidationSnapshot simSnapshot =
        terrainSimulation.validationSnapshot();
    Season2026DriveSimulation.SupportSnapshot supportDiagnostics = simSnapshot.support();
    Season2026DriveSimulation.TerrainContactSnapshot terrainContactSample =
        simSnapshot.terrainContact();
    Season2026DriveSimulation.TractionSnapshot tractionState = simSnapshot.traction();
    RenderTelemetry renderTelemetry =
        renderTelemetrySupplier == null ? null : renderTelemetrySupplier.get();
    Pose3d renderedPose = renderTelemetry == null ? null : renderTelemetry.publishedRobotPose3d();
    Pose3d terrainAdjustedPose =
        renderTelemetry == null ? null : renderTelemetry.terrainAdjustedRobotPose3d();
    SwerveSubsystem.ValidationModuleSample[] moduleSamples = drive.getValidationModuleSamples();
    double averageDriveAuthorityScale =
        java.util.Arrays.stream(moduleSamples)
            .mapToDouble(SwerveSubsystem.ValidationModuleSample::terrainDriveAuthorityScale)
            .average()
            .orElse(Double.NaN);
    double minDriveAuthorityScale =
        java.util.Arrays.stream(moduleSamples)
            .mapToDouble(SwerveSubsystem.ValidationModuleSample::terrainDriveAuthorityScale)
            .min()
            .orElse(Double.NaN);
    double averageDesiredSpeedMetersPerSecond =
        java.util.Arrays.stream(moduleSamples)
            .mapToDouble(SwerveSubsystem.ValidationModuleSample::desiredSpeedMetersPerSec)
            .average()
            .orElse(Double.NaN);
    double averageActualSpeedMetersPerSecond =
        java.util.Arrays.stream(moduleSamples)
            .mapToDouble(SwerveSubsystem.ValidationModuleSample::actualSpeedMetersPerSec)
            .average()
            .orElse(Double.NaN);
    double forwardAccelerationMetersPerSecondSq =
        (fieldAcceleration.getX() * Math.cos(pose.getRotation().getRadians()))
            + (fieldAcceleration.getY() * Math.sin(pose.getRotation().getRadians()));
    double chassisSpeedSquared =
        (fieldVelocity.getX() * fieldVelocity.getX())
            + (fieldVelocity.getY() * fieldVelocity.getY())
            + Math.pow(simSnapshot.verticalVelocityMetersPerSecond(), 2.0);
    double translationalKineticEnergyJoules =
        0.5
            * org.Griffins1884.frc2026.simulation.maple.Rebuilt2026FieldModel
                .CHASSIS_MASS_PROPERTIES.massKg()
            * chassisSpeedSquared;

    return new TraceSample(
        elapsedSeconds,
        segment.variant().name(),
        segment.phase(),
        segment.enabled(),
        segment.leftX(),
        segment.leftY(),
        segment.rightX(),
        pose.getX(),
        pose.getY(),
        pose.getRotation().getRadians(),
        robotRelative.vxMetersPerSecond,
        robotRelative.vyMetersPerSecond,
        robotRelative.omegaRadiansPerSecond,
        fieldVelocity.getX(),
        fieldVelocity.getY(),
        fieldAcceleration.getX(),
        fieldAcceleration.getY(),
        drive.getYawRateDegreesPerSec(),
        simSnapshot.rollRateRadPerSec(),
        simSnapshot.pitchRateRadPerSec(),
        drive.getRequestedTranslationalMps(),
        drive.getRequestedOmegaRadPerSec(),
        robotRelative.vxMetersPerSecond,
        robotRelative.vyMetersPerSecond,
        forwardAccelerationMetersPerSecondSq,
        translationalKineticEnergyJoules,
        commandedRobotRelative.vxMetersPerSecond,
        commandedRobotRelative.vyMetersPerSecond,
        commandedRobotRelative.omegaRadiansPerSecond,
        simSnapshot.chassisZMeters(),
        simSnapshot.rollRadians(),
        simSnapshot.pitchRadians(),
        simSnapshot.verticalVelocityMetersPerSecond(),
        simSnapshot.verticalAccelerationMetersPerSecondSq(),
        supportDiagnostics.supportPlaneHeightMeters(),
        supportDiagnostics.chassisZAboveTerrainMeters(),
        supportDiagnostics.bodyBottomToTerrainGapMeters(),
        supportDiagnostics.bodySupported(),
        supportDiagnostics.supportContactCount(),
        supportDiagnostics.frontLeftSupported(),
        supportDiagnostics.frontRightSupported(),
        supportDiagnostics.rearLeftSupported(),
        supportDiagnostics.rearRightSupported(),
        supportDiagnostics.actualAirborne(),
        terrainContactSample.feature(),
        terrainContactSample.terrainHeightMeters(),
        terrainContactSample.underbodyClearanceMarginMeters(),
        Double.isFinite(terrainContactSample.underbodyClearanceMarginMeters())
            && terrainContactSample.underbodyClearanceMarginMeters() <= 0.0,
        terrainContactSample.traversableSurface(),
        terrainContactSample.clearanceSatisfied(),
        tractionState.tractionAvailable(),
        tractionState.totalNormalForceNewtons(),
        tractionState.averageNormalizedLoad(),
        tractionState.frontLeftNormalForceNewtons(),
        tractionState.frontRightNormalForceNewtons(),
        tractionState.rearLeftNormalForceNewtons(),
        tractionState.rearRightNormalForceNewtons(),
        averageDriveAuthorityScale,
        minDriveAuthorityScale,
        averageDesiredSpeedMetersPerSecond,
        averageActualSpeedMetersPerSecond,
        poseZ(renderedPose),
        rotationX(renderedPose),
        rotationY(renderedPose),
        renderedPoseGapMeters(poseZ(renderedPose), supportDiagnostics.terrainHeightMeters()),
        renderedPoseGapMeters(poseZ(renderedPose), supportDiagnostics.supportPlaneHeightMeters()),
        poseZ(terrainAdjustedPose),
        rotationX(terrainAdjustedPose),
        rotationY(terrainAdjustedPose),
        renderTelemetry != null && renderTelemetry.usingAuthoritativeRobotPose(),
        List.of(moduleSamples));
  }

  private void writeTraceAndExit() {
    if (wroteTrace) {
      return;
    }
    wroteTrace = true;
    Rebuilt2026FieldModel.setValidationFlatTerrainOverride(false);
    AggressiveBumpSummary summary = summarize(trace);
    TraceReport report = new TraceReport(trace, summary);
    try {
      Files.createDirectories(outputPath.getParent());
      JSON.writeValue(outputPath.toFile(), report);
      RobotLogging.info("[SIM VALIDATION] Trace written to " + outputPath.toAbsolutePath());
      for (VariantSummary variant : summary.variants()) {
        RobotLogging.info("[SIM VALIDATION] variant=" + variant.name());
        for (PhaseSummary phaseSummary : variant.phaseSummaries()) {
          RobotLogging.info("[SIM VALIDATION] " + phaseSummary.toAuditLine());
        }
        RobotLogging.info("[SIM VALIDATION] jump_verdict=" + variant.jumpVerdict().toAuditLine());
      }
      RobotLogging.info("[SIM VALIDATION] overall=" + summary.overallConclusion());
    } catch (IOException e) {
      RobotLogging.error("[SIM VALIDATION] Failed to write trace", e);
    }
    HAL.exitMain();
  }

  private static AggressiveBumpSummary summarize(List<TraceSample> trace) {
    List<VariantSummary> variants = new ArrayList<>();
    for (RunVariant variant : VARIANTS) {
      List<TraceSample> variantSamples =
          trace.stream().filter(sample -> sample.variant().equals(variant.name())).toList();
      variants.add(summarizeVariant(variant, variantSamples));
    }
    OverallConclusion overallConclusion = overallConclusion(variants);
    return new AggressiveBumpSummary(variants, overallConclusion);
  }

  private static VariantSummary summarizeVariant(RunVariant variant, List<TraceSample> samples) {
    List<PhaseSummary> phaseSummaries = new ArrayList<>();
    for (Phase phase :
        List.of(
            Phase.SETTLE_START, Phase.APPROACH_BUMP, Phase.COAST_AFTER_BUMP, Phase.STOP_SETTLE)) {
      phaseSummaries.add(summarizePhase(variant.name(), phase, samples));
    }
    JumpVerdict jumpVerdict = summarizeJump(variant.name(), samples);
    return new VariantSummary(variant.name(), phaseSummaries, jumpVerdict);
  }

  private static PhaseSummary summarizePhase(
      String variantName, Phase phase, List<TraceSample> samples) {
    List<TraceSample> phaseSamples =
        samples.stream().filter(sample -> sample.phase() == phase).toList();
    if (phaseSamples.isEmpty()) {
      return PhaseSummary.empty(variantName, phase);
    }
    TraceSample first = phaseSamples.get(0);
    TraceSample last = phaseSamples.get(phaseSamples.size() - 1);
    double deltaX = last.poseX() - first.poseX();
    double deltaY = last.poseY() - first.poseY();
    double deltaHeading = last.headingRadians() - first.headingRadians();
    double maxLinearSpeed =
        phaseSamples.stream()
            .mapToDouble(
                sample ->
                    Math.hypot(sample.fieldVxMetersPerSecond(), sample.fieldVyMetersPerSecond()))
            .max()
            .orElse(0.0);
    double maxAngularSpeed =
        phaseSamples.stream()
            .mapToDouble(sample -> Math.abs(sample.robotOmegaRadPerSec()))
            .max()
            .orElse(0.0);
    double maxZ = phaseSamples.stream().mapToDouble(TraceSample::zMeters).max().orElse(Double.NaN);
    double minZ = phaseSamples.stream().mapToDouble(TraceSample::zMeters).min().orElse(Double.NaN);
    double maxPitch =
        phaseSamples.stream()
            .mapToDouble(sample -> Math.abs(sample.pitchRadians()))
            .max()
            .orElse(0.0);
    double maxRoll =
        phaseSamples.stream()
            .mapToDouble(sample -> Math.abs(sample.rollRadians()))
            .max()
            .orElse(0.0);
    double maxVerticalSpeed =
        phaseSamples.stream()
            .mapToDouble(sample -> Math.abs(sample.verticalVelocityMetersPerSecond()))
            .max()
            .orElse(0.0);
    int supportLossFrames =
        (int) phaseSamples.stream().filter(sample -> sample.supportContactCount() == 0).count();
    int actualAirborneFrames =
        (int) phaseSamples.stream().filter(TraceSample::actualAirborne).count();
    double maxBodyGapMeters =
        phaseSamples.stream()
            .mapToDouble(TraceSample::bodyBottomToTerrainGapMeters)
            .filter(Double::isFinite)
            .max()
            .orElse(0.0);
    double maxRenderedGapMeters =
        phaseSamples.stream()
            .mapToDouble(TraceSample::renderedBodyBottomToTerrainGapMeters)
            .filter(Double::isFinite)
            .max()
            .orElse(0.0);
    Set<String> features = new LinkedHashSet<>();
    for (TraceSample sample : phaseSamples) {
      features.add(sample.terrainFeature());
    }
    return new PhaseSummary(
        variantName,
        phase,
        first.elapsedSeconds(),
        last.elapsedSeconds(),
        deltaX,
        deltaY,
        deltaHeading,
        maxZ,
        minZ,
        maxPitch,
        maxRoll,
        maxVerticalSpeed,
        maxLinearSpeed,
        maxAngularSpeed,
        supportLossFrames,
        actualAirborneFrames,
        maxBodyGapMeters,
        maxRenderedGapMeters,
        List.copyOf(features));
  }

  private static JumpVerdict summarizeJump(String variantName, List<TraceSample> samples) {
    List<TraceSample> bumpSamples =
        samples.stream()
            .filter(
                sample ->
                    sample.phase() == Phase.APPROACH_BUMP
                        || sample.phase() == Phase.COAST_AFTER_BUMP)
            .toList();
    List<TraceSample> settleSamples =
        samples.stream().filter(sample -> sample.phase() == Phase.SETTLE_START).toList();

    double baselineNormalForce =
        settleSamples.stream()
            .mapToDouble(TraceSample::totalNormalForceNewtons)
            .filter(Double::isFinite)
            .average()
            .orElse(Double.NaN);
    double minNormalForce =
        bumpSamples.stream()
            .mapToDouble(TraceSample::totalNormalForceNewtons)
            .filter(Double::isFinite)
            .min()
            .orElse(Double.NaN);
    double normalForceDropRatio =
        Double.isFinite(baselineNormalForce) && baselineNormalForce > 1e-9
            ? minNormalForce / baselineNormalForce
            : Double.NaN;

    boolean supportLossObserved =
        bumpSamples.stream().anyMatch(sample -> sample.supportContactCount() == 0);
    Interval supportLossInterval =
        intervalForSamples(bumpSamples, sample -> sample.supportContactCount() == 0);
    boolean actualAirborneObserved = bumpSamples.stream().anyMatch(TraceSample::actualAirborne);
    Interval actualAirborneInterval = intervalForSamples(bumpSamples, TraceSample::actualAirborne);
    double maxZ = bumpSamples.stream().mapToDouble(TraceSample::zMeters).max().orElse(0.0);
    double minZ = bumpSamples.stream().mapToDouble(TraceSample::zMeters).min().orElse(0.0);
    double maxChassisZAboveTerrainMeters =
        bumpSamples.stream()
            .mapToDouble(TraceSample::chassisZAboveTerrainMeters)
            .filter(Double::isFinite)
            .max()
            .orElse(0.0);
    double maxBodyBottomGapMeters =
        bumpSamples.stream()
            .mapToDouble(TraceSample::bodyBottomToTerrainGapMeters)
            .filter(Double::isFinite)
            .max()
            .orElse(0.0);
    double maxRenderedBodyBottomGapMeters =
        bumpSamples.stream()
            .mapToDouble(TraceSample::renderedBodyBottomToTerrainGapMeters)
            .filter(Double::isFinite)
            .max()
            .orElse(0.0);
    double maxVerticalSpeed =
        bumpSamples.stream()
            .mapToDouble(sample -> Math.abs(sample.verticalVelocityMetersPerSecond()))
            .max()
            .orElse(0.0);
    double maxVerticalAcceleration =
        bumpSamples.stream()
            .mapToDouble(sample -> Math.abs(sample.verticalAccelerationMetersPerSecondSq()))
            .max()
            .orElse(0.0);
    boolean visitedBump =
        bumpSamples.stream().anyMatch(sample -> sample.terrainFeature().contains("BUMP"));
    boolean recontactObserved =
        supportLossObserved
            && supportLossInterval.resumeTimeSeconds() != null
            && finalLinearSpeed(samples) <= RECONTACT_LINEAR_SPEED_LIMIT
            && finalAngularSpeed(samples) <= RECONTACT_ANGULAR_SPEED_LIMIT;
    boolean renderedPoseMatchesPhysics =
        bumpSamples.stream()
            .filter(sample -> sample.renderUsesAuthoritativePose())
            .allMatch(
                sample ->
                    differenceWithin(
                            sample.zMeters(),
                            sample.renderedPoseZMeters(),
                            RENDER_POSE_MATCH_TOLERANCE_METERS)
                        && differenceWithin(
                            sample.rollRadians(),
                            sample.renderedRollRadians(),
                            RENDER_POSE_MATCH_TOLERANCE_RAD)
                        && differenceWithin(
                            sample.pitchRadians(),
                            sample.renderedPitchRadians(),
                            RENDER_POSE_MATCH_TOLERANCE_RAD));
    boolean shoulderClimb =
        !supportLossObserved && (maxZ - minZ) >= 0.03 && maxPitch(samples) >= 0.05;
    boolean meaningfulRenderedSeparationObserved =
        bumpSamples.stream()
            .anyMatch(
                sample ->
                    sample.renderUsesAuthoritativePose()
                        && sample.renderedBodyBottomToTerrainGapMeters()
                            >= MEANINGFUL_AIRBORNE_GAP_METERS);

    RunClassification classification;
    if (actualAirborneObserved
        && actualAirborneInterval.maxDurationSeconds() >= AIRBORNE_MIN_DURATION_SEC
        && maxBodyBottomGapMeters >= MEANINGFUL_AIRBORNE_GAP_METERS
        && meaningfulRenderedSeparationObserved
        && renderedPoseMatchesPhysics
        && recontactObserved
        && visitedBump) {
      classification = RunClassification.MEANINGFUL_AIRBORNE_JUMP;
    } else if (supportLossObserved
        || maxBodyBottomGapMeters >= MINOR_UNLOAD_GAP_METERS
        || maxRenderedBodyBottomGapMeters >= MINOR_UNLOAD_GAP_METERS) {
      classification = RunClassification.MINOR_UNLOADING;
    } else if (visitedBump
        && (maxZ - minZ) >= MIN_BUMP_Z_GAIN_METERS
        && maxVerticalSpeed >= MIN_BUMP_VERTICAL_SPEED_MPS
        && maxPitch(samples) >= MIN_BUMP_PITCH_RAD) {
      classification = RunClassification.SUPPORTED_DYNAMIC_CLIMB;
    } else if (shoulderClimb) {
      classification = RunClassification.SUPPORTED_DYNAMIC_CLIMB;
    } else {
      classification = RunClassification.SUPPORTED_DYNAMIC_CLIMB;
    }

    return new JumpVerdict(
        variantName,
        classification,
        supportLossObserved,
        supportLossInterval.maxDurationSeconds(),
        supportLossInterval.firstTimeSeconds(),
        supportLossInterval.resumeTimeSeconds(),
        actualAirborneObserved,
        actualAirborneInterval.maxDurationSeconds(),
        actualAirborneInterval.firstTimeSeconds(),
        actualAirborneInterval.resumeTimeSeconds(),
        maxZ,
        maxZ - minZ,
        maxChassisZAboveTerrainMeters,
        maxBodyBottomGapMeters,
        maxRenderedBodyBottomGapMeters,
        maxVerticalSpeed,
        maxVerticalAcceleration,
        normalForceDropRatio,
        renderedPoseMatchesPhysics,
        recontactObserved,
        visitedBump);
  }

  private static Interval intervalForSamples(
      List<TraceSample> samples, Predicate<TraceSample> predicate) {
    Double firstTime = null;
    Double resumeTime = null;
    double maxDuration = 0.0;
    Double currentStart = null;
    for (TraceSample sample : samples) {
      if (predicate.test(sample)) {
        if (firstTime == null) {
          firstTime = sample.elapsedSeconds();
        }
        if (currentStart == null) {
          currentStart = sample.elapsedSeconds();
        }
      } else if (currentStart != null) {
        double duration = sample.elapsedSeconds() - currentStart;
        if (duration > maxDuration) {
          maxDuration = duration;
        }
        if (resumeTime == null) {
          resumeTime = sample.elapsedSeconds();
        }
        currentStart = null;
      }
    }
    if (currentStart != null) {
      double duration = samples.get(samples.size() - 1).elapsedSeconds() - currentStart;
      if (duration > maxDuration) {
        maxDuration = duration;
      }
    }
    return new Interval(firstTime, resumeTime, maxDuration);
  }

  private static double poseZ(Pose3d pose) {
    return pose == null ? Double.NaN : pose.getZ();
  }

  private static double rotationX(Pose3d pose) {
    return pose == null ? Double.NaN : pose.getRotation().getX();
  }

  private static double rotationY(Pose3d pose) {
    return pose == null ? Double.NaN : pose.getRotation().getY();
  }

  private static double renderedPoseGapMeters(double renderedPoseZMeters, double terrainZMeters) {
    return Double.isFinite(renderedPoseZMeters) && Double.isFinite(terrainZMeters)
        ? renderedPoseZMeters - terrainZMeters
        : Double.NaN;
  }

  private static boolean differenceWithin(double expected, double actual, double tolerance) {
    return Double.isFinite(expected)
        && Double.isFinite(actual)
        && Math.abs(expected - actual) <= tolerance;
  }

  private static double finalLinearSpeed(List<TraceSample> samples) {
    if (samples.isEmpty()) {
      return 0.0;
    }
    TraceSample sample = samples.get(samples.size() - 1);
    return Math.hypot(sample.fieldVxMetersPerSecond(), sample.fieldVyMetersPerSecond());
  }

  private static double finalAngularSpeed(List<TraceSample> samples) {
    return samples.isEmpty()
        ? 0.0
        : Math.abs(samples.get(samples.size() - 1).robotOmegaRadPerSec());
  }

  private static double maxPitch(List<TraceSample> samples) {
    return samples.stream()
        .mapToDouble(sample -> Math.abs(sample.pitchRadians()))
        .max()
        .orElse(0.0);
  }

  private static OverallConclusion overallConclusion(List<VariantSummary> variants) {
    if (variants.stream()
        .anyMatch(
            variant ->
                variant.jumpVerdict().classification()
                    == RunClassification.MEANINGFUL_AIRBORNE_JUMP)) {
      return OverallConclusion.MEANINGFUL_AIRBORNE_JUMP_OBSERVED;
    }
    if (variants.stream()
        .anyMatch(
            variant ->
                variant.jumpVerdict().classification() == RunClassification.MINOR_UNLOADING)) {
      return OverallConclusion.MINOR_UNLOADING_OBSERVED;
    }
    return OverallConclusion.SUPPORTED_DYNAMIC_CLIMB_ONLY;
  }

  private List<Segment> buildSchedule(List<RunVariant> variants) {
    List<Segment> segments = new ArrayList<>();
    double cursor = 0.0;
    for (RunVariant variant : variants) {
      double settleEnd = cursor + variant.settleSeconds();
      segments.add(
          new Segment(variant, Phase.SETTLE_START, cursor, settleEnd, true, false, 0.0, 0.0, 0.0));
      cursor = settleEnd;

      double approachEnd = cursor + variant.approachSeconds();
      segments.add(
          new Segment(
              variant,
              Phase.APPROACH_BUMP,
              cursor,
              approachEnd,
              false,
              true,
              variant.approachLeftX(),
              variant.approachLeftY(),
              variant.approachRightX()));
      cursor = approachEnd;

      double coastEnd = cursor + variant.coastSeconds();
      segments.add(
          new Segment(
              variant, Phase.COAST_AFTER_BUMP, cursor, coastEnd, false, true, 0.0, 0.0, 0.0));
      cursor = coastEnd;

      double stopEnd = cursor + variant.stopSeconds();
      segments.add(
          new Segment(variant, Phase.STOP_SETTLE, cursor, stopEnd, false, true, 0.0, 0.0, 0.0));
      cursor = stopEnd;
    }
    segments.add(
        new Segment(
            VARIANTS.get(VARIANTS.size() - 1),
            Phase.COMPLETE,
            cursor,
            cursor + 0.25,
            false,
            false,
            0.0,
            0.0,
            0.0));
    return List.copyOf(segments);
  }

  private double totalDurationSeconds() {
    return schedule.get(schedule.size() - 1).endSeconds();
  }

  public record RunVariant(
      String name,
      Pose2d startPose,
      double approachLeftX,
      double approachLeftY,
      double approachRightX,
      double settleSeconds,
      double approachSeconds,
      double coastSeconds,
      double stopSeconds,
      boolean flatTerrainControl) {}

  private record Segment(
      RunVariant variant,
      Phase phase,
      double startSeconds,
      double endSeconds,
      boolean resetPose,
      boolean enabled,
      double leftX,
      double leftY,
      double rightX) {}

  public enum Phase {
    SETTLE_START("settle_start"),
    APPROACH_BUMP("approach_bump"),
    COAST_AFTER_BUMP("coast_after_bump"),
    STOP_SETTLE("stop_settle"),
    COMPLETE("complete");

    private final String wireName;

    Phase(String wireName) {
      this.wireName = wireName;
    }

    public String wireName() {
      return wireName;
    }
  }

  public record TraceSample(
      double elapsedSeconds,
      String variant,
      Phase phase,
      boolean enabled,
      double leftX,
      double leftY,
      double rightX,
      double poseX,
      double poseY,
      double headingRadians,
      double robotVxMetersPerSecond,
      double robotVyMetersPerSecond,
      double robotOmegaRadPerSec,
      double fieldVxMetersPerSecond,
      double fieldVyMetersPerSecond,
      double fieldAxMetersPerSecondSq,
      double fieldAyMetersPerSecondSq,
      double yawRateDegreesPerSecond,
      double rollRateRadPerSec,
      double pitchRateRadPerSec,
      double requestedTranslationalMps,
      double requestedOmegaRadPerSec,
      double forwardSpeedMetersPerSecond,
      double lateralSpeedMetersPerSecond,
      double forwardAccelerationMetersPerSecondSq,
      double translationalKineticEnergyJoules,
      double commandedRobotVxMetersPerSecond,
      double commandedRobotVyMetersPerSecond,
      double commandedRobotOmegaRadPerSec,
      double zMeters,
      double rollRadians,
      double pitchRadians,
      double verticalVelocityMetersPerSecond,
      double verticalAccelerationMetersPerSecondSq,
      double supportPlaneHeightMeters,
      double chassisZAboveTerrainMeters,
      double bodyBottomToTerrainGapMeters,
      boolean bodySupported,
      int supportContactCount,
      boolean frontLeftSupported,
      boolean frontRightSupported,
      boolean rearLeftSupported,
      boolean rearRightSupported,
      boolean actualAirborne,
      String terrainFeature,
      double terrainHeightMeters,
      double underbodyClearanceMarginMeters,
      boolean underbodyCollisionLikely,
      boolean traversableSurface,
      boolean clearanceSatisfied,
      boolean tractionAvailable,
      double totalNormalForceNewtons,
      double averageNormalizedLoad,
      double frontLeftNormalForceNewtons,
      double frontRightNormalForceNewtons,
      double rearLeftNormalForceNewtons,
      double rearRightNormalForceNewtons,
      double averageDriveAuthorityScale,
      double minDriveAuthorityScale,
      double averageDesiredModuleSpeedMetersPerSecond,
      double averageActualModuleSpeedMetersPerSecond,
      double renderedPoseZMeters,
      double renderedRollRadians,
      double renderedPitchRadians,
      double renderedChassisZAboveTerrainMeters,
      double renderedBodyBottomToTerrainGapMeters,
      double terrainAdjustedPoseZMeters,
      double terrainAdjustedRollRadians,
      double terrainAdjustedPitchRadians,
      boolean renderUsesAuthoritativePose,
      List<SwerveSubsystem.ValidationModuleSample> modules) {}

  public record TraceReport(List<TraceSample> trace, AggressiveBumpSummary summary) {}

  public record AggressiveBumpSummary(
      List<VariantSummary> variants, OverallConclusion overallConclusion) {}

  public record VariantSummary(
      String name, List<PhaseSummary> phaseSummaries, JumpVerdict jumpVerdict) {}

  public record PhaseSummary(
      String variant,
      Phase phase,
      double startSeconds,
      double endSeconds,
      double deltaX,
      double deltaY,
      double deltaHeading,
      double maxZ,
      double minZ,
      double maxPitch,
      double maxRoll,
      double maxVerticalSpeed,
      double maxLinearSpeed,
      double maxAngularSpeed,
      int supportLossFrames,
      int actualAirborneFrames,
      double maxBodyGapMeters,
      double maxRenderedGapMeters,
      List<String> terrainFeatures) {
    static PhaseSummary empty(String variant, Phase phase) {
      return new PhaseSummary(
          variant, phase, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0.0,
          0.0, List.of());
    }

    String toAuditLine() {
      return String.format(
          Locale.ROOT,
          "%s/%s t=[%.2f,%.2f] dx=%.3f dy=%.3f dh=%.3f z=[%.3f,%.3f] pitchMax=%.3f rollMax=%.3f vzMax=%.3f vmax=%.3f wmax=%.3f supportLossFrames=%d airborneFrames=%d maxGap=%.3f renderedGap=%.3f features=%s",
          variant,
          phase.wireName(),
          startSeconds,
          endSeconds,
          deltaX,
          deltaY,
          deltaHeading,
          minZ,
          maxZ,
          maxPitch,
          maxRoll,
          maxVerticalSpeed,
          maxLinearSpeed,
          maxAngularSpeed,
          supportLossFrames,
          actualAirborneFrames,
          maxBodyGapMeters,
          maxRenderedGapMeters,
          terrainFeatures);
    }
  }

  public record JumpVerdict(
      String variant,
      RunClassification classification,
      boolean supportLossObserved,
      double maxSupportLossDurationSeconds,
      Double firstSupportLossTimeSeconds,
      Double supportResumeTimeSeconds,
      boolean actualAirborneObserved,
      double maxActualAirborneDurationSeconds,
      Double firstActualAirborneTimeSeconds,
      Double actualAirborneResumeTimeSeconds,
      double peakZMeters,
      double zTravelMeters,
      double maxChassisZAboveTerrainMeters,
      double maxBodyBottomGapMeters,
      double maxRenderedBodyBottomGapMeters,
      double peakVerticalSpeedMetersPerSecond,
      double peakVerticalAccelerationMetersPerSecondSq,
      double normalForceDropRatio,
      boolean renderedPoseMatchesPhysics,
      boolean recontactObserved,
      boolean visitedBumpFeature) {
    String toAuditLine() {
      return String.format(
          Locale.ROOT,
          "%s variant=%s supportLoss=%s supportLossDuration=%.3f firstLoss=%s resume=%s actualAirborne=%s airborneDuration=%.3f firstAirborne=%s airborneResume=%s peakZ=%.3f zTravel=%.3f zAboveTerrain=%.3f bodyGap=%.3f renderedGap=%.3f peakVz=%.3f peakAz=%.3f normalForceDrop=%.3f renderMatchesPhysics=%s recontact=%s visitedBump=%s",
          classification.name(),
          variant,
          supportLossObserved,
          maxSupportLossDurationSeconds,
          firstSupportLossTimeSeconds == null
              ? "n/a"
              : String.format(Locale.ROOT, "%.3f", firstSupportLossTimeSeconds),
          supportResumeTimeSeconds == null
              ? "n/a"
              : String.format(Locale.ROOT, "%.3f", supportResumeTimeSeconds),
          actualAirborneObserved,
          maxActualAirborneDurationSeconds,
          firstActualAirborneTimeSeconds == null
              ? "n/a"
              : String.format(Locale.ROOT, "%.3f", firstActualAirborneTimeSeconds),
          actualAirborneResumeTimeSeconds == null
              ? "n/a"
              : String.format(Locale.ROOT, "%.3f", actualAirborneResumeTimeSeconds),
          peakZMeters,
          zTravelMeters,
          maxChassisZAboveTerrainMeters,
          maxBodyBottomGapMeters,
          maxRenderedBodyBottomGapMeters,
          peakVerticalSpeedMetersPerSecond,
          peakVerticalAccelerationMetersPerSecondSq,
          normalForceDropRatio,
          renderedPoseMatchesPhysics,
          recontactObserved,
          visitedBumpFeature);
    }
  }

  private record Interval(
      Double firstTimeSeconds, Double resumeTimeSeconds, double maxDurationSeconds) {}

  public enum RunClassification {
    SUPPORTED_DYNAMIC_CLIMB,
    MINOR_UNLOADING,
    MEANINGFUL_AIRBORNE_JUMP
  }

  public enum OverallConclusion {
    MEANINGFUL_AIRBORNE_JUMP_OBSERVED,
    MINOR_UNLOADING_OBSERVED,
    SUPPORTED_DYNAMIC_CLIMB_ONLY
  }
}
