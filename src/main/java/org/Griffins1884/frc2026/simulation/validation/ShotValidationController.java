package org.Griffins1884.frc2026.simulation.validation;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.Griffins1884.frc2026.simulation.visualization.RobotStateVisualizer.RenderTelemetry;
import org.Griffins1884.frc2026.simulation.visualization.RobotStateVisualizer.ShotTelemetryRecord;
import org.Griffins1884.frc2026.simulation.visualization.RobotStateVisualizer.ShotTelemetrySample;
import org.Griffins1884.frc2026.simulation.visualization.RobotStateVisualizer.ShotTelemetrySnapshot;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.Superstructure.SuperstructureOutcome;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.util.RobotLogging;

/** Unattended shot-arc validation through the real Season2026 simulateJava release path. */
public final class ShotValidationController implements SimulationValidationRoutine {
  private static final ObjectMapper JSON =
      new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);

  private static final double SIM_LOOP_PERIOD_SECONDS = 0.02;
  private static final int RESET_SETTLE_CAPTURE_CYCLES = 8;
  private static final double MAX_PREDICTED_OBSERVED_PATH_ERROR_METERS = 0.20;
  private static final double MIN_FORWARD_DISTANCE_METERS = 0.25;
  private static final double MIN_APEX_RISE_METERS = 0.08;
  private static final double MAX_SAMPLE_STEP_EXTRA_METERS = 0.40;
  private static final int MAX_RELEASE_RETRIES = 1;

  private static final List<ShotCase> CASES =
      List.of(
          new ShotCase(
              "lower_energy_short", new Pose2d(0.35, 0.55, new Rotation2d()), 0.45, 6.0, 5.0, 0.35),
          new ShotCase("medium", new Pose2d(0.20, 0.35, new Rotation2d()), 0.45, 6.0, 5.0, 0.35),
          new ShotCase(
              "stronger_long", new Pose2d(0.15, 0.30, new Rotation2d()), 0.45, 6.0, 5.0, 0.35));

  private final SwerveSubsystem drive;
  private final Superstructure superstructure;
  private final Supplier<RenderTelemetry> renderTelemetrySupplier;
  private final Supplier<ShotTelemetrySnapshot> shotTelemetrySupplier;
  private final Consumer<Pose2d> simulationReset;
  private final XboxControllerSim controllerSim = new XboxControllerSim(0);
  private final Path outputPath;
  private final List<TraceSample> trace = new ArrayList<>();
  private final List<CaseResult> results = new ArrayList<>();

  private int activeCaseIndex = -1;
  private int activeCaseAttempt = 0;
  private ShotCase activeCase = null;
  private Phase phase = Phase.COMPLETE;
  private double phaseStartSeconds = 0.0;
  private double elapsedSimulationSeconds = 0.0;
  private int caseStartReleaseCount = 0;
  private int caseStartCompletedCount = 0;
  private int suppressedCaptureCycles = 0;
  private boolean initialized = false;
  private boolean wroteTrace = false;

  private ShotValidationController(
      SwerveSubsystem drive,
      Superstructure superstructure,
      Supplier<RenderTelemetry> renderTelemetrySupplier,
      Supplier<ShotTelemetrySnapshot> shotTelemetrySupplier,
      Consumer<Pose2d> simulationReset,
      Path outputPath) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.renderTelemetrySupplier = renderTelemetrySupplier;
    this.shotTelemetrySupplier = shotTelemetrySupplier;
    this.simulationReset = simulationReset;
    this.outputPath = outputPath;
  }

  public static ShotValidationController create(
      SwerveSubsystem drive,
      Superstructure superstructure,
      Supplier<RenderTelemetry> renderTelemetrySupplier,
      Supplier<ShotTelemetrySnapshot> shotTelemetrySupplier,
      Consumer<Pose2d> simulationReset) {
    String outputDirProperty =
        System.getProperty("season2026.simValidation.outputDir", "build/shot-validation");
    Path outputDir = Path.of(outputDirProperty);
    return new ShotValidationController(
        drive,
        superstructure,
        renderTelemetrySupplier,
        shotTelemetrySupplier,
        simulationReset,
        outputDir.resolve("shot-validation-trace.json"));
  }

  @Override
  public void applyInputs() {
    if (wroteTrace) {
      return;
    }
    ensureInitialized();

    boolean shootHeld = phase == Phase.WAIT_FOR_RELEASE;
    boolean enabled =
        phase == Phase.WAIT_FOR_RELEASE
            || phase == Phase.TRACK_FLIGHT
            || phase == Phase.STOP_SETTLE;
    boolean ballLoaded = phase == Phase.SETTLE_START || phase == Phase.WAIT_FOR_RELEASE;

    if (superstructure != null) {
      superstructure.setSimBallPresentOverride(ballLoaded);
    }

    controllerSim.setAxisCount(6);
    controllerSim.setButtonCount(10);
    controllerSim.setPOVCount(1);
    DriverStationSim.setJoystickType(0, GenericHID.HIDType.kXInputGamepad.value);
    DriverStationSim.setJoystickIsXbox(0, true);
    DriverStationSim.setJoystickName(0, "Season2026 Shot Validation Xbox");
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(enabled);

    controllerSim.setLeftY(0.0);
    controllerSim.setLeftX(0.0);
    controllerSim.setRightX(0.0);
    controllerSim.setRightY(0.0);
    controllerSim.setLeftTriggerAxis(0.0);
    controllerSim.setRightTriggerAxis(0.0);
    controllerSim.setAButton(false);
    controllerSim.setBButton(false);
    controllerSim.setXButton(shootHeld);
    controllerSim.setYButton(false);
    controllerSim.setBackButton(false);
    controllerSim.setStartButton(false);
    controllerSim.setLeftBumperButton(false);
    controllerSim.setRightBumperButton(false);
    controllerSim.setPOV(-1);
    controllerSim.notifyNewData();
  }

  @Override
  public void captureStepAndMaybeExit() {
    if (wroteTrace) {
      return;
    }
    ensureInitialized();

    ShotTelemetrySnapshot shotTelemetry = shotTelemetry();
    if (suppressedCaptureCycles > 0) {
      suppressedCaptureCycles--;
      elapsedSimulationSeconds += SIM_LOOP_PERIOD_SECONDS;
      return;
    }

    trace.add(capture(elapsedSimulationSeconds, shotTelemetry));
    advanceState(shotTelemetry);
    elapsedSimulationSeconds += SIM_LOOP_PERIOD_SECONDS;
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
    DriverStationSim.setJoystickName(0, "Season2026 Shot Validation Xbox");
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    initialized = true;
    RobotLogging.info("[SHOT VALIDATION] Enabled unattended shot-validation script.");
    enterCase(0);
  }

  private void enterCase(int caseIndex) {
    enterCase(caseIndex, 0);
  }

  private void enterCase(int caseIndex, int attempt) {
    activeCaseIndex = caseIndex;
    activeCaseAttempt = attempt;
    activeCase = CASES.get(caseIndex);
    phase = Phase.SETTLE_START;
    phaseStartSeconds = elapsedSimulationSeconds;
    DriverStationSim.setEnabled(false);
    controllerSim.setXButton(false);
    controllerSim.notifyNewData();
    if (superstructure != null) {
      superstructure.setSimBallPresentOverride(true);
      superstructure.setShootEnabled(false);
    }
    simulationReset.accept(activeCase.startPose());
    ShotTelemetrySnapshot shotTelemetry = shotTelemetry();
    caseStartReleaseCount = shotTelemetry.releaseCount();
    caseStartCompletedCount = shotTelemetry.completedCount();
    suppressedCaptureCycles = RESET_SETTLE_CAPTURE_CYCLES;
    RobotLogging.info(
        "[SHOT VALIDATION] case="
            + activeCase.name()
            + " attempt="
            + activeCaseAttempt
            + " startPose="
            + activeCase.startPose());
  }

  private void advanceState(ShotTelemetrySnapshot shotTelemetry) {
    double phaseElapsed = elapsedSimulationSeconds - phaseStartSeconds;
    switch (phase) {
      case SETTLE_START -> {
        if (phaseElapsed >= activeCase.settleSeconds()) {
          enterPhase(Phase.WAIT_FOR_RELEASE);
        }
      }
      case WAIT_FOR_RELEASE -> {
        if (shotTelemetry.releaseCount() > caseStartReleaseCount) {
          if (superstructure != null) {
            superstructure.setSimBallPresentOverride(false);
          }
          enterPhase(Phase.TRACK_FLIGHT);
        } else if (phaseElapsed >= activeCase.releaseTimeoutSeconds()) {
          if (activeCaseAttempt < MAX_RELEASE_RETRIES) {
            RobotLogging.warn(
                "[SHOT VALIDATION] release timeout for "
                    + activeCase.name()
                    + "; retrying same case once");
            enterCase(activeCaseIndex, activeCaseAttempt + 1);
            return;
          }
          results.add(
              CaseResult.timeout(
                  activeCase.name(),
                  activeCase.startPose(),
                  phase,
                  "release edge was not observed before timeout",
                  latestRecord(shotTelemetry)));
          enterPhase(Phase.STOP_SETTLE);
        }
      }
      case TRACK_FLIGHT -> {
        if (shotTelemetry.completedCount() > caseStartCompletedCount
            && shotTelemetry.lastCompletedShot() != null) {
          results.add(summarizeCase(activeCase, shotTelemetry.lastCompletedShot(), shotTelemetry));
          enterPhase(Phase.STOP_SETTLE);
        } else if (phaseElapsed >= activeCase.flightTimeoutSeconds()) {
          results.add(
              CaseResult.timeout(
                  activeCase.name(),
                  activeCase.startPose(),
                  phase,
                  "projectile impact was not observed before timeout",
                  latestRecord(shotTelemetry)));
          enterPhase(Phase.STOP_SETTLE);
        }
      }
      case STOP_SETTLE -> {
        if (phaseElapsed >= activeCase.stopSettleSeconds()) {
          if (activeCaseIndex + 1 < CASES.size()) {
            enterCase(activeCaseIndex + 1);
          } else {
            writeTraceAndExit();
          }
        }
      }
      case COMPLETE -> {}
    }
  }

  private void enterPhase(Phase nextPhase) {
    phase = nextPhase;
    phaseStartSeconds = elapsedSimulationSeconds;
    if (nextPhase != Phase.WAIT_FOR_RELEASE && superstructure != null) {
      superstructure.setShootEnabled(false);
    }
  }

  private TraceSample capture(double elapsedSeconds, ShotTelemetrySnapshot shotTelemetry) {
    Pose2d pose = drive != null ? drive.getPose() : null;
    SuperstructureOutcome outcome =
        superstructure != null ? superstructure.getLatestOutcome() : null;
    RenderTelemetry renderTelemetry =
        renderTelemetrySupplier == null ? null : renderTelemetrySupplier.get();
    ShotTelemetryRecord record = latestRecord(shotTelemetry);
    return new TraceSample(
        elapsedSeconds,
        activeCase == null ? "" : activeCase.name(),
        activeCaseAttempt,
        phase,
        pose == null ? Double.NaN : pose.getX(),
        pose == null ? Double.NaN : pose.getY(),
        pose == null ? Double.NaN : pose.getRotation().getRadians(),
        superstructure != null && superstructure.isShootEnabled(),
        superstructure != null && superstructure.hasBall(),
        outcome == null ? "" : outcome.state().name(),
        outcome == null ? "" : outcome.requestedState().name(),
        outcome == null ? "" : outcome.indexerGoal().name(),
        outcome == null ? Double.NaN : outcome.shooterTargetVelocityRpm(),
        superstructure != null && superstructure.getRollers().shooter != null
            ? superstructure.getRollers().shooter.getVelocityRpm()
            : Double.NaN,
        superstructure != null
            && superstructure.getRollers().shooter != null
            && superstructure.getRollers().shooter.isAtGoal(),
        outcome == null ? Double.NaN : outcome.shooterPivotPosition(),
        superstructure != null && superstructure.isShootReadyLatched(),
        superstructure != null && superstructure.isTurretReadyForFeed(),
        shotTelemetry.releaseCount(),
        shotTelemetry.completedCount(),
        record == null ? -1 : record.projectileId(),
        record == null ? Double.NaN : record.launchSpeedMetersPerSecond(),
        record == null ? Double.NaN : Math.toDegrees(record.launchAngleRadians()),
        record == null ? Double.NaN : record.apexHeightMeters(),
        record == null || !record.completed() ? Double.NaN : record.timeOfFlightSeconds(),
        renderTelemetry != null && renderTelemetry.usingAuthoritativeRobotPose());
  }

  private CaseResult summarizeCase(
      ShotCase shotCase, ShotTelemetryRecord shot, ShotTelemetrySnapshot telemetry) {
    ShotAssessment assessment = assessShot(shot);
    return new CaseResult(
        shotCase.name(),
        shotCase.startPose(),
        true,
        "completed",
        shot,
        assessment,
        telemetry.releaseCount(),
        telemetry.completedCount());
  }

  private static ShotAssessment assessShot(ShotTelemetryRecord shot) {
    List<ShotTelemetrySample> samples = shot.observedArcSamples();
    Translation3d releaseVelocity = shot.releaseVelocityMetersPerSecond();
    double horizontalSpeed = Math.hypot(releaseVelocity.getX(), releaseVelocity.getY());
    double dirX = horizontalSpeed > 1e-9 ? releaseVelocity.getX() / horizontalSpeed : Double.NaN;
    double dirY = horizontalSpeed > 1e-9 ? releaseVelocity.getY() / horizontalSpeed : Double.NaN;

    boolean forwardMonotonic = true;
    double previousForward = Double.NEGATIVE_INFINITY;
    for (ShotTelemetrySample sample : samples) {
      double forward =
          ((sample.xMeters() - shot.releasePose().getX()) * dirX)
              + ((sample.yMeters() - shot.releasePose().getY()) * dirY);
      if (Double.isFinite(previousForward) && forward + 1e-4 < previousForward) {
        forwardMonotonic = false;
      }
      previousForward = Math.max(previousForward, forward);
    }

    int apexIndex = apexIndex(samples);
    boolean clearApexThenDescent =
        apexIndex > 0
            && apexIndex < samples.size() - 1
            && samples.get(apexIndex).zMeters() - samples.get(0).zMeters() >= MIN_APEX_RISE_METERS
            && samples.get(apexIndex).zMeters() - samples.get(samples.size() - 1).zMeters()
                >= MIN_APEX_RISE_METERS;

    double averageVerticalAcceleration = averageVerticalAcceleration(samples);
    boolean gravityPlausible =
        averageVerticalAcceleration <= -8.0 && averageVerticalAcceleration >= -11.8;
    boolean timeOfFlightPlausible =
        shot.completed() && shot.timeOfFlightSeconds() > 0.15 && shot.timeOfFlightSeconds() <= 5.0;
    double forwardDistanceMeters =
        samples.isEmpty()
            ? Double.NaN
            : ((samples.get(samples.size() - 1).xMeters() - shot.releasePose().getX()) * dirX)
                + ((samples.get(samples.size() - 1).yMeters() - shot.releasePose().getY()) * dirY);
    boolean impactPlausible =
        shot.completed()
            && shot.impactPose() != null
            && shot.impactPose().getZ() <= 1e-6
            && forwardDistanceMeters >= MIN_FORWARD_DISTANCE_METERS;
    double maxObservedStepMeters = maxObservedStepMeters(samples);
    boolean noDiscontinuousTrajectory =
        maxObservedStepMeters <= maxExpectedStepMeters(samples) + MAX_SAMPLE_STEP_EXTRA_METERS;
    double predictedObservedMaxErrorMeters = predictedObservedMaxErrorMeters(shot);
    boolean predictedPathMatchesObservedProjectilePath =
        Double.isFinite(predictedObservedMaxErrorMeters)
            && predictedObservedMaxErrorMeters <= MAX_PREDICTED_OBSERVED_PATH_ERROR_METERS;
    boolean physicallyBelievable =
        forwardMonotonic
            && clearApexThenDescent
            && gravityPlausible
            && timeOfFlightPlausible
            && impactPlausible
            && noDiscontinuousTrajectory;
    boolean scoringArcValid = shot.clearsTop() && shot.descendsIntoBottom();

    return new ShotAssessment(
        forwardMonotonic,
        clearApexThenDescent,
        gravityPlausible,
        averageVerticalAcceleration,
        timeOfFlightPlausible,
        impactPlausible,
        forwardDistanceMeters,
        noDiscontinuousTrajectory,
        maxObservedStepMeters,
        true,
        predictedObservedMaxErrorMeters,
        predictedPathMatchesObservedProjectilePath,
        "not_modeled",
        physicallyBelievable,
        shot.clearsTop(),
        shot.descendsIntoBottom(),
        shot.topClearanceMeters(),
        shot.bottomEntryErrorMeters(),
        shot.topClearancePose(),
        shot.bottomEntryPose(),
        scoringArcValid);
  }

  private static int apexIndex(List<ShotTelemetrySample> samples) {
    if (samples == null || samples.isEmpty()) {
      return -1;
    }
    int apexIndex = 0;
    double maxZ = Double.NEGATIVE_INFINITY;
    for (int i = 0; i < samples.size(); i++) {
      if (samples.get(i).zMeters() > maxZ) {
        maxZ = samples.get(i).zMeters();
        apexIndex = i;
      }
    }
    return apexIndex;
  }

  private static double averageVerticalAcceleration(List<ShotTelemetrySample> samples) {
    double sum = 0.0;
    int count = 0;
    for (int i = 1; i < samples.size(); i++) {
      ShotTelemetrySample previous = samples.get(i - 1);
      ShotTelemetrySample current = samples.get(i);
      double dt = current.elapsedSeconds() - previous.elapsedSeconds();
      if (dt <= 1e-9
          || !Double.isFinite(previous.vzMetersPerSecond())
          || !Double.isFinite(current.vzMetersPerSecond())) {
        continue;
      }
      sum += (current.vzMetersPerSecond() - previous.vzMetersPerSecond()) / dt;
      count++;
    }
    return count == 0 ? Double.NaN : sum / count;
  }

  private static double maxObservedStepMeters(List<ShotTelemetrySample> samples) {
    double maxStep = 0.0;
    for (int i = 1; i < samples.size(); i++) {
      maxStep = Math.max(maxStep, distance(samples.get(i - 1), samples.get(i)));
    }
    return maxStep;
  }

  private static double maxExpectedStepMeters(List<ShotTelemetrySample> samples) {
    double maxExpected = 0.0;
    for (int i = 1; i < samples.size(); i++) {
      ShotTelemetrySample previous = samples.get(i - 1);
      ShotTelemetrySample current = samples.get(i);
      double dt = current.elapsedSeconds() - previous.elapsedSeconds();
      double speed = Math.max(previous.speedMetersPerSecond(), current.speedMetersPerSecond());
      if (dt > 0.0 && Double.isFinite(speed)) {
        maxExpected = Math.max(maxExpected, speed * dt);
      }
    }
    return maxExpected;
  }

  private static double predictedObservedMaxErrorMeters(ShotTelemetryRecord shot) {
    if (shot.predictedArcSamples().isEmpty() || shot.observedArcSamples().isEmpty()) {
      return Double.NaN;
    }
    double maxError = 0.0;
    for (ShotTelemetrySample predicted : shot.predictedArcSamples()) {
      ShotTelemetrySample nearest =
          shot.observedArcSamples().stream()
              .min(
                  Comparator.comparingDouble(
                      observed -> Math.abs(observed.elapsedSeconds() - predicted.elapsedSeconds())))
              .orElse(null);
      if (nearest != null) {
        maxError = Math.max(maxError, distance(predicted, nearest));
      }
    }
    return maxError;
  }

  private static double distance(ShotTelemetrySample a, ShotTelemetrySample b) {
    return Math.sqrt(
        Math.pow(a.xMeters() - b.xMeters(), 2.0)
            + Math.pow(a.yMeters() - b.yMeters(), 2.0)
            + Math.pow(a.zMeters() - b.zMeters(), 2.0));
  }

  private ShotTelemetryRecord latestRecord(ShotTelemetrySnapshot telemetry) {
    if (telemetry.lastCompletedShot() != null) {
      return telemetry.lastCompletedShot();
    }
    if (telemetry.activeShot() != null) {
      return telemetry.activeShot();
    }
    return telemetry.lastReleasedShot();
  }

  private ShotTelemetrySnapshot shotTelemetry() {
    return shotTelemetrySupplier == null
        ? new ShotTelemetrySnapshot(0, 0, null, null, null, List.of())
        : shotTelemetrySupplier.get();
  }

  private void writeTraceAndExit() {
    if (wroteTrace) {
      return;
    }
    wroteTrace = true;
    if (superstructure != null) {
      superstructure.setSimBallPresentOverride(false);
      superstructure.setShootEnabled(false);
    }
    DriverStationSim.setEnabled(false);
    controllerSim.setXButton(false);
    controllerSim.notifyNewData();

    ShotValidationSummary summary = summarize(results);
    TraceReport report = new TraceReport(trace, results, summary);
    try {
      Files.createDirectories(outputPath.getParent());
      JSON.writeValue(outputPath.toFile(), report);
      RobotLogging.info("[SHOT VALIDATION] Trace written to " + outputPath.toAbsolutePath());
      for (CaseResult result : results) {
        RobotLogging.info("[SHOT VALIDATION] " + result.auditLine());
      }
      RobotLogging.info("[SHOT VALIDATION] overall=" + summary.status());
    } catch (IOException e) {
      RobotLogging.error("[SHOT VALIDATION] Failed to write trace", e);
    }
    HAL.exitMain();
  }

  private static ShotValidationSummary summarize(List<CaseResult> results) {
    long completed = results.stream().filter(CaseResult::completed).count();
    boolean allBelievable =
        results.size() == CASES.size()
            && completed == CASES.size()
            && results.stream()
                .allMatch(
                    result ->
                        result.assessment() != null
                            && result.assessment().physicallyBelievable()
                            && result.assessment().scoringArcValid());
    String status =
        allBelievable ? "COMPLETE_PHYSICALLY_BELIEVABLE_AND_SCORING_VALID" : "PARTIAL_OR_ISSUE";
    return new ShotValidationSummary(status, completed, CASES.size());
  }

  private record ShotCase(
      String name,
      Pose2d startPose,
      double settleSeconds,
      double releaseTimeoutSeconds,
      double flightTimeoutSeconds,
      double stopSettleSeconds) {}

  public enum Phase {
    SETTLE_START,
    WAIT_FOR_RELEASE,
    TRACK_FLIGHT,
    STOP_SETTLE,
    COMPLETE
  }

  public record TraceSample(
      double elapsedSeconds,
      String caseName,
      int attempt,
      Phase phase,
      double poseX,
      double poseY,
      double headingRadians,
      boolean shootEnabled,
      boolean hasBall,
      String superState,
      String requestedState,
      String indexerGoal,
      double shooterTargetVelocityRpm,
      double shooterVelocityRpm,
      boolean shooterAtGoal,
      double shooterPivotPosition,
      boolean shootReadyLatched,
      boolean turretReadyForFeed,
      int releaseCount,
      int completedCount,
      int activeProjectileId,
      double launchSpeedMetersPerSecond,
      double launchAngleDegrees,
      double apexHeightMeters,
      double timeOfFlightSeconds,
      boolean renderUsesAuthoritativeRobotPose) {}

  public record CaseResult(
      String caseName,
      Pose2d startPose,
      boolean completed,
      String status,
      ShotTelemetryRecord shot,
      ShotAssessment assessment,
      int releaseCount,
      int completedCount) {
    static CaseResult timeout(
        String caseName, Pose2d startPose, Phase phase, String reason, ShotTelemetryRecord shot) {
      return new CaseResult(
          caseName, startPose, false, phase.name() + ": " + reason, shot, null, 0, 0);
    }

    String auditLine() {
      return String.format(
          Locale.ROOT,
          "case=%s completed=%s status=%s launchSpeed=%.3f launchAngleDeg=%.2f tof=%.3f believable=%s",
          caseName,
          completed,
          status,
          shot == null ? Double.NaN : shot.launchSpeedMetersPerSecond(),
          shot == null ? Double.NaN : Math.toDegrees(shot.launchAngleRadians()),
          shot == null ? Double.NaN : shot.timeOfFlightSeconds(),
          assessment != null && assessment.physicallyBelievable());
    }
  }

  public record ShotAssessment(
      boolean forwardMonotonic,
      boolean clearApexThenDescent,
      boolean gravityPlausible,
      double averageVerticalAccelerationMetersPerSecondSq,
      boolean timeOfFlightPlausible,
      boolean impactPlausible,
      double forwardDistanceMeters,
      boolean noDiscontinuousTrajectory,
      double maxObservedStepMeters,
      boolean observedRenderPathUsesSimulatedProjectilePath,
      double predictedObservedMaxErrorMeters,
      boolean predictedPathMatchesObservedProjectilePath,
      String dragAssessment,
      boolean physicallyBelievable,
      boolean clearsTop,
      boolean descendsIntoBottom,
      double topClearanceMeters,
      double bottomEntryErrorMeters,
      Pose3d topClearancePose,
      Pose3d bottomEntryPose,
      boolean scoringArcValid) {}

  public record ShotValidationSummary(String status, long completedCases, int expectedCases) {}

  public record TraceReport(
      List<TraceSample> trace, List<CaseResult> cases, ShotValidationSummary summary) {}
}
