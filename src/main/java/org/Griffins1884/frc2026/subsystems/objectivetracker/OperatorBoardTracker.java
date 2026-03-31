package org.Griffins1884.frc2026.subsystems.objectivetracker;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.Griffins1884.frc2026.Config;
import org.Griffins1884.frc2026.mechanisms.MechanismHealth;
import org.Griffins1884.frc2026.runtime.RuntimeModeManager;
import org.Griffins1884.frc2026.runtime.RuntimeModeProfile;
import org.Griffins1884.frc2026.runtime.RuntimeProfileCodec;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.Superstructure.SuperState;
import org.Griffins1884.frc2026.subsystems.groups.Arms;
import org.Griffins1884.frc2026.subsystems.groups.Rollers;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.Griffins1884.frc2026.subsystems.vision.Vision;
import org.Griffins1884.frc2026.util.HubShiftTracker;
import org.Griffins1884.frc2026.util.LogRollover;
import org.littletonrobotics.junction.Logger;

public class OperatorBoardTracker extends SubsystemBase implements AutoCloseable {
  private static final double TELEMETRY_PUBLISH_PERIOD_SEC = 0.05;
  private static final double DISABLED_TELEMETRY_PUBLISH_PERIOD_SEC = 0.10;
  private static final ObjectMapper JSON = new ObjectMapper();

  private final OperatorBoardIO io;
  private final OperatorBoardIOInputsAutoLogged inputs = new OperatorBoardIOInputsAutoLogged();
  private final Superstructure superstructure;
  private final SwerveSubsystem drive;
  private final TurretSubsystem turret;
  private final Vision vision;
  private final OperatorBoardWebServer webServer;
  private final RebuiltSpotLibrary spotLibrary;
  private final RebuiltAutoQueue autoQueue;
  private final DeployAutoLibrary deployAutoLibrary;
  private final OperatorBoardPersistence persistence;
  private final OperatorBoardDiagnosticBundleWriter diagnosticBundleWriter;

  private String lastRequestedState = "";
  private boolean lastRequestAccepted = true;
  private String lastRequestReason = "";
  private String lastRuntimeProfileSpecJson =
      RuntimeProfileCodec.toJson(RuntimeModeManager.getActiveProfile());
  private String lastRuntimeProfileStatus = "READY";
  private String lastSubsystemDescriptionsJson = "{}";
  private OperatorBoardDiagnosticBundleWriter.OperatorBoardDiagnosticSnapshot
      lastDiagnosticSnapshot;
  private double lastTelemetryPublishTimestampSec = Double.NEGATIVE_INFINITY;
  private ActionTraceState lastActionTraceState =
      new ActionTraceState(
          Timer.getFPGATimestamp(),
          "SYSTEM",
          "OPERATOR_BOARD_INIT",
          "pass",
          "Operator board tracker initialized.",
          "UNKNOWN",
          "UNKNOWN");
  private double lastQueueActionTimestampSec = Double.NEGATIVE_INFINITY;
  private final Map<String, PublishTopicAccumulator> sampledPublishTopics = new HashMap<>();
  private final Map<String, String> lastPublishedStringTopics = new HashMap<>();
  private double sampledPublishWindowStartSec = Timer.getFPGATimestamp();
  private int sampledPublishWindowCount = 0;
  private int sampledPublishWindowBytes = 0;
  private double sampledPublishRateHz = 0.0;
  private double sampledPublishBytesPerSec = 0.0;
  private double sampledPublishPeakRateHz = 0.0;
  private double sampledPublishPeakBytesPerSec = 0.0;

  public OperatorBoardTracker(OperatorBoardIO io, Superstructure superstructure) {
    this(io, superstructure, null, null, null);
  }

  public OperatorBoardTracker(
      OperatorBoardIO io, Superstructure superstructure, SwerveSubsystem drive) {
    this(io, superstructure, drive, null, null);
  }

  public OperatorBoardTracker(
      OperatorBoardIO io,
      Superstructure superstructure,
      SwerveSubsystem drive,
      TurretSubsystem turret) {
    this(io, superstructure, drive, turret, null);
  }

  public OperatorBoardTracker(
      OperatorBoardIO io,
      Superstructure superstructure,
      SwerveSubsystem drive,
      TurretSubsystem turret,
      Vision vision) {
    this.io = Objects.requireNonNull(io, "io");
    this.superstructure = superstructure;
    this.drive = drive;
    this.turret = turret;
    this.vision = vision;
    this.persistence = new OperatorBoardPersistence();
    this.persistence.initialize();
    this.diagnosticBundleWriter = new OperatorBoardDiagnosticBundleWriter(persistence);
    this.lastSubsystemDescriptionsJson = writeJson(persistence.readSubsystemDescriptions());
    this.spotLibrary = new RebuiltSpotLibrary();
    this.deployAutoLibrary =
        new DeployAutoLibrary(
            Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos"));
    this.autoQueue = new RebuiltAutoQueue(spotLibrary, superstructure, drive, deployAutoLibrary);
    this.webServer = maybeStartWebServer();
    if (superstructure != null) {
      lastRequestedState = superstructure.getRequestedState().name();
    }
  }

  private OperatorBoardWebServer maybeStartWebServer() {
    if (!Config.WebUIConfig.ENABLED) {
      return null;
    }
    try {
      Path deployDir = Filesystem.getDeployDirectory().toPath();
      OperatorBoardWebServer server =
          new OperatorBoardWebServer(
              deployDir.resolve("operatorboard"),
              deployAutoLibrary,
              deployDir.resolve("music"),
              Config.WebUIConfig.BIND_ADDRESS,
              Config.WebUIConfig.PORT);
      server.start();
      return server;
    } catch (IOException ex) {
      DriverStation.reportError("Failed to start operator board server", ex.getStackTrace());
      return null;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("OperatorBoard", inputs);

    handleStateRequests();
    autoQueue.handleInputs(inputs);
    syncQueueActionTrace();
    autoQueue.periodic();
    autoQueue.logState();
    if (shouldPublishTelemetry()) {
      publishTelemetry();
    }
  }

  private void handleStateRequests() {
    if (inputs.playSwerveMusicRequested) {
      if (drive != null) {
        drive.playSwerveMusic();
        lastRequestedState = "SWERVE_MUSIC_PLAY";
        lastRequestAccepted = true;
        lastRequestReason = "";
        recordActionTrace("UTILITY", "SWERVE_MUSIC_PLAY", true, "Drive music playback started.");
      } else {
        lastRequestedState = "SWERVE_MUSIC_PLAY";
        lastRequestAccepted = false;
        lastRequestReason = "Drive unavailable";
        recordActionTrace("UTILITY", "SWERVE_MUSIC_PLAY", false, lastRequestReason);
      }
    }
    if (inputs.stopSwerveMusicRequested) {
      if (drive != null) {
        drive.stopSwerveMusic();
        lastRequestedState = "SWERVE_MUSIC_STOP";
        lastRequestAccepted = true;
        lastRequestReason = "";
        recordActionTrace("UTILITY", "SWERVE_MUSIC_STOP", true, "Drive music playback stopped.");
      } else {
        lastRequestedState = "SWERVE_MUSIC_STOP";
        lastRequestAccepted = false;
        lastRequestReason = "Drive unavailable";
        recordActionTrace("UTILITY", "SWERVE_MUSIC_STOP", false, lastRequestReason);
      }
    }
    if (Double.isFinite(inputs.swerveMusicVolume)) {
      if (drive != null) {
        drive.setSwerveMusicVolume(inputs.swerveMusicVolume);
      }
    }
    if (inputs.rollLogsRequested) {
      boolean rolled = LogRollover.roll();
      lastRequestedState = "ROLL_LOGS";
      lastRequestAccepted = rolled;
      lastRequestReason = rolled ? "" : "Log rollover " + LogRollover.getStatus().toLowerCase();
      recordActionTrace(
          "UTILITY", "ROLL_LOGS", rolled, rolled ? "Log rollover completed." : lastRequestReason);
    }
    if (inputs.cleanLogsRequested) {
      boolean cleaned = LogRollover.cleanLogsFolder();
      lastRequestedState = "CLEAN_LOGS";
      lastRequestAccepted = cleaned;
      lastRequestReason =
          cleaned ? "" : "Log cleanup " + LogRollover.getCleanStatus().toLowerCase();
      recordActionTrace(
          "UTILITY", "CLEAN_LOGS", cleaned, cleaned ? "Log cleanup completed." : lastRequestReason);
    }
    if (inputs.requestIntakeDeployRezero) {
      if (superstructure == null) {
        lastRequestedState = "INTAKE_DEPLOY_REZERO_REQUEST";
        lastRequestAccepted = false;
        lastRequestReason = "Superstructure unavailable";
        recordActionTrace("INTAKE_PIVOT", "INTAKE_DEPLOY_REZERO_REQUEST", false, lastRequestReason);
      } else {
        superstructure.requestIntakeDeployRezero();
        lastRequestedState = "INTAKE_DEPLOY_REZERO_REQUEST";
        lastRequestAccepted = true;
        lastRequestReason = "";
        recordActionTrace(
            "INTAKE_PIVOT",
            "INTAKE_DEPLOY_REZERO_REQUEST",
            true,
            "Intake deploy rezero requested.");
      }
    }
    if (inputs.cancelIntakeDeployRezero) {
      if (superstructure == null) {
        lastRequestedState = "INTAKE_DEPLOY_REZERO_CANCEL";
        lastRequestAccepted = false;
        lastRequestReason = "Superstructure unavailable";
        recordActionTrace("INTAKE_PIVOT", "INTAKE_DEPLOY_REZERO_CANCEL", false, lastRequestReason);
      } else {
        superstructure.cancelIntakeDeployRezero();
        lastRequestedState = "INTAKE_DEPLOY_REZERO_CANCEL";
        lastRequestAccepted = true;
        lastRequestReason = "";
        recordActionTrace(
            "INTAKE_PIVOT", "INTAKE_DEPLOY_REZERO_CANCEL", true, "Intake deploy rezero cancelled.");
      }
    }
    if (inputs.requestManualIntakeDeployZeroSeek) {
      if (superstructure == null) {
        lastRequestedState = "INTAKE_DEPLOY_MANUAL_ZERO_SEEK_REQUEST";
        lastRequestAccepted = false;
        lastRequestReason = "Superstructure unavailable";
        recordActionTrace(
            "INTAKE_PIVOT", "INTAKE_DEPLOY_MANUAL_ZERO_SEEK_REQUEST", false, lastRequestReason);
      } else {
        superstructure.requestManualIntakeDeployZeroSeek();
        lastRequestedState = "INTAKE_DEPLOY_MANUAL_ZERO_SEEK_REQUEST";
        lastRequestAccepted = true;
        lastRequestReason = "";
        recordActionTrace(
            "INTAKE_PIVOT",
            "INTAKE_DEPLOY_MANUAL_ZERO_SEEK_REQUEST",
            true,
            "Manual intake zero seek requested.");
      }
    }
    if (inputs.cancelManualIntakeDeployZeroSeek) {
      if (superstructure == null) {
        lastRequestedState = "INTAKE_DEPLOY_MANUAL_ZERO_SEEK_CANCEL";
        lastRequestAccepted = false;
        lastRequestReason = "Superstructure unavailable";
        recordActionTrace(
            "INTAKE_PIVOT", "INTAKE_DEPLOY_MANUAL_ZERO_SEEK_CANCEL", false, lastRequestReason);
      } else {
        superstructure.cancelManualIntakeDeployZeroSeek();
        lastRequestedState = "INTAKE_DEPLOY_MANUAL_ZERO_SEEK_CANCEL";
        lastRequestAccepted = true;
        lastRequestReason = "";
        recordActionTrace(
            "INTAKE_PIVOT",
            "INTAKE_DEPLOY_MANUAL_ZERO_SEEK_CANCEL",
            true,
            "Manual intake zero seek cancelled.");
      }
    }
    if (inputs.runtimeProfileSpec.length > 0) {
      lastRuntimeProfileSpecJson = inputs.runtimeProfileSpec[0];
      lastRuntimeProfileStatus = "SPEC_RECEIVED";
      recordActionTrace("RUNTIME", "RUNTIME_PROFILE_SPEC", true, "Runtime profile spec staged.");
    }
    if (inputs.resetRuntimeProfile) {
      RuntimeModeManager.resetToDefaults();
      lastRuntimeProfileSpecJson =
          RuntimeProfileCodec.toJson(RuntimeModeManager.getActiveProfile());
      lastRuntimeProfileStatus = "RESET_TO_DEFAULTS";
      lastRequestedState = "RUNTIME_PROFILE_RESET";
      lastRequestAccepted = true;
      lastRequestReason = "";
      recordActionTrace(
          "RUNTIME", "RUNTIME_PROFILE_RESET", true, "Runtime profile reset to defaults.");
    }
    if (inputs.applyRuntimeProfile) {
      try {
        RuntimeModeManager.setActiveProfile(
            RuntimeProfileCodec.fromJson(lastRuntimeProfileSpecJson));
        lastRuntimeProfileSpecJson =
            RuntimeProfileCodec.toJson(RuntimeModeManager.getActiveProfile());
        lastRuntimeProfileStatus = "APPLIED";
        lastRequestedState = "RUNTIME_PROFILE_APPLY";
        lastRequestAccepted = true;
        lastRequestReason = "";
        recordActionTrace("RUNTIME", "RUNTIME_PROFILE_APPLY", true, "Runtime profile applied.");
      } catch (Exception ex) {
        lastRuntimeProfileStatus = "INVALID_SPEC: " + ex.getMessage();
        lastRequestedState = "RUNTIME_PROFILE_APPLY";
        lastRequestAccepted = false;
        lastRequestReason = "Runtime profile invalid";
        recordActionTrace("RUNTIME", "RUNTIME_PROFILE_APPLY", false, lastRequestReason);
      }
    }
    if (inputs.autoStateEnableRequested) {
      if (superstructure == null) {
        lastRequestedState = "AUTO";
        lastRequestAccepted = false;
        lastRequestReason = "Superstructure unavailable";
        recordActionTrace("SUPERSTRUCTURE", "AUTO_STATE_ENABLE", false, lastRequestReason);
      } else {
        superstructure.setAutoStateEnabled(true);
        lastRequestedState = "AUTO";
        lastRequestAccepted = true;
        lastRequestReason = "";
        recordActionTrace(
            "SUPERSTRUCTURE", "AUTO_STATE_ENABLE", true, "Automatic state selection enabled.");
      }
      return;
    }
    if (inputs.requestedState.length == 0) {
      return;
    }
    String raw = inputs.requestedState[0];
    if (raw == null || raw.isBlank()) {
      return;
    }
    String trimmed = raw.trim();

    SuperState parsed = parseState(trimmed);
    if (parsed == null) {
      lastRequestedState = trimmed;
      lastRequestAccepted = false;
      lastRequestReason = "Unknown state: " + trimmed;
      recordActionTrace("SUPERSTRUCTURE", "STATE_REQUEST", false, lastRequestReason);
      return;
    }

    lastRequestedState = parsed.name();

    if (superstructure == null) {
      lastRequestAccepted = false;
      lastRequestReason = "Superstructure unavailable";
      recordActionTrace("SUPERSTRUCTURE", "STATE_REQUEST", false, lastRequestReason);
      return;
    }

    Superstructure.StateRequestResult result = superstructure.requestStateFromDashboard(parsed);
    lastRequestAccepted = result.accepted();
    lastRequestReason = result.reason() == null ? "" : result.reason();
    recordActionTrace(
        "SUPERSTRUCTURE",
        "STATE_REQUEST:" + parsed.name(),
        result.accepted(),
        result.accepted() ? "State request accepted." : lastRequestReason);
  }

  private SuperState parseState(String value) {
    if (value == null) {
      return null;
    }
    String token = value.trim();
    if (token.isEmpty()) {
      return null;
    }
    try {
      return SuperState.valueOf(token.toUpperCase(Locale.ROOT));
    } catch (IllegalArgumentException ex) {
      return null;
    }
  }

  private void publishTelemetry() {
    rollPublishWindow(Timer.getFPGATimestamp());
    String requestedState =
        !lastRequestedState.isBlank()
            ? lastRequestedState
            : superstructure != null ? superstructure.getRequestedState().name() : "UNKNOWN";
    if (shouldPublishString(OperatorBoardContract.REQUESTED_STATE, requestedState)) {
      trackPublish(OperatorBoardContract.REQUESTED_STATE, requestedState);
      io.setRequestedState(requestedState);
    }
    String currentState =
        superstructure != null ? superstructure.getCurrentState().name() : "UNKNOWN";
    if (shouldPublishString(OperatorBoardContract.CURRENT_STATE, currentState)) {
      trackPublish(OperatorBoardContract.CURRENT_STATE, currentState);
      io.setCurrentState(currentState);
    }
    trackPublish(OperatorBoardContract.REQUEST_ACCEPTED, lastRequestAccepted);
    io.setRequestAccepted(lastRequestAccepted);
    if (shouldPublishString(OperatorBoardContract.REQUEST_REASON, lastRequestReason)) {
      trackPublish(OperatorBoardContract.REQUEST_REASON, lastRequestReason);
      io.setRequestReason(lastRequestReason);
    }

    TargetSnapshot target = computeTargetSnapshot();
    if (shouldPublishString(OperatorBoardContract.TARGET_TYPE, target.type())) {
      trackPublish(OperatorBoardContract.TARGET_TYPE, target.type());
      io.setTargetType(target.type());
    }
    trackPublish(OperatorBoardContract.TARGET_POSE, target.pose());
    io.setTargetPose(target.pose());
    trackPublish(OperatorBoardContract.TARGET_POSE_VALID, target.valid());
    io.setTargetPoseValid(target.valid());

    double[] robotPose = computeRobotPose();
    trackPublish(OperatorBoardContract.ROBOT_POSE, robotPose);
    io.setRobotPose(robotPose);
    Optional<Pose2d> queuePreviewPose = autoQueue.getPreviewPose();
    String autoQueueStateJson = autoQueue.getQueueStateJson();
    String selectedAutoStateJson = autoQueue.getSelectedAutoStateJson();
    double[] previewPoseArray =
        queuePreviewPose.map(OperatorBoardTracker::toPoseArray).orElseGet(() -> new double[] {});
    String runtimeProfileStateJson =
        RuntimeProfileCodec.toJson(RuntimeModeManager.getActiveProfile());
    String systemCheckStateJson = buildSystemCheckStateJson(queuePreviewPose);
    String autoCheckStateJson = buildAutoCheckStateJson(queuePreviewPose);
    String autoQuickRunStateJson = autoQueue.getQuickRunStateJson();
    String mechanismStatusStateJson = buildMechanismStatusStateJson();
    String actionTraceStateJson = writeJson(lastActionTraceState);

    if (shouldPublishString(OperatorBoardContract.AUTO_QUEUE_STATE, autoQueueStateJson)) {
      trackPublish(OperatorBoardContract.AUTO_QUEUE_STATE, autoQueueStateJson);
      io.setAutoQueueState(autoQueueStateJson);
    }
    trackPublish(OperatorBoardContract.AUTO_QUEUE_PREVIEW_POSE, previewPoseArray);
    io.setAutoQueuePreviewPose(previewPoseArray);
    trackPublish(OperatorBoardContract.AUTO_QUEUE_PREVIEW_POSE_VALID, queuePreviewPose.isPresent());
    io.setAutoQueuePreviewPoseValid(queuePreviewPose.isPresent());
    if (shouldPublishString(OperatorBoardContract.SELECTED_AUTO_STATE, selectedAutoStateJson)) {
      trackPublish(OperatorBoardContract.SELECTED_AUTO_STATE, selectedAutoStateJson);
      io.setSelectedAutoState(selectedAutoStateJson);
    }
    if (shouldPublishString(OperatorBoardContract.RUNTIME_PROFILE_STATE, runtimeProfileStateJson)) {
      trackPublish(OperatorBoardContract.RUNTIME_PROFILE_STATE, runtimeProfileStateJson);
      io.setRuntimeProfileState(runtimeProfileStateJson);
    }
    if (shouldPublishString(
        OperatorBoardContract.RUNTIME_PROFILE_STATUS, lastRuntimeProfileStatus)) {
      trackPublish(OperatorBoardContract.RUNTIME_PROFILE_STATUS, lastRuntimeProfileStatus);
      io.setRuntimeProfileStatus(lastRuntimeProfileStatus);
    }
    if (shouldPublishString(OperatorBoardContract.SYSTEM_CHECK_STATE, systemCheckStateJson)) {
      trackPublish(OperatorBoardContract.SYSTEM_CHECK_STATE, systemCheckStateJson);
      io.setSystemCheckState(systemCheckStateJson);
    }
    if (shouldPublishString(OperatorBoardContract.AUTO_CHECK_STATE, autoCheckStateJson)) {
      trackPublish(OperatorBoardContract.AUTO_CHECK_STATE, autoCheckStateJson);
      io.setAutoCheckState(autoCheckStateJson);
    }
    if (shouldPublishString(OperatorBoardContract.AUTO_QUICK_RUN_STATE, autoQuickRunStateJson)) {
      trackPublish(OperatorBoardContract.AUTO_QUICK_RUN_STATE, autoQuickRunStateJson);
      io.setAutoQuickRunState(autoQuickRunStateJson);
    }
    if (shouldPublishString(
        OperatorBoardContract.MECHANISM_STATUS_STATE, mechanismStatusStateJson)) {
      trackPublish(OperatorBoardContract.MECHANISM_STATUS_STATE, mechanismStatusStateJson);
      io.setMechanismStatusState(mechanismStatusStateJson);
    }
    if (shouldPublishString(OperatorBoardContract.ACTION_TRACE_STATE, actionTraceStateJson)) {
      trackPublish(OperatorBoardContract.ACTION_TRACE_STATE, actionTraceStateJson);
      io.setActionTraceState(actionTraceStateJson);
    }

    String ntDiagnosticsStateJson = buildNtDiagnosticsStateJson();
    if (shouldPublishString(OperatorBoardContract.NT_DIAGNOSTICS_STATE, ntDiagnosticsStateJson)) {
      trackPublish(OperatorBoardContract.NT_DIAGNOSTICS_STATE, ntDiagnosticsStateJson);
      io.setNtDiagnosticsState(ntDiagnosticsStateJson);
    }
    lastDiagnosticSnapshot =
        buildDiagnosticSnapshot(
            systemCheckStateJson,
            autoCheckStateJson,
            ntDiagnosticsStateJson,
            mechanismStatusStateJson,
            actionTraceStateJson,
            runtimeProfileStateJson,
            selectedAutoStateJson,
            autoQueueStateJson);
    diagnosticBundleWriter.maybeWrite(lastDiagnosticSnapshot);

    io.setHasBall(superstructure != null && superstructure.hasBall());

    io.setDsMode(getDsMode());
    io.setBatteryVoltage(RobotController.getBatteryVoltage());
    io.setBrownout(RobotController.isBrownedOut());
    io.setAlliance(getAlliance());
    io.setMatchTime(DriverStation.getMatchTime());

    HubShiftTracker.Snapshot hubSnapshot = HubShiftTracker.fromDriverStation();
    io.setHubTimeframe(hubSnapshot.timeframe().name());
    io.setHubStatusValid(hubSnapshot.hubStatusValid());
    io.setRedHubStatus(hubSnapshot.redHubStatus().name());
    io.setBlueHubStatus(hubSnapshot.blueHubStatus().name());
    io.setOurHubStatus(hubSnapshot.ourHubStatus().name());
    io.setOurHubActive(hubSnapshot.ourHubActive());
    io.setAutoWinnerAlliance(
        hubSnapshot.autoWinner().isPresent() ? hubSnapshot.autoWinner().get().name() : "UNKNOWN");
    io.setGameDataRaw(hubSnapshot.gameDataRaw());
    io.setHubRecommendation(hubSnapshot.recommendation());

    io.setTurretAtSetpoint(turret != null && turret.isAtGoal());
    io.setTurretMode(turret != null ? turret.getControlMode().name() : "UNAVAILABLE");
    io.setSysIdDrivePhase(drive != null ? drive.getDriveSysIdPhase() : "UNAVAILABLE");
    io.setSysIdDriveActive(drive != null && drive.isDriveSysIdActive());
    io.setSysIdDriveLastCompleted(drive != null ? drive.getDriveSysIdLastCompleted() : Double.NaN);
    io.setSysIdDriveLastCompletedPhase(
        drive != null ? drive.getDriveSysIdLastCompletedPhase() : "UNAVAILABLE");
    io.setSysIdTurnPhase(drive != null ? drive.getTurnSysIdPhase() : "UNAVAILABLE");
    io.setSysIdTurnActive(drive != null && drive.isTurnSysIdActive());
    io.setSysIdTurnLastCompleted(drive != null ? drive.getTurnSysIdLastCompleted() : Double.NaN);
    io.setSysIdTurnLastCompletedPhase(
        drive != null ? drive.getTurnSysIdLastCompletedPhase() : "UNAVAILABLE");
    io.setVisionStatus(Config.Subsystems.VISION_ENABLED ? "ENABLED" : "DISABLED");
    io.setVisionPoseVisible(vision != null && vision.anyCameraHasAcceptedPose());
    io.setShootEnabled(superstructure != null && superstructure.isShootEnabled());
    io.setIntakeRollersHeld(superstructure != null && superstructure.isIntakeRollersHeld());
    io.setIntakeDeployed(superstructure != null && superstructure.isIntakeDeployed());
    io.setTeleopOverrideActive(superstructure != null && superstructure.isTeleopOverrideActive());
    io.setDriverControllerControlActive(
        superstructure != null && superstructure.isDriverControllerControlActive());
    io.setShootReadyLatched(superstructure != null && superstructure.isShootReadyLatched());
    io.setIntakeDeployRezeroInProgress(
        superstructure != null && superstructure.isIntakeDeployRezeroInProgress());
    io.setManualIntakeDeployZeroSeekInProgress(
        superstructure != null && superstructure.isManualIntakeDeployZeroSeekInProgress());
    io.setLogRollStatus(LogRollover.getStatus());
    io.setLogRollLastTimestamp(LogRollover.getLastRollTimestampSec());
    io.setLogRollCount(LogRollover.getRollCount());
    io.setLogCleanStatus(LogRollover.getCleanStatus());
    io.setLogCleanLastTimestamp(LogRollover.getLastCleanTimestampSec());
    io.setLogCleanCount(LogRollover.getCleanCount());
    io.setLogCleanDeletedEntries(LogRollover.getLastCleanDeletedEntries());
  }

  private boolean shouldPublishTelemetry() {
    double now = Timer.getFPGATimestamp();
    double publishPeriod =
        DriverStation.isDisabled()
            ? DISABLED_TELEMETRY_PUBLISH_PERIOD_SEC
            : TELEMETRY_PUBLISH_PERIOD_SEC;
    if (now - lastTelemetryPublishTimestampSec < publishPeriod) {
      return false;
    }
    lastTelemetryPublishTimestampSec = now;
    return true;
  }

  private TargetSnapshot computeTargetSnapshot() {
    if (superstructure == null) {
      return TargetSnapshot.unknown();
    }
    Translation2d target = superstructure.getCurrentTurretTarget();
    String targetType = superstructure.isInAllianceZone() ? "HUB" : "FERRY";

    if (target == null || !isFinite(target.getX()) || !isFinite(target.getY())) {
      return new TargetSnapshot(targetType, new double[] {}, false);
    }

    Pose2d pose = new Pose2d(target, new Rotation2d());
    return new TargetSnapshot(targetType, toPoseArray(pose), true);
  }

  private double[] computeRobotPose() {
    if (drive == null) {
      return new double[] {};
    }
    Pose2d pose = drive.getPose();
    if (pose == null || !isFinitePose(pose)) {
      return new double[] {};
    }
    return toPoseArray(pose);
  }

  private static double[] toPoseArray(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
  }

  private OperatorBoardDiagnosticBundleWriter.OperatorBoardDiagnosticSnapshot
      buildDiagnosticSnapshot(
          String systemCheckStateJson,
          String autoCheckStateJson,
          String ntDiagnosticsStateJson,
          String mechanismStatusStateJson,
          String actionTraceStateJson,
          String runtimeProfileStateJson,
          String selectedAutoStateJson,
          String autoQueueStateJson) {
    return new OperatorBoardDiagnosticBundleWriter.OperatorBoardDiagnosticSnapshot(
        extractJsonField(systemCheckStateJson, "status", "unknown"),
        extractJsonField(systemCheckStateJson, "summary", "UNKNOWN"),
        extractJsonField(autoCheckStateJson, "summary", "UNKNOWN"),
        extractJsonField(mechanismStatusStateJson, "summary", "UNKNOWN"),
        lastRuntimeProfileStatus,
        extractJsonField(actionTraceStateJson, "status", "unknown"),
        getDsMode(),
        systemCheckStateJson,
        autoCheckStateJson,
        ntDiagnosticsStateJson,
        mechanismStatusStateJson,
        actionTraceStateJson,
        runtimeProfileStateJson,
        selectedAutoStateJson,
        autoQueueStateJson,
        lastSubsystemDescriptionsJson);
  }

  private static String extractJsonField(String json, String field, String fallback) {
    if (json == null || json.isBlank() || field == null || field.isBlank()) {
      return fallback;
    }
    try {
      var node = JSON.readTree(json);
      if (node == null || !node.hasNonNull(field)) {
        return fallback;
      }
      return node.get(field).asText(fallback);
    } catch (IOException ex) {
      return fallback;
    }
  }

  public Command getAutonomousCommand() {
    return autoQueue.createAutonomousCommand();
  }

  public Optional<Pose2d> getQueuedStartPose() {
    return autoQueue.getQueuedStartPose();
  }

  private static boolean isFinitePose(Pose2d pose) {
    return isFinite(pose.getX())
        && isFinite(pose.getY())
        && isFinite(pose.getRotation().getRadians());
  }

  private static boolean isFinite(double value) {
    return Double.isFinite(value);
  }

  private static String getDsMode() {
    if (DriverStation.isDisabled()) {
      return "DISABLED";
    }
    if (DriverStation.isAutonomous()) {
      return "AUTO";
    }
    if (DriverStation.isTeleop()) {
      return "TELEOP";
    }
    if (DriverStation.isTest()) {
      return "TEST";
    }
    return "UNKNOWN";
  }

  private static String getAlliance() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return "UNKNOWN";
    }
    return alliance.get() == DriverStation.Alliance.Red ? "RED" : "BLUE";
  }

  private void syncQueueActionTrace() {
    RebuiltAutoQueue.QueueActionTrace queueActionTrace = autoQueue.getLastActionTrace();
    if (queueActionTrace == null
        || queueActionTrace.timestampSec() <= lastQueueActionTimestampSec) {
      return;
    }
    lastQueueActionTimestampSec = queueActionTrace.timestampSec();
    lastActionTraceState =
        new ActionTraceState(
            queueActionTrace.timestampSec(),
            "QUEUE",
            queueActionTrace.action(),
            queueActionTrace.accepted() ? "pass" : "fail",
            queueActionTrace.detail(),
            superstructure != null ? superstructure.getRequestedState().name() : lastRequestedState,
            superstructure != null ? superstructure.getCurrentState().name() : "UNKNOWN");
  }

  private void recordActionTrace(String source, String action, boolean accepted, String detail) {
    lastActionTraceState =
        new ActionTraceState(
            Timer.getFPGATimestamp(),
            source,
            action,
            accepted ? "pass" : "fail",
            detail == null ? "" : detail,
            superstructure != null ? superstructure.getRequestedState().name() : lastRequestedState,
            superstructure != null ? superstructure.getCurrentState().name() : "UNKNOWN");
  }

  private String buildSystemCheckStateJson(Optional<Pose2d> queuePreviewPose) {
    ArrayList<CheckItem> items = new ArrayList<>();
    int connectionCount = NetworkTableInstance.getDefault().getConnections().length;
    double batteryVoltage = RobotController.getBatteryVoltage();
    var canStatus = RobotController.getCANStatus();

    items.add(
        checkItem(
            "NetworkTables link",
            connectionCount > 0 ? CheckStatus.PASS : CheckStatus.FAIL,
            connectionCount > 0
                ? "At least one NetworkTables client is connected."
                : "No NetworkTables clients are connected."));
    items.add(
        checkItem(
            "Battery",
            batteryStatus(batteryVoltage),
            String.format(Locale.ROOT, "%.2f V", batteryVoltage)));
    items.add(
        checkItem(
            "CAN bus",
            canStatus.percentBusUtilization >= 0.90
                    || canStatus.txFullCount > 0
                    || canStatus.receiveErrorCount > 0
                    || canStatus.transmitErrorCount > 0
                ? CheckStatus.FAIL
                : canStatus.percentBusUtilization >= 0.70 ? CheckStatus.WARN : CheckStatus.PASS,
            String.format(
                Locale.ROOT,
                "util %.0f%%, txFull %d, rxErr %d, txErr %d",
                canStatus.percentBusUtilization * 100.0,
                canStatus.txFullCount,
                canStatus.receiveErrorCount,
                canStatus.transmitErrorCount)));
    items.add(
        checkItem(
            "Brownout state",
            RobotController.isBrownedOut() ? CheckStatus.FAIL : CheckStatus.PASS,
            RobotController.isBrownedOut()
                ? "Robot is currently browned out."
                : "No brownout reported."));
    items.add(
        checkItem(
            "Vision pose updates",
            !Config.Subsystems.VISION_ENABLED
                ? CheckStatus.FAIL
                : vision != null && vision.anyCameraHasAcceptedPose()
                    ? CheckStatus.PASS
                    : CheckStatus.WARN,
            !Config.Subsystems.VISION_ENABLED
                ? "Vision subsystem disabled."
                : vision != null && vision.anyCameraHasAcceptedPose()
                    ? "At least one camera currently has an accepted pose."
                    : "No accepted vision pose is currently visible."));
    items.add(
        checkItem(
            "Turret readiness",
            turret == null
                ? CheckStatus.FAIL
                : turret.isAtGoal() ? CheckStatus.PASS : CheckStatus.WARN,
            turret == null
                ? "Turret subsystem unavailable."
                : turret.getControlMode().name()
                    + (turret.isAtGoal() ? " and at goal." : " still moving.")));
    items.add(
        checkItem(
            "Mechanism zeroing",
            superstructure != null
                    && (superstructure.isIntakeDeployRezeroInProgress()
                        || superstructure.isManualIntakeDeployZeroSeekInProgress())
                ? CheckStatus.WARN
                : CheckStatus.PASS,
            superstructure == null
                ? "Superstructure unavailable."
                : superstructure.isIntakeDeployRezeroInProgress()
                    ? "Intake deploy rezero is running."
                    : superstructure.isManualIntakeDeployZeroSeekInProgress()
                        ? "Manual intake zero seek is running."
                        : "No intake zeroing process active."));
    items.add(
        checkItem(
            "Request channel",
            lastRequestAccepted ? CheckStatus.PASS : CheckStatus.FAIL,
            lastRequestAccepted
                ? "Last dashboard request was accepted."
                : (lastRequestReason == null || lastRequestReason.isBlank())
                    ? "Last dashboard request was rejected."
                    : lastRequestReason));
    items.add(
        checkItem(
            "Shoot gate",
            superstructure != null && superstructure.isShootEnabled()
                ? CheckStatus.PASS
                : CheckStatus.WARN,
            superstructure != null && superstructure.isShootEnabled()
                ? "Shoot gate currently enabled."
                : "Shoot gate currently disabled."));

    return writeJson(evaluateChecks("system", queuePreviewPose, items));
  }

  private String buildAutoCheckStateJson(Optional<Pose2d> queuePreviewPose) {
    boolean hasQueuedSteps = autoQueue.getQueueLength() > 0;
    String queuePhase = autoQueue.getPhaseName();
    int queueLength = autoQueue.getQueueLength();
    ArrayList<CheckItem> items = new ArrayList<>();
    items.add(
        checkItem(
            "Drive availability",
            drive != null ? CheckStatus.PASS : CheckStatus.FAIL,
            drive != null ? "Drive subsystem available." : "Drive subsystem unavailable."));
    items.add(
        checkItem(
            "Queue loaded",
            queueLength > 0 ? CheckStatus.PASS : CheckStatus.FAIL,
            queueLength > 0 ? queueLength + " queued step(s) staged." : "No queued steps staged."));
    items.add(
        checkItem(
            "Queue phase",
            "ERROR".equals(queuePhase)
                ? CheckStatus.FAIL
                : "READY".equals(queuePhase)
                        || "RUNNING".equals(queuePhase)
                        || "COMPLETE".equals(queuePhase)
                    ? CheckStatus.PASS
                    : CheckStatus.WARN,
            "Phase: " + (queuePhase == null || queuePhase.isBlank() ? "UNKNOWN" : queuePhase)));
    items.add(
        checkItem(
            "Preview pose",
            queuePreviewPose.isPresent()
                ? CheckStatus.PASS
                : hasQueuedSteps ? CheckStatus.WARN : CheckStatus.FAIL,
            queuePreviewPose
                .map(OperatorBoardTracker::formatPose)
                .orElse(
                    hasQueuedSteps
                        ? "Queue has steps but no preview pose is available."
                        : "Queue is empty.")));
    items.add(
        checkItem(
            "Driver station mode",
            DriverStation.isAutonomous() || DriverStation.isDisabled()
                ? CheckStatus.PASS
                : CheckStatus.WARN,
            "DS mode is " + getDsMode() + "."));
    items.add(
        checkItem(
            "Requested superstate",
            superstructure != null ? CheckStatus.PASS : CheckStatus.FAIL,
            superstructure != null
                ? "Requested state is " + superstructure.getRequestedState().name() + "."
                : "Superstructure unavailable."));

    return writeJson(evaluateChecks("auto", queuePreviewPose, items));
  }

  private String buildNtDiagnosticsStateJson() {
    rollPublishWindow(Timer.getFPGATimestamp());
    var instance = NetworkTableInstance.getDefault();
    String queueStateJson = autoQueue.getQueueStateJson();
    RuntimeModeProfile profile = RuntimeModeManager.getActiveProfile();
    var canStatus = RobotController.getCANStatus();
    ArrayList<PublishTopicSnapshot> hotTopics = new ArrayList<>();
    sampledPublishTopics.values().stream()
        .filter(
            stat ->
                stat.lastWindowBytesPerSec > 0.0
                    || stat.lastWindowPublishesPerSec > 0.0
                    || stat.totalPublishes > 0)
        .sorted(
            (left, right) ->
                Double.compare(right.lastWindowBytesPerSec, left.lastWindowBytesPerSec))
        .limit(5)
        .forEach(
            stat ->
                hotTopics.add(
                    new PublishTopicSnapshot(
                        stat.topic,
                        stat.totalPublishes,
                        stat.totalBytes,
                        stat.lastWindowPublishesPerSec,
                        stat.lastWindowBytesPerSec,
                        stat.peakPublishesPerSec,
                        stat.peakBytesPerSec,
                        stat.lastPayloadBytes)));
    return writeJson(
        new NtDiagnosticsState(
            Timer.getFPGATimestamp(),
            instance.getConnections().length,
            countTopics(instance),
            countOperatorBoardTopics(instance),
            queueStateJson.length(),
            lastRuntimeProfileSpecJson.length(),
            DriverStation.isDisabled()
                ? DISABLED_TELEMETRY_PUBLISH_PERIOD_SEC
                : TELEMETRY_PUBLISH_PERIOD_SEC,
            canStatus.percentBusUtilization,
            canStatus.txFullCount,
            canStatus.receiveErrorCount,
            canStatus.transmitErrorCount,
            sampledPublishRateHz,
            sampledPublishBytesPerSec,
            sampledPublishPeakRateHz,
            sampledPublishPeakBytesPerSec,
            profile.loggingMode().name(),
            profile.debugSubsystems().size(),
            profile.loggedSignals().size(),
            profile.publishedSignals().size(),
            hotTopics));
  }

  private void trackPublish(String topic, String value) {
    recordPublish(topic, value == null ? 0 : value.length());
  }

  private boolean shouldPublishString(String topic, String value) {
    String normalized = value == null ? "" : value;
    String previous = lastPublishedStringTopics.put(topic, normalized);
    return previous == null || !previous.equals(normalized);
  }

  private void trackPublish(String topic, boolean value) {
    recordPublish(topic, 1);
  }

  private void trackPublish(String topic, double value) {
    recordPublish(topic, 8);
  }

  private void trackPublish(String topic, double[] value) {
    recordPublish(topic, value == null ? 0 : value.length * 8);
  }

  private void recordPublish(String topic, int payloadBytes) {
    if (topic == null || topic.isBlank()) {
      return;
    }
    double nowSec = Timer.getFPGATimestamp();
    rollPublishWindow(nowSec);
    sampledPublishWindowCount++;
    sampledPublishWindowBytes += Math.max(payloadBytes, 0);
    PublishTopicAccumulator accumulator =
        sampledPublishTopics.computeIfAbsent(topic, PublishTopicAccumulator::new);
    accumulator.windowPublishes++;
    accumulator.windowBytes += Math.max(payloadBytes, 0);
    accumulator.totalPublishes++;
    accumulator.totalBytes += Math.max(payloadBytes, 0);
    accumulator.lastPayloadBytes = Math.max(payloadBytes, 0);
    accumulator.lastPublishTimestampSec = nowSec;
  }

  private void rollPublishWindow(double nowSec) {
    double elapsedSec = nowSec - sampledPublishWindowStartSec;
    if (elapsedSec < 1.0) {
      return;
    }
    sampledPublishRateHz = sampledPublishWindowCount / elapsedSec;
    sampledPublishBytesPerSec = sampledPublishWindowBytes / elapsedSec;
    sampledPublishPeakRateHz = Math.max(sampledPublishPeakRateHz, sampledPublishRateHz);
    sampledPublishPeakBytesPerSec =
        Math.max(sampledPublishPeakBytesPerSec, sampledPublishBytesPerSec);
    for (PublishTopicAccumulator accumulator : sampledPublishTopics.values()) {
      accumulator.lastWindowPublishesPerSec = accumulator.windowPublishes / elapsedSec;
      accumulator.lastWindowBytesPerSec = accumulator.windowBytes / elapsedSec;
      accumulator.peakPublishesPerSec =
          Math.max(accumulator.peakPublishesPerSec, accumulator.lastWindowPublishesPerSec);
      accumulator.peakBytesPerSec =
          Math.max(accumulator.peakBytesPerSec, accumulator.lastWindowBytesPerSec);
      accumulator.windowPublishes = 0;
      accumulator.windowBytes = 0;
    }
    sampledPublishWindowStartSec = nowSec;
    sampledPublishWindowCount = 0;
    sampledPublishWindowBytes = 0;
  }

  private String buildMechanismStatusStateJson() {
    ArrayList<MechanismStatusItem> items = new ArrayList<>();
    if (superstructure != null) {
      Rollers rollers = superstructure.getRollers();
      if (rollers != null) {
        if (rollers.intake != null) {
          items.add(
              mechanismItem(
                  rollers.intake.getDefinition().key(),
                  rollers.intake.getDefinition().displayName(),
                  rollers.intake.isConnected(),
                  rollers.intake.getHealth(),
                  rollers.intake.getControlMode().name(),
                  false,
                  String.format(
                      Locale.ROOT,
                      "goal %.1f V, applied %.1f V",
                      rollers.intake.getGoalVoltage(),
                      rollers.intake.getAppliedVolts())));
        }
        if (rollers.indexer != null) {
          items.add(
              mechanismItem(
                  rollers.indexer.getDefinition().key(),
                  rollers.indexer.getDefinition().displayName(),
                  rollers.indexer.isConnected(),
                  rollers.indexer.getHealth(),
                  "CLOSED_LOOP",
                  rollers.indexer.isAtGoal(),
                  String.format(
                      Locale.ROOT,
                      "target %.0f rpm, actual %.0f rpm",
                      rollers.indexer.getGoalVelocityRpm(),
                      rollers.indexer.getVelocityRpm())));
        }
        if (rollers.shooter != null) {
          items.add(
              mechanismItem(
                  rollers.shooter.getDefinition().key(),
                  rollers.shooter.getDefinition().displayName(),
                  rollers.shooter.isConnected(),
                  rollers.shooter.getHealth(),
                  "CLOSED_LOOP",
                  rollers.shooter.isAtGoal(),
                  String.format(
                      Locale.ROOT,
                      "target %.0f rpm, actual %.0f rpm",
                      rollers.shooter.getGoalVelocityRpm(),
                      rollers.shooter.getVelocityRpm())));
        }
      }
      Arms arms = superstructure.getArms();
      if (arms != null) {
        if (arms.intakePivot != null) {
          items.add(
              mechanismItem(
                  "intakePivot",
                  "Intake Pivot",
                  arms.intakePivot.isConnected(),
                  arms.intakePivot.getHealth(),
                  arms.intakePivot.getControlModeName(),
                  arms.intakePivot.isAtGoal(),
                  String.format(Locale.ROOT, "pos %.2f rad", arms.intakePivot.getPosition())));
        }
        if (arms.shooterPivot != null) {
          items.add(
              mechanismItem(
                  arms.shooterPivot.getDefinition().key(),
                  arms.shooterPivot.getDefinition().displayName(),
                  arms.shooterPivot.isConnected(),
                  arms.shooterPivot.getHealth(),
                  arms.shooterPivot.getControlMode().name(),
                  arms.shooterPivot.isAtGoal(),
                  String.format(
                      Locale.ROOT,
                      "target %.2f rad, pos %.2f rad",
                      arms.shooterPivot.getGoalPosition(),
                      arms.shooterPivot.getPosition())));
        }
      }
    }
    if (turret != null) {
      items.add(
          mechanismItem(
              turret.getDefinition().key(),
              turret.getDefinition().displayName(),
              turret.isConnected(),
              turret.getHealth(),
              turret.getControlMode().name(),
              turret.isAtGoal(),
              String.format(
                  Locale.ROOT,
                  "target %.2f rad, pos %.2f rad",
                  turret.getGoalRad(),
                  turret.getPositionRad())));
    }

    int offlineCount = 0;
    int degradedCount = 0;
    for (MechanismStatusItem item : items) {
      if (!item.connected()) {
        offlineCount++;
      } else if (!MechanismHealth.NOMINAL.name().equals(item.health())) {
        degradedCount++;
      }
    }
    String summary =
        items.isEmpty()
            ? "No migrated mechanisms available."
            : offlineCount > 0
                ? offlineCount + " offline"
                : degradedCount > 0 ? degradedCount + " degraded" : items.size() + " nominal";

    return writeJson(new MechanismStatusReport(Timer.getFPGATimestamp(), summary, items));
  }

  private static MechanismStatusItem mechanismItem(
      String key,
      String label,
      boolean connected,
      MechanismHealth health,
      String controlMode,
      boolean atGoal,
      String detail) {
    return new MechanismStatusItem(
        key,
        label,
        connected,
        health.name(),
        controlMode == null ? "UNKNOWN" : controlMode,
        atGoal,
        detail == null ? "" : detail);
  }

  private CheckReport evaluateChecks(
      String mode, Optional<Pose2d> queuePreviewPose, ArrayList<CheckItem> items) {
    int failCount = 0;
    int warnCount = 0;
    for (CheckItem item : items) {
      if (CheckStatus.FAIL.jsonName().equals(item.status())) {
        failCount++;
      } else if (CheckStatus.WARN.jsonName().equals(item.status())) {
        warnCount++;
      }
    }
    CheckStatus status =
        failCount > 0 ? CheckStatus.FAIL : warnCount > 0 ? CheckStatus.WARN : CheckStatus.PASS;
    String summary =
        status == CheckStatus.FAIL
            ? failCount + " fail, " + warnCount + " warn"
            : status == CheckStatus.WARN
                ? warnCount + (warnCount == 1 ? " warning" : " warnings")
                : "READY";
    return new CheckReport(
        Timer.getFPGATimestamp(),
        mode,
        status.jsonName(),
        summary,
        queuePreviewPose.map(OperatorBoardTracker::formatPose).orElse(""),
        items);
  }

  private static CheckItem checkItem(String label, CheckStatus status, String detail) {
    return new CheckItem(label, status.jsonName(), detail);
  }

  private static CheckStatus batteryStatus(double batteryVolts) {
    if (!Double.isFinite(batteryVolts)) {
      return CheckStatus.WARN;
    }
    if (batteryVolts >= 12.0) {
      return CheckStatus.PASS;
    }
    if (batteryVolts >= 11.2) {
      return CheckStatus.WARN;
    }
    return CheckStatus.FAIL;
  }

  private static String formatPose(Pose2d pose) {
    return String.format(
        Locale.ROOT,
        "x=%.2f, y=%.2f, h=%.1f deg",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  private static int countTopics(NetworkTableInstance instance) {
    try {
      return instance.getTopics("/", 0).length;
    } catch (Exception ex) {
      return -1;
    }
  }

  private static int countOperatorBoardTopics(NetworkTableInstance instance) {
    try {
      return instance.getTopics(OperatorBoardContract.BASE, 0).length;
    } catch (Exception ex) {
      return -1;
    }
  }

  private static String writeJson(Object value) {
    try {
      return JSON.writeValueAsString(value);
    } catch (JsonProcessingException ex) {
      DriverStation.reportError("Failed to serialize operator board state", ex.getStackTrace());
      return "{}";
    }
  }

  private record CheckReport(
      double timestampSec,
      String mode,
      String status,
      String summary,
      String previewPose,
      ArrayList<CheckItem> items) {}

  private record CheckItem(String label, String status, String detail) {}

  private record NtDiagnosticsState(
      double timestampSec,
      int connectionCount,
      int topicCount,
      int operatorBoardTopicCount,
      int queueStateBytes,
      int runtimeProfileBytes,
      double publishPeriodSec,
      double canBusUtilization,
      int canTxFullCount,
      int canReceiveErrorCount,
      int canTransmitErrorCount,
      double sampledPublishRateHz,
      double sampledPublishBytesPerSec,
      double sampledPublishPeakRateHz,
      double sampledPublishPeakBytesPerSec,
      String loggingMode,
      int debugSubsystemCount,
      int loggedSignalCount,
      int publishedSignalCount,
      ArrayList<PublishTopicSnapshot> hotTopics) {}

  private record PublishTopicSnapshot(
      String topic,
      long totalPublishes,
      long totalBytes,
      double publishesPerSec,
      double bytesPerSec,
      double peakPublishesPerSec,
      double peakBytesPerSec,
      int lastPayloadBytes) {}

  private record MechanismStatusReport(
      double timestampSec, String summary, ArrayList<MechanismStatusItem> items) {}

  private record MechanismStatusItem(
      String key,
      String label,
      boolean connected,
      String health,
      String controlMode,
      boolean atGoal,
      String detail) {}

  private record ActionTraceState(
      double timestampSec,
      String source,
      String action,
      String status,
      String detail,
      String requestedState,
      String currentState) {}

  private static final class PublishTopicAccumulator {
    private final String topic;
    private long totalPublishes = 0;
    private long totalBytes = 0;
    private int windowPublishes = 0;
    private int windowBytes = 0;
    private double lastWindowPublishesPerSec = 0.0;
    private double lastWindowBytesPerSec = 0.0;
    private double peakPublishesPerSec = 0.0;
    private double peakBytesPerSec = 0.0;
    private int lastPayloadBytes = 0;
    private double lastPublishTimestampSec = Double.NEGATIVE_INFINITY;

    private PublishTopicAccumulator(String topic) {
      this.topic = topic;
    }
  }

  private enum CheckStatus {
    PASS,
    WARN,
    FAIL;

    private String jsonName() {
      return name().toLowerCase(Locale.ROOT);
    }
  }

  @Override
  public void close() {
    if (webServer != null) {
      webServer.close();
    }
  }

  private record TargetSnapshot(String type, double[] pose, boolean valid) {
    private static TargetSnapshot unknown() {
      return new TargetSnapshot("UNKNOWN", new double[] {}, false);
    }
  }

  private final class OperatorBoardWebServer implements AutoCloseable {
    private final Path webRoot;
    private final DeployAutoLibrary deployAutoLibrary;
    private final Path musicDir;
    private final HttpServer server;
    private final ExecutorService executor;

    OperatorBoardWebServer(
        Path webRoot,
        DeployAutoLibrary deployAutoLibrary,
        Path musicDir,
        String bindAddress,
        int port)
        throws IOException {
      this.webRoot = Objects.requireNonNull(webRoot, "webRoot").toAbsolutePath().normalize();
      this.deployAutoLibrary = Objects.requireNonNull(deployAutoLibrary, "deployAutoLibrary");
      this.musicDir = Objects.requireNonNull(musicDir, "musicDir").toAbsolutePath().normalize();
      String bind = (bindAddress == null || bindAddress.isBlank()) ? "0.0.0.0" : bindAddress;
      this.server = HttpServer.create(new InetSocketAddress(bind, port), 0);
      this.executor =
          Executors.newFixedThreadPool(
              2,
              r -> {
                Thread t = new Thread(r);
                t.setName("OperatorBoardWeb-" + t.getId());
                t.setDaemon(true);
                return t;
              });
      server.setExecutor(executor);
      server.createContext("/", new StaticHandler());
      server.createContext("/planner-autos", new PlannerAutosHandler());
      server.createContext("/music-upload", new MusicUploadHandler());
      server.createContext("/api/storage/inventory", new StorageInventoryHandler());
      server.createContext("/api/subsystem-descriptions", new SubsystemDescriptionsHandler());
      server.createContext("/api/diagnostics/latest", new LatestDiagnosticHandler());
      server.createContext("/api/diagnostics/export", new DiagnosticExportHandler());
      server.createContext("/api/diagnostics/bundles", new DiagnosticBundlesHandler());
    }

    void start() {
      server.start();
    }

    @Override
    public void close() {
      server.stop(0);
      executor.shutdownNow();
    }

    private Path resolveStaticPath(String requestPath) {
      String normalized = requestPath;
      if (normalized == null || normalized.isBlank() || "/".equals(normalized)) {
        normalized = "/index.html";
      }
      Path resolved =
          webRoot.resolve(normalized.replaceFirst("^/", "")).normalize().toAbsolutePath();
      if (!resolved.startsWith(webRoot)) {
        return webRoot.resolve("index.html");
      }
      return resolved;
    }

    private final class StaticHandler implements HttpHandler {
      @Override
      public void handle(HttpExchange exchange) throws IOException {
        if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
          exchange.sendResponseHeaders(405, -1);
          return;
        }
        URI uri = exchange.getRequestURI();
        Path target = resolveStaticPath(uri.getPath());
        if (!Files.exists(target) || Files.isDirectory(target)) {
          target = webRoot.resolve("index.html");
        }
        byte[] bytes = Files.readAllBytes(target);
        Headers headers = exchange.getResponseHeaders();
        headers.add("Content-Type", inferredContentType(target));
        headers.add("Cache-Control", "no-store");
        exchange.sendResponseHeaders(200, bytes.length);
        try (OutputStream os = exchange.getResponseBody()) {
          os.write(bytes);
        }
      }
    }

    private final class MusicUploadHandler implements HttpHandler {
      @Override
      public void handle(HttpExchange exchange) throws IOException {
        if (!"POST".equalsIgnoreCase(exchange.getRequestMethod())) {
          exchange.sendResponseHeaders(405, -1);
          return;
        }
        byte[] bytes = exchange.getRequestBody().readAllBytes();
        if (bytes.length == 0) {
          exchange.sendResponseHeaders(400, -1);
          return;
        }
        Files.createDirectories(musicDir);
        Path target = musicDir.resolve("swerve.chrp");
        Files.write(target, bytes);
        byte[] response = "OK".getBytes();
        exchange.sendResponseHeaders(200, response.length);
        try (OutputStream os = exchange.getResponseBody()) {
          os.write(response);
        }
      }
    }

    private final class PlannerAutosHandler implements HttpHandler {
      @Override
      public void handle(HttpExchange exchange) throws IOException {
        if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
          exchange.sendResponseHeaders(405, -1);
          return;
        }
        String requestPath = exchange.getRequestURI().getPath();
        String suffix = requestPath.replaceFirst("^/planner-autos", "");
        if (suffix.isBlank() || "/".equals(suffix)) {
          sendJson(exchange, 200, deployAutoLibrary.buildManifestJson());
          return;
        }
        Optional<String> previewJson =
            deployAutoLibrary.loadAutoPreviewJson(suffix.replaceFirst("^/", ""));
        if (previewJson.isEmpty()) {
          exchange.sendResponseHeaders(404, -1);
          return;
        }
        byte[] bytes = previewJson.get().getBytes(StandardCharsets.UTF_8);
        Headers headers = exchange.getResponseHeaders();
        headers.add("Content-Type", "application/json; charset=utf-8");
        headers.add("Cache-Control", "no-store");
        exchange.sendResponseHeaders(200, bytes.length);
        try (OutputStream os = exchange.getResponseBody()) {
          os.write(bytes);
        }
      }
    }

    private final class StorageInventoryHandler implements HttpHandler {
      @Override
      public void handle(HttpExchange exchange) throws IOException {
        if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
          exchange.sendResponseHeaders(405, -1);
          return;
        }
        sendJson(exchange, 200, persistence.buildStorageInventory());
      }
    }

    private final class SubsystemDescriptionsHandler implements HttpHandler {
      @Override
      public void handle(HttpExchange exchange) throws IOException {
        if ("GET".equalsIgnoreCase(exchange.getRequestMethod())) {
          sendJson(exchange, 200, persistence.readSubsystemDescriptions());
          return;
        }
        if (!"PUT".equalsIgnoreCase(exchange.getRequestMethod())
            && !"POST".equalsIgnoreCase(exchange.getRequestMethod())) {
          exchange.sendResponseHeaders(405, -1);
          return;
        }
        try {
          OperatorBoardDataModels.SubsystemDescriptionsDocument request =
              OperatorBoardPersistence.JSON.readValue(
                  exchange.getRequestBody(),
                  OperatorBoardDataModels.SubsystemDescriptionsDocument.class);
          OperatorBoardDataModels.SubsystemDescriptionsDocument saved =
              persistence.saveSubsystemDescriptions(request);
          lastSubsystemDescriptionsJson = writeJson(saved);
          sendJson(exchange, 200, saved);
        } catch (IOException ex) {
          sendText(exchange, 400, "Invalid subsystem description payload");
        }
      }
    }

    private final class LatestDiagnosticHandler implements HttpHandler {
      @Override
      public void handle(HttpExchange exchange) throws IOException {
        if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
          exchange.sendResponseHeaders(405, -1);
          return;
        }
        Optional<OperatorBoardDataModels.DiagnosticBundleManifest> manifest =
            persistence.readLatestDiagnosticManifest();
        if (manifest.isEmpty() && lastDiagnosticSnapshot != null) {
          manifest = Optional.of(diagnosticBundleWriter.forceWrite(lastDiagnosticSnapshot));
        }
        if (manifest.isEmpty()) {
          exchange.sendResponseHeaders(404, -1);
          return;
        }
        sendJson(exchange, 200, manifest.get());
      }
    }

    private final class DiagnosticExportHandler implements HttpHandler {
      @Override
      public void handle(HttpExchange exchange) throws IOException {
        if (!"POST".equalsIgnoreCase(exchange.getRequestMethod())) {
          exchange.sendResponseHeaders(405, -1);
          return;
        }
        if (lastDiagnosticSnapshot == null) {
          sendText(exchange, 503, "Diagnostic snapshot not ready");
          return;
        }
        sendJson(exchange, 200, diagnosticBundleWriter.forceWrite(lastDiagnosticSnapshot));
      }
    }

    private final class DiagnosticBundlesHandler implements HttpHandler {
      @Override
      public void handle(HttpExchange exchange) throws IOException {
        if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
          exchange.sendResponseHeaders(405, -1);
          return;
        }
        sendJson(
            exchange,
            200,
            persistence.listDiagnosticBundles(parseLimit(exchange.getRequestURI(), 10)));
      }
    }

    private void sendJson(HttpExchange exchange, int statusCode, Object value) throws IOException {
      byte[] response =
          value instanceof String stringValue
              ? stringValue.getBytes(StandardCharsets.UTF_8)
              : OperatorBoardPersistence.JSON.writeValueAsBytes(value);
      Headers headers = exchange.getResponseHeaders();
      headers.set("Content-Type", "application/json; charset=utf-8");
      headers.set("Cache-Control", "no-store");
      exchange.sendResponseHeaders(statusCode, response.length);
      try (OutputStream os = exchange.getResponseBody()) {
        os.write(response);
      }
    }

    private void sendText(HttpExchange exchange, int statusCode, String body) throws IOException {
      byte[] response = (body == null ? "" : body).getBytes(StandardCharsets.UTF_8);
      Headers headers = exchange.getResponseHeaders();
      headers.set("Content-Type", "text/plain; charset=utf-8");
      headers.set("Cache-Control", "no-store");
      exchange.sendResponseHeaders(statusCode, response.length);
      try (OutputStream os = exchange.getResponseBody()) {
        os.write(response);
      }
    }

    private int parseLimit(URI uri, int fallback) {
      if (uri == null || uri.getQuery() == null || uri.getQuery().isBlank()) {
        return fallback;
      }
      for (String token : uri.getQuery().split("&")) {
        String[] parts = token.split("=", 2);
        if (parts.length == 2 && "limit".equals(parts[0])) {
          try {
            return Math.max(Integer.parseInt(parts[1]), 1);
          } catch (NumberFormatException ex) {
            return fallback;
          }
        }
      }
      return fallback;
    }

    private static String inferredContentType(Path file) {
      String name = file.getFileName().toString().toLowerCase(Locale.ROOT);
      if (name.endsWith(".html")) return "text/html; charset=utf-8";
      if (name.endsWith(".css")) return "text/css; charset=utf-8";
      if (name.endsWith(".js")) return "application/javascript; charset=utf-8";
      if (name.endsWith(".json")) return "application/json; charset=utf-8";
      if (name.endsWith(".png")) return "image/png";
      if (name.endsWith(".jpg") || name.endsWith(".jpeg")) return "image/jpeg";
      if (name.endsWith(".svg")) return "image/svg+xml";
      return "application/octet-stream";
    }
  }
}
