package org.Griffins1884.frc2026.subsystems.objectivetracker;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.Griffins1884.frc2026.commands.AutoAlignToPoseCommand;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.Superstructure.SuperState;
import org.Griffins1884.frc2026.subsystems.objectivetracker.RebuiltSpotLibrary.RebuiltSpot;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

final class RebuiltAutoQueue {
  private static final ObjectMapper JSON =
      new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
  private static final double EPSILON_METERS = 1e-6;

  private final RebuiltSpotLibrary spotLibrary;
  private final Superstructure superstructure;
  private final SwerveSubsystem drive;
  private final PathPlanAAutoLibrary autoLibrary;

  private final List<QueueStep> queueSteps = new ArrayList<>();
  private final List<NoGoZone> noGoZones = new ArrayList<>();
  private PoseSpec queuedStartPose;
  private int queueRevision = 0;
  private boolean queueRunning = false;
  private QueuePhase phase = QueuePhase.IDLE;
  private Command activeCommand;
  private String activeLabel = "";
  private String queueMessage = "Select a deployed PathPlanA auto to preview and run.";
  private String lastQueueStateJson = "";
  private String lastQuickRunStateJson = "";
  private SelectedAutoState selectedAutoState =
      new SelectedAutoState(
          "", "", "", "", false, "No auto selected.", false, 0, Timer.getFPGATimestamp(), null);
  private QueueActionTrace lastActionTrace =
      new QueueActionTrace(
          Timer.getFPGATimestamp(),
          "QUEUE_INIT",
          true,
          "Queue initialized.",
          "Select a deployed PathPlanA auto to preview and run.");

  RebuiltAutoQueue(
      RebuiltSpotLibrary spotLibrary,
      Superstructure superstructure,
      SwerveSubsystem drive,
      PathPlanAAutoLibrary autoLibrary) {
    this.spotLibrary = spotLibrary;
    this.superstructure = superstructure;
    this.drive = drive;
    this.autoLibrary = autoLibrary;
  }

  void handleInputs(OperatorBoardIO.OperatorBoardIOInputs inputs) {
    if (inputs.selectedAutoId.length > 0) {
      applySelectedAutoId(inputs.selectedAutoId[inputs.selectedAutoId.length - 1]);
    }
    if (inputs.autoQueueSpec.length > 0) {
      applyQueueSpec(inputs.autoQueueSpec[inputs.autoQueueSpec.length - 1]);
    }
    if (inputs.autoQueueCommand.length > 0) {
      applyQueueCommand(inputs.autoQueueCommand[inputs.autoQueueCommand.length - 1]);
    }
  }

  void periodic() {
    updateQueueExecution();
  }

  String getQueueStateJson() {
    String json = buildQueueStateJson();
    lastQueueStateJson = json;
    return json;
  }

  Optional<Pose2d> getPreviewPose() {
    Optional<Pose2d> startPose = getQueuedStartPose();
    if (startPose.isPresent()) {
      return startPose;
    }
    if (queueSteps.isEmpty()) {
      return Optional.empty();
    }
    return queueSteps.get(0).resolvePose(spotLibrary, getRuntimeAlliance());
  }

  Optional<Pose2d> getQueuedStartPose() {
    if (queuedStartPose != null) {
      return queuedStartPose.resolvePose(spotLibrary, getRuntimeAlliance());
    }
    if (queueSteps.isEmpty()) {
      return Optional.empty();
    }
    return queueSteps.get(0).resolvePose(spotLibrary, getRuntimeAlliance());
  }

  int getQueueLength() {
    return queueSteps.size();
  }

  String getPhaseName() {
    return phase.name();
  }

  QueueActionTrace getLastActionTrace() {
    return lastActionTrace;
  }

  String getQuickRunStateJson() {
    return lastQuickRunStateJson;
  }

  String getSelectedAutoStateJson() {
    try {
      return JSON.writeValueAsString(selectedAutoState);
    } catch (JsonProcessingException ex) {
      DriverStation.reportError("Failed to serialize selected auto state", ex.getStackTrace());
      return "{}";
    }
  }

  Command createAutonomousCommand() {
    if (queueSteps.isEmpty()) {
      return null;
    }
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              if (drive == null) {
                return;
              }
              getQueuedStartPose().ifPresent(drive::resetOdometry);
            }),
        Commands.runOnce(this::startQueue),
        Commands.waitUntil(() -> !queueRunning && activeCommand == null));
  }

  private void applyQueueSpec(String raw) {
    if (raw == null || raw.isBlank()) {
      clearQueue("Queue cleared.");
      return;
    }
    try {
      QueueSpecPayload payload = JSON.readValue(raw, QueueSpecPayload.class);
      List<QueueStep> parsed = new ArrayList<>();
      if (payload.steps != null) {
        for (QueueStepDto dto : payload.steps) {
          QueueStep.fromDto(dto).ifPresent(parsed::add);
        }
      }
      queuedStartPose = PoseSpec.fromDto(payload.startPose).orElse(null);
      noGoZones.clear();
      if (payload.noGoZones != null) {
        for (NoGoZoneDto dto : payload.noGoZones) {
          NoGoZone.fromDto(dto).ifPresent(noGoZones::add);
        }
      }
      queueSteps.clear();
      queueSteps.addAll(parsed);
      queueRevision = payload.revision != null ? payload.revision.intValue() : queueRevision + 1;
      cancelActiveCommand();
      queueRunning = false;
      phase = queueSteps.isEmpty() ? QueuePhase.IDLE : QueuePhase.READY;
      activeLabel = "";
      queueMessage =
          queueSteps.isEmpty()
              ? "Queue cleared."
              : "Queue updated. Press Start or enable autonomous.";
      selectedAutoState =
          new SelectedAutoState(
              selectedAutoState.id(),
              selectedAutoState.name(),
              selectedAutoState.folder(),
              selectedAutoState.relativePath(),
              selectedAutoState.loaded(),
              queueMessage,
              selectedAutoState.selected(),
              queueSteps.size(),
              Timer.getFPGATimestamp(),
              queuedStartPose == null
                  ? null
                  : new PoseState(
                      queuedStartPose.xMeters,
                      queuedStartPose.yMeters,
                      queuedStartPose.headingDeg));
      recordAction("QUEUE_SPEC_APPLY", true, queueMessage);
    } catch (Exception ex) {
      recordAction("QUEUE_SPEC_APPLY", false, "Failed to parse queue spec.");
      DriverStation.reportError(
          "Failed to parse auto queue spec: " + ex.getMessage(), ex.getStackTrace());
    }
  }

  private void applyQueueCommand(String raw) {
    if (raw == null || raw.isBlank()) {
      return;
    }
    try {
      QueueCommandPayload payload = JSON.readValue(raw, QueueCommandPayload.class);
      QueueCommand command = QueueCommand.parse(payload.command);
      if (command == null) {
        recordAction("QUEUE_COMMAND", false, "Unknown queue command.");
        return;
      }
      switch (command) {
        case START -> recordAction("QUEUE_START", startQueue(), queueMessage);
        case STOP -> {
          stopQueue("Queue stopped.");
          recordAction("QUEUE_STOP", true, queueMessage);
        }
        case CLEAR -> {
          clearQueue("Queue cleared.");
          recordAction("QUEUE_CLEAR", true, queueMessage);
        }
        case SKIP -> recordAction("QUEUE_SKIP", skipStep(), queueMessage);
        case QUICK_RUN -> {
          lastQuickRunStateJson = buildQuickRunStateJson();
          recordAction("QUEUE_QUICK_RUN", true, "Quick run validation completed.");
        }
      }
    } catch (Exception ex) {
      recordAction("QUEUE_COMMAND", false, "Failed to parse queue command.");
      DriverStation.reportError(
          "Failed to parse auto queue command: " + ex.getMessage(), ex.getStackTrace());
    }
  }

  private boolean startQueue() {
    if (drive == null) {
      queueMessage = "Drive unavailable.";
      queueRunning = false;
      phase = QueuePhase.ERROR;
      return false;
    }
    if (queueSteps.isEmpty()) {
      queueMessage = "Queue is empty.";
      queueRunning = false;
      phase = QueuePhase.IDLE;
      return false;
    }
    if (getQueuedStartPose().isEmpty()) {
      queueMessage = "Queue start pose is invalid.";
      queueRunning = false;
      phase = QueuePhase.ERROR;
      return false;
    }
    queueRunning = true;
    phase = QueuePhase.RUNNING;
    queueMessage = "Executing queue.";
    if (activeCommand == null) {
      startNextStep();
    }
    return true;
  }

  private void stopQueue(String message) {
    queueRunning = false;
    cancelActiveCommand();
    phase = queueSteps.isEmpty() ? QueuePhase.IDLE : QueuePhase.READY;
    activeLabel = "";
    queueMessage = message;
  }

  private boolean skipStep() {
    if (queueSteps.isEmpty()) {
      queueMessage = "No queued step to skip.";
      return false;
    }
    removeHeadStep("Skipped step.");
    return true;
  }

  private void clearQueue(String message) {
    queueSteps.clear();
    queuedStartPose = null;
    noGoZones.clear();
    queueRevision++;
    cancelActiveCommand();
    queueRunning = false;
    phase = QueuePhase.IDLE;
    activeLabel = "";
    queueMessage = message;
  }

  private void applySelectedAutoId(String autoId) {
    String normalizedId = blankToNull(autoId);
    if (normalizedId != null && normalizedId.equals(selectedAutoState.id())) {
      return;
    }
    if (normalizedId == null) {
      clearQueue("No auto selected.");
      selectedAutoState =
          new SelectedAutoState(
              "", "", "", "", false, "No auto selected.", false, 0, Timer.getFPGATimestamp(), null);
      recordAction("AUTO_SELECT", true, "Cleared selected auto.");
      return;
    }
    if (autoLibrary == null) {
      selectedAutoState =
          new SelectedAutoState(
              normalizedId,
              normalizedId,
              "",
              "",
              false,
              "Auto library unavailable.",
              true,
              0,
              Timer.getFPGATimestamp(),
              null);
      clearQueue("Auto library unavailable.");
      recordAction("AUTO_SELECT", false, "Auto library unavailable.");
      return;
    }
    Optional<PathPlanAAutoLibrary.LoadedAuto> loadedAuto = autoLibrary.loadAuto(normalizedId);
    if (loadedAuto.isEmpty()) {
      selectedAutoState =
          new SelectedAutoState(
              normalizedId,
              normalizedId,
              "",
              "",
              false,
              "Selected auto was not found in deploy.",
              true,
              0,
              Timer.getFPGATimestamp(),
              null);
      clearQueue("Selected auto was not found in deploy.");
      recordAction("AUTO_SELECT", false, "Selected auto was not found in deploy.");
      return;
    }
    loadSelectedAuto(loadedAuto.get());
  }

  private void loadSelectedAuto(PathPlanAAutoLibrary.LoadedAuto loadedAuto) {
    queueSteps.clear();
    noGoZones.clear();
    for (PathPlanAAutoLibrary.StepSpec step : loadedAuto.steps()) {
      queueSteps.add(QueueStep.fromLibrary(step));
    }
    for (PathPlanAAutoLibrary.ZoneSpec zone : loadedAuto.customZones()) {
      noGoZones.add(NoGoZone.fromLibrary(zone));
    }
    queuedStartPose = PoseSpec.fromLibrary(loadedAuto.startPose()).orElse(null);
    queueRevision++;
    cancelActiveCommand();
    queueRunning = false;
    phase = queueSteps.isEmpty() ? QueuePhase.IDLE : QueuePhase.READY;
    activeLabel = "";
    queueMessage =
        queueSteps.isEmpty()
            ? "Selected auto has no runnable steps."
            : "Selected " + defaultAutoName(loadedAuto) + " for autonomous.";
    selectedAutoState =
        new SelectedAutoState(
            nullToEmpty(loadedAuto.id()),
            defaultAutoName(loadedAuto),
            nullToEmpty(loadedAuto.folder()),
            nullToEmpty(loadedAuto.relativePath()),
            !queueSteps.isEmpty(),
            queueMessage,
            true,
            queueSteps.size(),
            Timer.getFPGATimestamp(),
            queuedStartPose == null
                ? null
                : new PoseState(
                    queuedStartPose.xMeters, queuedStartPose.yMeters, queuedStartPose.headingDeg));
    recordAction("AUTO_SELECT", !queueSteps.isEmpty(), queueMessage);
  }

  private void updateQueueExecution() {
    if (queueRunning && activeCommand == null) {
      startNextStep();
      return;
    }

    if (activeCommand != null && !CommandScheduler.getInstance().isScheduled(activeCommand)) {
      activeCommand = null;
      if (!queueSteps.isEmpty()) {
        queueSteps.remove(0);
        queueRevision++;
      }
      if (queueSteps.isEmpty()) {
        finishQueue();
      } else if (queueRunning) {
        startNextStep();
      }
    }
  }

  private void startNextStep() {
    if (!queueRunning || queueSteps.isEmpty()) {
      finishQueue();
      return;
    }
    QueueStep step = queueSteps.get(0);
    Command command = buildCommand(step, queueSteps.size() <= 1, getRuntimeStepStartPose());
    if (command == null) {
      removeHeadStep("Unable to build safe command for " + step.displayLabel(spotLibrary) + ".");
      return;
    }
    cancelActiveCommand();
    activeCommand = command;
    activeLabel = step.displayLabel(spotLibrary);
    queueMessage = "Running " + activeLabel + ".";
    phase = QueuePhase.RUNNING;
    CommandScheduler.getInstance().schedule(command);
  }

  private void removeHeadStep(String message) {
    if (!queueSteps.isEmpty()) {
      queueSteps.remove(0);
      queueRevision++;
    }
    cancelActiveCommand();
    activeLabel = "";
    if (queueSteps.isEmpty()) {
      queueRunning = false;
      phase = QueuePhase.IDLE;
      queueMessage = message;
      return;
    }
    queueMessage = message;
    if (queueRunning) {
      startNextStep();
    }
  }

  private void finishQueue() {
    cancelActiveCommand();
    queueRunning = false;
    phase = QueuePhase.COMPLETE;
    activeLabel = "";
    queueMessage = "Queue complete.";
  }

  private void cancelActiveCommand() {
    if (activeCommand != null && CommandScheduler.getInstance().isScheduled(activeCommand)) {
      activeCommand.cancel();
    }
    activeCommand = null;
  }

  private Command buildCommand(QueueStep step) {
    return buildCommand(step, queueSteps.size() <= 1, getRuntimeStepStartPose());
  }

  private Command buildCommand(
      QueueStep step, boolean finalStep, Optional<Pose2d> startingPoseOverride) {
    List<Pose2d> routePoses = resolveRoutePoses(step);
    if (routePoses.isEmpty() || drive == null || !isRouteSafe(startingPoseOverride, routePoses)) {
      return null;
    }

    double alignConstraintFactor = RebuiltAutoConstants.QUEUE_ALIGN_CONSTRAINT_FACTOR.get();
    double alignToleranceMeters = RebuiltAutoConstants.QUEUE_ALIGN_TOLERANCE_METERS.get();
    double alignTimeoutSeconds = RebuiltAutoConstants.QUEUE_ALIGN_TIMEOUT_SEC.get();
    double flowThroughEndVelocity = RebuiltAutoConstants.QUEUE_FLOW_THROUGH_END_VELOCITY_MPS.get();
    double stepConstraintFactor =
        step.constraintFactor != null
            ? Math.max(0.05, step.constraintFactor)
            : alignConstraintFactor;
    double stepToleranceMeters =
        step.toleranceMeters != null ? Math.max(0.01, step.toleranceMeters) : alignToleranceMeters;
    double stepTimeoutSeconds =
        step.timeoutSeconds != null ? Math.max(0.05, step.timeoutSeconds) : alignTimeoutSeconds;
    boolean stepStopOnEnd = step.stopOnEnd == null ? finalStep : step.stopOnEnd.booleanValue();

    ArrayList<Command> routeCommands = new ArrayList<>();
    for (int i = 0; i < routePoses.size(); i++) {
      boolean finalRoutePose = i == routePoses.size() - 1;
      double endVelocity =
          step.endVelocityMps != null
              ? step.endVelocityMps
              : (finalStep && finalRoutePose ? 0.0 : flowThroughEndVelocity);
      routeCommands.add(
          new AutoAlignToPoseCommand(
                  drive,
                  routePoses.get(i),
                  stepConstraintFactor,
                  endVelocity,
                  stepToleranceMeters,
                  false,
                  finalRoutePose && stepStopOnEnd)
              .withTimeout(stepTimeoutSeconds));
    }

    Command routeCommand = Commands.sequence(routeCommands.toArray(new Command[0]));
    Optional<SuperState> requestedState = step.resolveRequestedState();
    if (requestedState.isPresent() && superstructure != null) {
      return Commands.sequence(
          Commands.runOnce(() -> superstructure.requestStateFromDashboard(requestedState.get())),
          routeCommand);
    }
    return routeCommand;
  }

  private List<Pose2d> resolveRoutePoses(QueueStep step) {
    Alliance alliance = getRuntimeAlliance();
    ArrayList<Pose2d> routePoses = new ArrayList<>();
    for (PoseSpec waypoint : step.routeWaypoints) {
      waypoint.resolvePose(spotLibrary, alliance).ifPresent(routePoses::add);
    }
    step.resolvePose(spotLibrary, alliance).ifPresent(routePoses::add);
    return routePoses;
  }

  private Optional<Pose2d> getRuntimeStepStartPose() {
    if (drive != null) {
      Pose2d currentPose = drive.getPose();
      if (currentPose != null) {
        return Optional.of(currentPose);
      }
    }
    return getQueuedStartPose();
  }

  private boolean isRouteSafe(Optional<Pose2d> startPose, List<Pose2d> routePoses) {
    if (routePoses.isEmpty()) {
      return false;
    }
    if (startPose.isEmpty() || !isPoseSafe(startPose.get())) {
      return false;
    }
    Pose2d previous = startPose.get();
    for (Pose2d pose : routePoses) {
      if (!isPoseSafe(pose) || segmentHitsKeepOut(previous, pose)) {
        return false;
      }
      previous = pose;
    }
    return true;
  }

  private boolean isPoseSafe(Pose2d pose) {
    if (!Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getRadians())) {
      return false;
    }
    if (pose.getY() < -EPSILON_METERS
        || pose.getY() > spotLibrary.getFieldWidthMeters() + EPSILON_METERS) {
      return false;
    }
    double x = pose.getX();
    Alliance alliance = getRuntimeAlliance();
    double safeMaxX = spotLibrary.getSafeAutoMaxXMeters();
    if (alliance == Alliance.Red) {
      return x >= spotLibrary.getFieldLengthMeters() - safeMaxX - EPSILON_METERS
          && x <= spotLibrary.getFieldLengthMeters() + EPSILON_METERS;
    }
    return x >= -EPSILON_METERS && x <= safeMaxX + EPSILON_METERS;
  }

  private boolean segmentHitsKeepOut(Pose2d start, Pose2d end) {
    for (NoGoZone zone : getRuntimeZones()) {
      if (zone.contains(start) || zone.contains(end) || zone.segmentIntersects(start, end)) {
        return true;
      }
    }
    return false;
  }

  private List<NoGoZone> getRuntimeZones() {
    Alliance alliance = getRuntimeAlliance();
    ArrayList<NoGoZone> runtimeZones = new ArrayList<>();
    for (NoGoZone zone : noGoZones) {
      runtimeZones.add(zone.resolveForAlliance(spotLibrary, alliance));
    }
    return runtimeZones;
  }

  private Alliance getRuntimeAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  private String buildQueueStateJson() {
    try {
      List<QueueStateStep> steps = new ArrayList<>();
      Alliance alliance = getRuntimeAlliance();
      for (int i = 0; i < queueSteps.size(); i++) {
        QueueStep step = queueSteps.get(i);
        ArrayList<PoseState> routeWaypoints = new ArrayList<>();
        for (PoseSpec waypoint : step.routeWaypoints) {
          waypoint
              .resolvePose(spotLibrary, alliance)
              .ifPresent(
                  pose ->
                      routeWaypoints.add(
                          new PoseState(
                              pose.getX(), pose.getY(), pose.getRotation().getDegrees())));
        }
        steps.add(
            new QueueStateStep(
                step.spotId,
                step.displayLabel(spotLibrary),
                step.requestedStateName,
                statusForIndex(i).name(),
                step.group,
                step.alliance,
                step.xMeters,
                step.yMeters,
                step.headingDeg,
                routeWaypoints));
      }

      ArrayList<NoGoZoneState> zoneStates = new ArrayList<>();
      for (NoGoZone zone : noGoZones) {
        zoneStates.add(
            new NoGoZoneState(
                zone.id,
                zone.label,
                zone.xMinMeters,
                zone.yMinMeters,
                zone.xMaxMeters,
                zone.yMaxMeters,
                zone.locked));
      }

      QueueStatePayload payload =
          new QueueStatePayload(
              selectedAutoState.id(),
              selectedAutoState.name(),
              queueRevision,
              queueRunning,
              phase.name(),
              queueRunning && !queueSteps.isEmpty() ? 0 : -1,
              queueMessage,
              activeLabel,
              queuedStartPose == null
                  ? null
                  : new PoseState(
                      queuedStartPose.xMeters, queuedStartPose.yMeters, queuedStartPose.headingDeg),
              zoneStates,
              steps);
      return JSON.writeValueAsString(payload);
    } catch (JsonProcessingException ex) {
      DriverStation.reportError("Failed to serialize queue state", ex.getStackTrace());
      return lastQueueStateJson.isBlank() ? "{}" : lastQueueStateJson;
    }
  }

  private StepStatus statusForIndex(int index) {
    if (!queueRunning) {
      return index == 0 && phase == QueuePhase.READY ? StepStatus.READY : StepStatus.PENDING;
    }
    if (index == 0) {
      return activeCommand != null ? StepStatus.ACTIVE : StepStatus.READY;
    }
    return StepStatus.QUEUED;
  }

  void logState() {
    Logger.recordOutput("AutoQueue/Running", queueRunning);
    Logger.recordOutput("AutoQueue/Phase", phase.name());
    Logger.recordOutput("AutoQueue/Length", queueSteps.size());
    Logger.recordOutput("AutoQueue/ActiveLabel", activeLabel);
    Logger.recordOutput("AutoQueue/Message", queueMessage);
  }

  private String buildQuickRunStateJson() {
    try {
      ArrayList<QuickRunItem> items = new ArrayList<>();
      if (drive == null) {
        items.add(new QuickRunItem("Drive availability", "fail", "Drive subsystem unavailable."));
      } else {
        items.add(new QuickRunItem("Drive availability", "pass", "Drive subsystem available."));
      }
      Optional<Pose2d> startPose = getQueuedStartPose();
      if (startPose.isEmpty()) {
        items.add(
            new QuickRunItem("Queue start pose", "fail", "Start pose is not set or invalid."));
      } else if (!isPoseSafe(startPose.get())) {
        items.add(
            new QuickRunItem(
                "Queue start pose",
                "fail",
                "Start pose is outside alliance-safe autonomous bounds."));
      } else {
        items.add(
            new QuickRunItem(
                "Queue start pose", "pass", "Start pose " + formatPose(startPose.get())));
      }
      if (queueSteps.isEmpty()) {
        items.add(new QuickRunItem("Queue loaded", "fail", "No queued steps staged."));
      } else {
        items.add(
            new QuickRunItem(
                "Queue loaded", "pass", queueSteps.size() + " queued step(s) staged."));
      }

      Pose2d previous = startPose.orElse(null);
      for (int i = 0; i < queueSteps.size(); i++) {
        QueueStep step = queueSteps.get(i);
        String label = "Step " + (i + 1) + " • " + step.displayLabel(spotLibrary);
        Optional<Pose2d> pose = step.resolvePose(spotLibrary, getRuntimeAlliance());
        if (pose.isEmpty()) {
          items.add(new QuickRunItem(label, "fail", "Unable to resolve target pose."));
          continue;
        }
        if (step.requestedStateName != null && step.resolveRequestedState().isEmpty()) {
          items.add(
              new QuickRunItem(
                  label,
                  "fail",
                  "Requested state \"" + step.requestedStateName + "\" is invalid."));
          continue;
        }
        List<Pose2d> routePoses = resolveRoutePoses(step);
        if (routePoses.isEmpty()) {
          items.add(new QuickRunItem(label, "fail", "Safe route is empty."));
          continue;
        }
        if (!isRouteSafe(Optional.ofNullable(previous), routePoses)) {
          items.add(
              new QuickRunItem(
                  label, "fail", "Route crosses a keep-out zone, white line, or field boundary."));
          continue;
        }
        if (previous != null && segmentHitsKeepOut(previous, routePoses.get(0))) {
          items.add(new QuickRunItem(label, "fail", "Step entry path is blocked."));
          continue;
        }
        Command command =
            buildCommand(step, i == queueSteps.size() - 1, Optional.ofNullable(previous));
        if (command == null) {
          items.add(new QuickRunItem(label, "fail", "Unable to build command."));
          continue;
        }
        previous = routePoses.get(routePoses.size() - 1);
        items.add(
            new QuickRunItem(
                label,
                "pass",
                String.format(
                    Locale.ROOT,
                    "Route %d leg(s), target %s, state %s",
                    routePoses.size(),
                    formatPose(pose.get()),
                    step.requestedStateName == null ? "UNCHANGED" : step.requestedStateName)));
      }

      long failCount = items.stream().filter(item -> "fail".equals(item.status())).count();
      long warnCount = items.stream().filter(item -> "warn".equals(item.status())).count();
      String status = failCount > 0 ? "fail" : warnCount > 0 ? "warn" : "pass";
      String summary =
          failCount > 0
              ? failCount + " fail, " + warnCount + " warn"
              : warnCount > 0 ? warnCount + " warning" + (warnCount == 1 ? "" : "s") : "READY";
      return JSON.writeValueAsString(
          new QuickRunReport(Timer.getFPGATimestamp(), status, summary, items));
    } catch (JsonProcessingException ex) {
      DriverStation.reportError("Failed to serialize quick run state", ex.getStackTrace());
      return lastQuickRunStateJson.isBlank() ? "{}" : lastQuickRunStateJson;
    }
  }

  private static String formatPose(Pose2d pose) {
    return String.format(
        Locale.ROOT,
        "x=%.2f, y=%.2f, h=%.1f deg",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  private void recordAction(String action, boolean accepted, String detail) {
    lastActionTrace =
        new QueueActionTrace(
            Timer.getFPGATimestamp(), action, accepted, detail == null ? "" : detail, queueMessage);
  }

  private static Rotation2d headingDegrees(double headingDeg) {
    return Rotation2d.fromDegrees(headingDeg);
  }

  private static double normalizeHeading(double headingDeg) {
    double normalized = headingDeg;
    while (normalized > 180.0) {
      normalized -= 360.0;
    }
    while (normalized <= -180.0) {
      normalized += 360.0;
    }
    return normalized;
  }

  private static double mirrorHeading(double headingDeg) {
    return normalizeHeading(180.0 - headingDeg);
  }

  private static Double sanitizeFinite(Double value) {
    return value != null && Double.isFinite(value.doubleValue()) ? value : null;
  }

  private static String blankToNull(String value) {
    return value == null || value.isBlank() ? null : value.trim();
  }

  private static String nullToEmpty(String value) {
    return value == null ? "" : value;
  }

  private static String defaultAutoName(PathPlanAAutoLibrary.LoadedAuto loadedAuto) {
    if (loadedAuto.name() != null && !loadedAuto.name().isBlank()) {
      return loadedAuto.name();
    }
    if (loadedAuto.id() != null && !loadedAuto.id().isBlank()) {
      return loadedAuto.id();
    }
    return "Unnamed Auto";
  }

  private enum QueuePhase {
    IDLE,
    READY,
    RUNNING,
    COMPLETE,
    ERROR
  }

  private enum StepStatus {
    PENDING,
    READY,
    ACTIVE,
    QUEUED
  }

  private enum QueueCommand {
    START,
    STOP,
    CLEAR,
    SKIP,
    QUICK_RUN;

    static QueueCommand parse(String value) {
      if (value == null || value.isBlank()) {
        return null;
      }
      try {
        return QueueCommand.valueOf(value.trim().toUpperCase(Locale.ROOT));
      } catch (IllegalArgumentException ex) {
        return null;
      }
    }
  }

  private static final class PoseSpec {
    private final double xMeters;
    private final double yMeters;
    private final double headingDeg;

    private PoseSpec(double xMeters, double yMeters, double headingDeg) {
      this.xMeters = xMeters;
      this.yMeters = yMeters;
      this.headingDeg = headingDeg;
    }

    static Optional<PoseSpec> fromDto(PoseDto dto) {
      if (dto == null) {
        return Optional.empty();
      }
      Double xMeters = sanitizeFinite(dto.xMeters);
      Double yMeters = sanitizeFinite(dto.yMeters);
      Double headingDeg = sanitizeFinite(dto.headingDeg);
      if (xMeters == null || yMeters == null) {
        return Optional.empty();
      }
      return Optional.of(new PoseSpec(xMeters, yMeters, headingDeg == null ? 0.0 : headingDeg));
    }

    static Optional<PoseSpec> fromLibrary(PathPlanAAutoLibrary.PoseSpec pose) {
      if (pose == null) {
        return Optional.empty();
      }
      return Optional.of(new PoseSpec(pose.xMeters(), pose.yMeters(), pose.headingDeg()));
    }

    Optional<Pose2d> resolvePose(RebuiltSpotLibrary spotLibrary, Alliance alliance) {
      if (!Double.isFinite(xMeters) || !Double.isFinite(yMeters) || !Double.isFinite(headingDeg)) {
        return Optional.empty();
      }
      if (alliance == Alliance.Red) {
        return Optional.of(
            new Pose2d(
                spotLibrary.getFieldLengthMeters() - xMeters,
                yMeters,
                headingDegrees(mirrorHeading(headingDeg))));
      }
      return Optional.of(new Pose2d(xMeters, yMeters, headingDegrees(headingDeg)));
    }
  }

  private static final class QueueStep {
    private final String spotId;
    private final String requestedStateName;
    private final String label;
    private final String group;
    private final String alliance;
    private final Double xMeters;
    private final Double yMeters;
    private final Double headingDeg;
    private final Double constraintFactor;
    private final Double toleranceMeters;
    private final Double timeoutSeconds;
    private final Double endVelocityMps;
    private final Boolean stopOnEnd;
    private final List<PoseSpec> routeWaypoints;

    private QueueStep(
        String spotId,
        String requestedStateName,
        String label,
        String group,
        String alliance,
        Double xMeters,
        Double yMeters,
        Double headingDeg,
        Double constraintFactor,
        Double toleranceMeters,
        Double timeoutSeconds,
        Double endVelocityMps,
        Boolean stopOnEnd,
        List<PoseSpec> routeWaypoints) {
      this.spotId = spotId;
      this.requestedStateName = requestedStateName;
      this.label = label;
      this.group = group;
      this.alliance = alliance;
      this.xMeters = xMeters;
      this.yMeters = yMeters;
      this.headingDeg = headingDeg;
      this.constraintFactor = constraintFactor;
      this.toleranceMeters = toleranceMeters;
      this.timeoutSeconds = timeoutSeconds;
      this.endVelocityMps = endVelocityMps;
      this.stopOnEnd = stopOnEnd;
      this.routeWaypoints = routeWaypoints;
    }

    static Optional<QueueStep> fromDto(QueueStepDto dto) {
      if (dto == null) {
        return Optional.empty();
      }
      String spotId = blankToNull(dto.spotId);
      Double xMeters = sanitizeFinite(dto.xMeters);
      Double yMeters = sanitizeFinite(dto.yMeters);
      Double headingDeg = sanitizeFinite(dto.headingDeg);
      if (spotId == null && (xMeters == null || yMeters == null)) {
        return Optional.empty();
      }
      ArrayList<PoseSpec> routeWaypoints = new ArrayList<>();
      if (dto.routeWaypoints != null) {
        for (PoseDto waypoint : dto.routeWaypoints) {
          PoseSpec.fromDto(waypoint).ifPresent(routeWaypoints::add);
        }
      }
      return Optional.of(
          new QueueStep(
              spotId,
              blankToNull(dto.requestedState),
              blankToNull(dto.label),
              blankToNull(dto.group),
              blankToNull(dto.alliance),
              xMeters,
              yMeters,
              headingDeg,
              sanitizeFinite(dto.constraintFactor),
              sanitizeFinite(dto.toleranceMeters),
              sanitizeFinite(dto.timeoutSeconds),
              sanitizeFinite(dto.endVelocityMps),
              dto.stopOnEnd,
              List.copyOf(routeWaypoints)));
    }

    static QueueStep fromLibrary(PathPlanAAutoLibrary.StepSpec step) {
      ArrayList<PoseSpec> routeWaypoints = new ArrayList<>();
      for (PathPlanAAutoLibrary.PoseSpec waypoint : step.routeWaypoints()) {
        PoseSpec.fromLibrary(waypoint).ifPresent(routeWaypoints::add);
      }
      return new QueueStep(
          step.spotId(),
          step.requestedState(),
          step.label(),
          step.group(),
          step.alliance(),
          step.xMeters(),
          step.yMeters(),
          step.headingDeg(),
          step.constraintFactor(),
          step.toleranceMeters(),
          step.timeoutSeconds(),
          step.endVelocityMps(),
          step.stopOnEnd(),
          List.copyOf(routeWaypoints));
    }

    Optional<Pose2d> resolvePose(RebuiltSpotLibrary spotLibrary, Alliance alliance) {
      if (xMeters != null && yMeters != null) {
        double heading = headingDeg != null ? headingDeg.doubleValue() : 0.0;
        return new PoseSpec(xMeters, yMeters, heading).resolvePose(spotLibrary, alliance);
      }
      if (spotId == null) {
        return Optional.empty();
      }
      return spotLibrary
          .getSpot(spotId)
          .map(RebuiltSpot::toPose)
          .map(
              pose ->
                  alliance == Alliance.Red
                      ? new Pose2d(
                          spotLibrary.getFieldLengthMeters() - pose.getX(),
                          pose.getY(),
                          headingDegrees(mirrorHeading(pose.getRotation().getDegrees())))
                      : pose);
    }

    Optional<SuperState> resolveRequestedState() {
      if (requestedStateName == null || requestedStateName.isBlank()) {
        return Optional.empty();
      }
      try {
        return Optional.of(SuperState.valueOf(requestedStateName.trim().toUpperCase(Locale.ROOT)));
      } catch (IllegalArgumentException ex) {
        return Optional.empty();
      }
    }

    String displayLabel(RebuiltSpotLibrary spotLibrary) {
      if (label != null) {
        return requestedStateName == null ? label : label + " • " + requestedStateName;
      }
      if (spotId == null) {
        return requestedStateName == null ? "Custom Pose" : "Custom Pose • " + requestedStateName;
      }
      return spotLibrary
          .getSpot(spotId)
          .map(
              spot ->
                  requestedStateName == null
                      ? spot.displayLabel()
                      : spot.displayLabel() + " • " + requestedStateName)
          .orElse(spotId);
    }
  }

  private static final class NoGoZone {
    private final String id;
    private final String label;
    private final double xMinMeters;
    private final double yMinMeters;
    private final double xMaxMeters;
    private final double yMaxMeters;
    private final boolean locked;

    private NoGoZone(
        String id,
        String label,
        double xMinMeters,
        double yMinMeters,
        double xMaxMeters,
        double yMaxMeters,
        boolean locked) {
      this.id = id;
      this.label = label;
      this.xMinMeters = Math.min(xMinMeters, xMaxMeters);
      this.yMinMeters = Math.min(yMinMeters, yMaxMeters);
      this.xMaxMeters = Math.max(xMinMeters, xMaxMeters);
      this.yMaxMeters = Math.max(yMinMeters, yMaxMeters);
      this.locked = locked;
    }

    static Optional<NoGoZone> fromDto(NoGoZoneDto dto) {
      if (dto == null) {
        return Optional.empty();
      }
      Double xMin = sanitizeFinite(dto.xMinMeters);
      Double yMin = sanitizeFinite(dto.yMinMeters);
      Double xMax = sanitizeFinite(dto.xMaxMeters);
      Double yMax = sanitizeFinite(dto.yMaxMeters);
      if (xMin == null || yMin == null || xMax == null || yMax == null) {
        return Optional.empty();
      }
      return Optional.of(
          new NoGoZone(
              blankToNull(dto.id), blankToNull(dto.label), xMin, yMin, xMax, yMax, dto.locked));
    }

    static NoGoZone fromLibrary(PathPlanAAutoLibrary.ZoneSpec zone) {
      return new NoGoZone(
          blankToNull(zone.id()),
          blankToNull(zone.label()),
          zone.xMinMeters(),
          zone.yMinMeters(),
          zone.xMaxMeters(),
          zone.yMaxMeters(),
          zone.locked());
    }

    NoGoZone resolveForAlliance(RebuiltSpotLibrary spotLibrary, Alliance alliance) {
      if (alliance != Alliance.Red) {
        return this;
      }
      return new NoGoZone(
          id,
          label,
          spotLibrary.getFieldLengthMeters() - xMaxMeters,
          yMinMeters,
          spotLibrary.getFieldLengthMeters() - xMinMeters,
          yMaxMeters,
          locked);
    }

    boolean contains(Pose2d pose) {
      double x = pose.getX();
      double y = pose.getY();
      return x >= xMinMeters - EPSILON_METERS
          && x <= xMaxMeters + EPSILON_METERS
          && y >= yMinMeters - EPSILON_METERS
          && y <= yMaxMeters + EPSILON_METERS;
    }

    boolean segmentIntersects(Pose2d start, Pose2d end) {
      double x1 = start.getX();
      double y1 = start.getY();
      double x2 = end.getX();
      double y2 = end.getY();
      if (Math.max(x1, x2) < xMinMeters - EPSILON_METERS
          || Math.min(x1, x2) > xMaxMeters + EPSILON_METERS
          || Math.max(y1, y2) < yMinMeters - EPSILON_METERS
          || Math.min(y1, y2) > yMaxMeters + EPSILON_METERS) {
        return false;
      }
      return segmentsIntersect(x1, y1, x2, y2, xMinMeters, yMinMeters, xMaxMeters, yMinMeters)
          || segmentsIntersect(x1, y1, x2, y2, xMaxMeters, yMinMeters, xMaxMeters, yMaxMeters)
          || segmentsIntersect(x1, y1, x2, y2, xMaxMeters, yMaxMeters, xMinMeters, yMaxMeters)
          || segmentsIntersect(x1, y1, x2, y2, xMinMeters, yMaxMeters, xMinMeters, yMinMeters);
    }
  }

  private static double cross(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  }

  private static boolean onSegment(
      double ax, double ay, double bx, double by, double px, double py) {
    return px >= Math.min(ax, bx) - EPSILON_METERS
        && px <= Math.max(ax, bx) + EPSILON_METERS
        && py >= Math.min(ay, by) - EPSILON_METERS
        && py <= Math.max(ay, by) + EPSILON_METERS;
  }

  private static boolean segmentsIntersect(
      double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
    double abC = cross(ax, ay, bx, by, cx, cy);
    double abD = cross(ax, ay, bx, by, dx, dy);
    double cdA = cross(cx, cy, dx, dy, ax, ay);
    double cdB = cross(cx, cy, dx, dy, bx, by);

    if ((abC > EPSILON_METERS && abD < -EPSILON_METERS
            || abC < -EPSILON_METERS && abD > EPSILON_METERS)
        && (cdA > EPSILON_METERS && cdB < -EPSILON_METERS
            || cdA < -EPSILON_METERS && cdB > EPSILON_METERS)) {
      return true;
    }

    if (Math.abs(abC) <= EPSILON_METERS && onSegment(ax, ay, bx, by, cx, cy)) {
      return true;
    }
    if (Math.abs(abD) <= EPSILON_METERS && onSegment(ax, ay, bx, by, dx, dy)) {
      return true;
    }
    if (Math.abs(cdA) <= EPSILON_METERS && onSegment(cx, cy, dx, dy, ax, ay)) {
      return true;
    }
    return Math.abs(cdB) <= EPSILON_METERS && onSegment(cx, cy, dx, dy, bx, by);
  }

  private static final class QueueSpecPayload {
    public Integer revision;
    public PoseDto startPose;
    public List<NoGoZoneDto> noGoZones = List.of();
    public List<QueueStepDto> steps = List.of();
  }

  private static final class PoseDto {
    public Double xMeters;
    public Double yMeters;
    public Double headingDeg;
  }

  private static final class NoGoZoneDto {
    public String id;
    public String label;
    public Double xMinMeters;
    public Double yMinMeters;
    public Double xMaxMeters;
    public Double yMaxMeters;
    public boolean locked;
  }

  private static final class QueueStepDto {
    public String spotId;
    public String requestedState;
    public String label;
    public String group;
    public String alliance;
    public Double xMeters;
    public Double yMeters;
    public Double headingDeg;
    public Double constraintFactor;
    public Double toleranceMeters;
    public Double timeoutSeconds;
    public Double endVelocityMps;
    public Boolean stopOnEnd;
    public List<PoseDto> routeWaypoints = List.of();
  }

  private static final class QueueCommandPayload {
    public String command;
  }

  record QueueActionTrace(
      double timestampSec, String action, boolean accepted, String detail, String queueMessage) {}

  private record SelectedAutoState(
      String id,
      String name,
      String folder,
      String relativePath,
      boolean loaded,
      String message,
      boolean selected,
      int stepCount,
      double timestampSec,
      PoseState startPose) {}

  private record QuickRunItem(String label, String status, String detail) {}

  private record QuickRunReport(
      double timestampSec, String status, String summary, ArrayList<QuickRunItem> items) {}

  private record PoseState(Double xMeters, Double yMeters, Double headingDeg) {}

  private record NoGoZoneState(
      String id,
      String label,
      Double xMinMeters,
      Double yMinMeters,
      Double xMaxMeters,
      Double yMaxMeters,
      boolean locked) {}

  private record QueueStateStep(
      String spotId,
      String label,
      String requestedState,
      String status,
      String group,
      String alliance,
      Double xMeters,
      Double yMeters,
      Double headingDeg,
      List<PoseState> routeWaypoints) {}

  private record QueueStatePayload(
      String selectedAutoId,
      String selectedAutoName,
      int revision,
      boolean running,
      String phase,
      int activeIndex,
      String message,
      String activeLabel,
      PoseState startPose,
      List<NoGoZoneState> noGoZones,
      List<QueueStateStep> steps) {}
}
