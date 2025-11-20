package frc.robot.subsystems.objectivetracker;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.NamedTargets;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.littletonrobotics.junction.Logger;

public class TabletInterfaceTracker extends SubsystemBase implements AutoCloseable {
  private static final ObjectMapper JSON =
      new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

  private final ReefControlsIO io;
  private final ReefControlsIOInputsAutoLogged inputs = new ReefControlsIOInputsAutoLogged();
  private final SwerveSubsystem drive;
  private final TabletWebServer webServer;

  // Legacy reef controls state mirrored back to the web UI
  private int selectedLevel = 2;
  private int level1Coral = 0;
  private int level2Coral = 0;
  private int level3Coral = 0;
  private int level4Coral = 0;
  private int algae = 0;
  private boolean coop = false;

  // Queue + autonomy state
  private final List<QueueStep> queueSteps = new ArrayList<>();
  private int queueRevision = 0;
  private boolean queueRunning = false;
  private boolean manualOverride = false;
  private QueuePhase phase = QueuePhase.IDLE;
  private Command activeCommand;
  private String activeLabel = "";
  private String queueMessage = "Tap a reef node to build the queue.";
  private String lastQueueStateJson = "";
  private ApproachSide preferredSourceSide = ApproachSide.LEFT;

  public TabletInterfaceTracker(ReefControlsIO io) {
    this(io, null);
  }

  public TabletInterfaceTracker(ReefControlsIO io, SwerveSubsystem drive) {
    this.io = Objects.requireNonNull(io, "io");
    this.drive = drive;
    this.webServer = maybeStartWebServer();
  }

  private TabletWebServer maybeStartWebServer() {
    if (!Config.WebUIConfig.ENABLED) {
      return null;
    }
    try {
      TabletWebServer server =
          new TabletWebServer(
              Filesystem.getDeployDirectory().toPath().resolve("reefcontrols"),
              Config.WebUIConfig.BIND_ADDRESS,
              Config.WebUIConfig.PORT);
      server.start();
      return server;
    } catch (IOException ex) {
      DriverStation.reportError("Failed to start tablet web server", ex.getStackTrace());
      return null;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ReefControls", inputs);

    updateReefControlsMirror();
    handleQueueInputs();
    updateQueueExecution();
    publishQueueState();
    logQueueState();
  }

  private void updateReefControlsMirror() {
    if (inputs.selectedLevel.length > 0) {
      selectedLevel = inputs.selectedLevel[0];
    }
    if (inputs.level1State.length > 0) {
      level1Coral = inputs.level1State[0];
    }
    if (inputs.level2State.length > 0) {
      level2Coral = inputs.level2State[0];
    }
    if (inputs.level3State.length > 0) {
      level3Coral = inputs.level3State[0];
    }
    if (inputs.level4State.length > 0) {
      level4Coral = inputs.level4State[0];
    }
    if (inputs.algaeState.length > 0) {
      algae = inputs.algaeState[0];
    }
    if (inputs.coopState.length > 0) {
      coop = inputs.coopState[0];
    }

    io.setSelectedLevel(selectedLevel);
    io.setLevel1State(level1Coral);
    io.setLevel2State(level2Coral);
    io.setLevel3State(level3Coral);
    io.setLevel4State(level4Coral);
    io.setAlgaeState(algae);
    io.setCoopState(coop);
  }

  private void handleQueueInputs() {
    if (inputs.queueSpec.length > 0) {
      applyQueueSpec(inputs.queueSpec[0]);
    }
    if (inputs.queueCommand.length > 0) {
      applyQueueCommand(inputs.queueCommand[0]);
    }
  }

  private void applyQueueSpec(String raw) {
    if (raw == null || raw.isBlank()) {
      clearQueue("Queue cleared.");
      return;
    }
    try {
      QueueSpecPayload payload = JSON.readValue(raw, QueueSpecPayload.class);
      int revision = payload.revision() == null ? queueRevision : payload.revision();
      List<QueueStep> parsed = new ArrayList<>();
      if (payload.steps() != null) {
        for (QueueStepDto dto : payload.steps()) {
          if (dto == null) {
            continue;
          }
          QueueStep.fromDto(dto).ifPresent(parsed::add);
        }
      }
      queueSteps.clear();
      queueSteps.addAll(parsed);
      updatePreferredSourceSideFromQueue();
      queueRevision = revision;
      cancelActiveCommand();
      queueRunning = false;
      phase = QueuePhase.IDLE;
      activeLabel = "";
      queueMessage = queueSteps.isEmpty() ? "Queue cleared." : "Queue updated. Press Start to run.";
    } catch (Exception ex) {
      DriverStation.reportError(
          "Tablet queue spec parse failed: " + ex.getMessage(), ex.getStackTrace());
    }
  }

  private void applyQueueCommand(String raw) {
    if (raw == null || raw.isBlank()) {
      return;
    }
    try {
      QueueCommandPayload payload = JSON.readValue(raw, QueueCommandPayload.class);
      QueueCommand command = QueueCommand.from(payload.command());
      if (command == null) {
        return;
      }
      switch (command) {
        case START -> startQueue();
        case STOP -> stopQueue("Queue paused.");
        case SKIP -> skipStep();
        case RESET -> clearQueue("Queue reset.");
        case ADD -> enqueueStep(payload.step());
        case REMOVE -> removeStep(payload.index());
        case MOVE -> moveStep(payload.fromIndex(), payload.toIndex());
        case MANUAL -> setManualOverride(payload.manual());
        case ALIGN_SOURCE -> alignSource(SourcePreference.parse(payload.source()));
      }
    } catch (Exception ex) {
      DriverStation.reportError(
          "Tablet queue command parse failed: " + ex.getMessage(), ex.getStackTrace());
    }
  }

  private void startQueue() {
    if (manualOverride) {
      queueMessage = "Manual override enabled. Disable it to run the queue.";
      queueRunning = false;
      phase = QueuePhase.MANUAL;
      return;
    }
    if (queueSteps.isEmpty()) {
      queueMessage = "Queue is empty. Tap a reef to add one.";
      queueRunning = false;
      phase = QueuePhase.IDLE;
      return;
    }
    queueRunning = true;
    phase = QueuePhase.RUNNING;
    queueMessage = "Executing queue.";
    if (activeCommand == null) {
      startNextStep();
    }
  }

  private void stopQueue(String message) {
    queueRunning = false;
    phase = manualOverride ? QueuePhase.MANUAL : QueuePhase.IDLE;
    queueMessage = message;
    cancelActiveCommand();
  }

  private void skipStep() {
    if (queueSteps.isEmpty()) {
      queueMessage = "Nothing to skip.";
      return;
    }
    removeStepInternal(0, true);
  }

  private void clearQueue(String message) {
    queueSteps.clear();
    queueRevision++;
    cancelActiveCommand();
    queueRunning = false;
    phase = manualOverride ? QueuePhase.MANUAL : QueuePhase.IDLE;
    activeLabel = "";
    preferredSourceSide = ApproachSide.LEFT;
    queueMessage = message;
  }

  private void enqueueStep(QueueStepDto dto) {
    if (dto == null) {
      queueMessage = "Ignored empty queue step.";
      return;
    }
    QueueStep.fromDto(dto)
        .ifPresent(
            step -> {
              queueSteps.add(step);
              queueRevision++;
              if (step.type == StepType.REEF && step.side != null) {
                preferredSourceSide = step.side;
              }
              queueMessage = "Queued " + step.label() + ".";
              if (queueRunning && activeCommand == null) {
                startNextStep();
              }
            });
  }

  private void removeStep(Integer indexValue) {
    if (indexValue == null) {
      queueMessage = "No step selected.";
      return;
    }
    removeStepInternal(indexValue.intValue(), false);
  }

  private void removeStepInternal(int index, boolean fromSkip) {
    if (index < 0 || index >= queueSteps.size()) {
      queueMessage = "No step found.";
      return;
    }
    boolean wasRunning = queueRunning;
    boolean removingActive = wasRunning && index == 0;
    QueueStep removed = queueSteps.remove(index);
    queueRevision++;
    if (removingActive) {
      cancelActiveCommand();
    }

    if (queueSteps.isEmpty()) {
      queueRunning = false;
      phase = manualOverride ? QueuePhase.MANUAL : QueuePhase.IDLE;
      activeLabel = "";
      queueMessage = fromSkip ? "Skipped last task." : "Removed " + removed.label() + ".";
      preferredSourceSide = ApproachSide.LEFT;
      return;
    }

    if (removingActive && wasRunning) {
      queueMessage = fromSkip ? "Skipped to next task." : "Removed active task.";
      if (queueRunning) {
        startNextStep();
      }
    } else {
      queueMessage = fromSkip ? "Skipped task." : "Removed " + removed.label() + ".";
    }
    updatePreferredSourceSideFromQueue();
  }

  private void moveStep(Integer fromIndexValue, Integer toIndexValue) {
    if (fromIndexValue == null || toIndexValue == null) {
      queueMessage = "Move request missing index.";
      return;
    }
    if (queueSteps.size() < 2) {
      queueMessage = "Not enough steps to reorder.";
      return;
    }
    int size = queueSteps.size();
    int from = Math.max(0, Math.min(size - 1, fromIndexValue.intValue()));
    int to = Math.max(0, Math.min(size, toIndexValue.intValue()));
    if (from == to) {
      queueMessage = "Step already in that position.";
      return;
    }
    QueueStep moved = queueSteps.remove(from);
    if (to > queueSteps.size()) {
      to = queueSteps.size();
    }
    queueSteps.add(to, moved);
    queueRevision++;
    updatePreferredSourceSideFromQueue();
    boolean movedHead = from == 0 || to == 0;
    if (movedHead) {
      cancelActiveCommand();
      activeLabel = "";
      if (queueRunning) {
        startNextStep();
      } else {
        queueMessage = "Queue reordered.";
      }
    } else {
      queueMessage = "Moved " + moved.label() + ".";
    }
  }

  private void setManualOverride(Boolean manualValue) {
    if (manualValue == null) {
      queueMessage =
          manualOverride ? "Manual override already enabled." : "Manual override already disabled.";
      return;
    }
    boolean enable = manualValue.booleanValue();
    if (enable == manualOverride) {
      queueMessage =
          enable ? "Manual override already enabled." : "Manual override already disabled.";
      return;
    }
    manualOverride = enable;
    if (manualOverride) {
      stopQueue("Manual override enabled. Queue paused.");
      activeLabel = "";
    } else {
      if (!queueRunning) {
        phase = queueSteps.isEmpty() ? QueuePhase.IDLE : QueuePhase.IDLE;
      }
      queueMessage = "Manual override disabled. Queue ready.";
    }
  }

  private void alignSource(SourcePreference sourcePreference) {
    SourcePreference pref = sourcePreference == null ? SourcePreference.NEAREST : sourcePreference;
    QueueStepDto dto = new QueueStepDto("SOURCE", pref.name(), null, null, null);
    QueueStep.fromDto(dto)
        .ifPresent(
            step -> {
              queueSteps.add(step);
              queueRevision++;
              queueMessage = "Queued " + step.label() + ".";
              if (queueRunning && activeCommand == null) {
                startNextStep();
              } else if (!queueRunning && !manualOverride) {
                startQueue();
              }
            });
  }

  private void updatePreferredSourceSideFromQueue() {
    preferredSourceSide =
        queueSteps.stream()
            .filter(step -> step.type == StepType.REEF && step.side != null)
            .map(step -> step.side)
            .findFirst()
            .orElse(ApproachSide.LEFT);
  }

  private void updateQueueExecution() {
    if (queueRunning && drive == null) {
      DriverStation.reportWarning("Tablet queue cannot run without a drive subsystem", false);
      stopQueue("Drive not available.");
      return;
    }

    if (queueRunning && activeCommand == null) {
      startNextStep();
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
    if (step.type == StepType.REEF && step.side != null) {
      preferredSourceSide = step.side;
    }
    Command command = buildCommand(step);
    if (command == null) {
      queueMessage = "Failed to build command for " + step.label() + ".";
      queueSteps.remove(0);
      queueRevision++;
      if (queueSteps.isEmpty()) {
        finishQueue();
      } else {
        startNextStep();
      }
      return;
    }
    cancelActiveCommand();
    activeCommand = command;
    CommandScheduler.getInstance().schedule(command);
    activeLabel = step.label();
    queueMessage = "Aligning to " + activeLabel;
    phase = QueuePhase.RUNNING;
  }

  private void finishQueue() {
    cancelActiveCommand();
    queueRunning = false;
    if (queueSteps.isEmpty()) {
      phase = manualOverride ? QueuePhase.MANUAL : QueuePhase.COMPLETE;
      queueMessage = "Queue finished.";
      preferredSourceSide = ApproachSide.LEFT;
      activeLabel = "";
    } else {
      phase = manualOverride ? QueuePhase.MANUAL : QueuePhase.IDLE;
    }
  }

  private void cancelActiveCommand() {
    if (activeCommand != null) {
      if (CommandScheduler.getInstance().isScheduled(activeCommand)) {
        activeCommand.cancel();
      }
      activeCommand = null;
    }
  }

  private Command buildCommand(QueueStep step) {
    if (drive == null) {
      return null;
    }
    return switch (step.type) {
      case SOURCE -> {
        SourcePreference preference = step.source == null ? SourcePreference.NEAREST : step.source;
        if (preference == SourcePreference.NEAREST && preferredSourceSide != null) {
          preference =
              preferredSourceSide == ApproachSide.LEFT
                  ? SourcePreference.LEFT
                  : SourcePreference.RIGHT;
        }
        yield switch (preference) {
          case LEFT -> DriveCommands.alignToCoralStationCommandAuto(drive, true);
          case RIGHT -> DriveCommands.alignToCoralStationCommandAuto(drive, false);
          case NEAREST -> DriveCommands.alignToNearestCoralStationCommandAuto(drive);
        };
      }
      case REEF -> {
        String target = step.targetName();
        yield NamedTargets.goTo(drive, target);
      }
    };
  }

  private void publishQueueState() {
    String json = buildQueueStateJson();
    if (!json.equals(lastQueueStateJson)) {
      io.setQueueState(json);
      lastQueueStateJson = json;
    }
  }

  private String buildQueueStateJson() {
    try {
      List<QueueStateStep> steps = new ArrayList<>();
      for (int i = 0; i < queueSteps.size(); i++) {
        QueueStep step = queueSteps.get(i);
        StepStatus status = statusForIndex(i);
        steps.add(
            new QueueStateStep(
                step.type.name(),
                step.source == null ? null : step.source.name(),
                step.type == StepType.REEF ? step.face : null,
                step.type == StepType.REEF && step.side != null ? step.side.name() : null,
                step.type == StepType.REEF && step.level != null ? step.level.name() : null,
                step.label(),
                status.name()));
      }
      int activeIndex = queueRunning && !queueSteps.isEmpty() ? 0 : -1;
      QueueStatePayload payload =
          new QueueStatePayload(
              queueRevision,
              queueRunning,
              manualOverride,
              phase.name(),
              activeIndex,
              queueMessage,
              activeLabel,
              steps);
      return JSON.writeValueAsString(payload);
    } catch (JsonProcessingException ex) {
      DriverStation.reportError("Failed to serialize queue state", ex.getStackTrace());
      return "{}";
    }
  }

  private StepStatus statusForIndex(int index) {
    if (index < 0 || index >= queueSteps.size()) {
      return StepStatus.PENDING;
    }
    if (!queueRunning) {
      return StepStatus.PENDING;
    }
    if (index == 0) {
      return activeCommand != null ? StepStatus.ACTIVE : StepStatus.READY;
    }
    return StepStatus.QUEUED;
  }

  private void logQueueState() {
    Logger.recordOutput("TabletQueue/Running", queueRunning);
    Logger.recordOutput("TabletQueue/ManualOverride", manualOverride);
    Logger.recordOutput("TabletQueue/Phase", phase.name());
    Logger.recordOutput("TabletQueue/Length", queueSteps.size());
    Logger.recordOutput("TabletQueue/ActiveTarget", activeLabel);
  }

  @Override
  public void close() {
    if (webServer != null) {
      webServer.close();
    }
  }

  private enum QueuePhase {
    IDLE,
    RUNNING,
    COMPLETE,
    MANUAL
  }

  private enum StepType {
    SOURCE,
    REEF
  }

  private enum StepStatus {
    PENDING,
    QUEUED,
    READY,
    ACTIVE
  }

  private enum SourcePreference {
    LEFT,
    RIGHT,
    NEAREST;

    private static SourcePreference parse(String value) {
      if (value == null) {
        return null;
      }
      try {
        return SourcePreference.valueOf(value.trim().toUpperCase(Locale.ROOT));
      } catch (IllegalArgumentException ex) {
        return null;
      }
    }
  }

  private enum ApproachSide {
    LEFT,
    RIGHT;

    private static ApproachSide parse(String value) {
      if (value == null) {
        return null;
      }
      try {
        return ApproachSide.valueOf(value.trim().toUpperCase(Locale.ROOT));
      } catch (IllegalArgumentException ex) {
        return null;
      }
    }
  }

  private enum CoralLevel {
    L1,
    L2,
    L3,
    L4;

    private static CoralLevel parse(String value) {
      if (value == null) {
        return CoralLevel.L3;
      }
      try {
        return CoralLevel.valueOf(value.trim().toUpperCase(Locale.ROOT));
      } catch (IllegalArgumentException ex) {
        return CoralLevel.L3;
      }
    }
  }

  private enum QueueCommand {
    START,
    STOP,
    SKIP,
    RESET,
    ADD,
    REMOVE,
    MOVE,
    MANUAL,
    ALIGN_SOURCE;

    private static QueueCommand from(String value) {
      if (value == null) {
        return null;
      }
      try {
        return QueueCommand.valueOf(value.trim().toUpperCase(Locale.ROOT));
      } catch (IllegalArgumentException ex) {
        return null;
      }
    }
  }

  private static final class QueueStep {
    private final StepType type;
    private final SourcePreference source;
    private final int face;
    private final ApproachSide side;
    private final CoralLevel level;

    private QueueStep(QueueStepDto dto) {
      Objects.requireNonNull(dto, "dto");
      String typeValue = Objects.requireNonNull(dto.type(), "type");
      this.type = StepType.valueOf(typeValue.trim().toUpperCase(Locale.ROOT));
      if (type == StepType.SOURCE) {
        this.source =
            Optional.ofNullable(SourcePreference.parse(dto.source()))
                .orElse(SourcePreference.NEAREST);
        this.face = -1;
        this.side = null;
        this.level = null;
      } else {
        this.source = null;
        this.face = dto.face() == null ? 1 : Math.max(1, Math.min(6, dto.face()));
        this.side = Optional.ofNullable(ApproachSide.parse(dto.side())).orElse(ApproachSide.LEFT);
        this.level = CoralLevel.parse(dto.level());
      }
    }

    static Optional<QueueStep> fromDto(QueueStepDto dto) {
      try {
        return Optional.of(new QueueStep(dto));
      } catch (Exception ex) {
        DriverStation.reportWarning("Skipping invalid queue step: " + ex.getMessage(), false);
        return Optional.empty();
      }
    }

    private String label() {
      return switch (type) {
        case SOURCE -> "Source (" + source.name() + ")";
        case REEF -> "Reef F"
            + face
            + " "
            + (side == null ? "" : side.name())
            + (level == null ? "" : " " + level.name());
      };
    }

    private String targetName() {
      if (type != StepType.REEF) {
        return "";
      }
      String sideToken = side == ApproachSide.LEFT ? "L" : "R";
      return "Reef_" + face + "_" + sideToken + "_Blue";
    }
  }

  private record QueueSpecPayload(Integer revision, List<QueueStepDto> steps) {}

  private record QueueStepDto(
      String type, String source, Integer face, String side, String level) {}

  private record QueueCommandPayload(
      String command,
      Long at,
      QueueStepDto step,
      Integer index,
      Boolean manual,
      Integer fromIndex,
      Integer toIndex,
      String source) {}

  private record QueueStatePayload(
      int revision,
      boolean running,
      boolean manual,
      String phase,
      int activeIndex,
      String message,
      String activeStep,
      List<QueueStateStep> steps) {}

  private record QueueStateStep(
      String type,
      String source,
      Integer face,
      String side,
      String level,
      String label,
      String status) {}

  private static final class TabletWebServer implements AutoCloseable {
    private final Path webRoot;
    private final HttpServer server;
    private final ExecutorService executor;

    TabletWebServer(Path webRoot, String bindAddress, int port) throws IOException {
      this.webRoot = Objects.requireNonNull(webRoot, "webRoot").toAbsolutePath().normalize();
      String bind = (bindAddress == null || bindAddress.isBlank()) ? "0.0.0.0" : bindAddress;
      this.server = HttpServer.create(new InetSocketAddress(bind, port), 0);
      this.executor =
          Executors.newFixedThreadPool(
              2,
              r -> {
                Thread t = new Thread(r);
                t.setName("TabletWeb-" + t.getId());
                t.setDaemon(true);
                return t;
              });
      server.setExecutor(executor);
      server.createContext("/", new StaticHandler());
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
