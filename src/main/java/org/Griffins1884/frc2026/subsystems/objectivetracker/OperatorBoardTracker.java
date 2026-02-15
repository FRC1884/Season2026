package org.Griffins1884.frc2026.subsystems.objectivetracker;

import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.Griffins1884.frc2026.Config;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.Superstructure.SuperState;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.Griffins1884.frc2026.util.HubShiftTracker;
import org.Griffins1884.frc2026.util.LogRollover;
import org.littletonrobotics.junction.Logger;

public class OperatorBoardTracker extends SubsystemBase implements AutoCloseable {
  private final OperatorBoardIO io;
  private final OperatorBoardIOInputsAutoLogged inputs = new OperatorBoardIOInputsAutoLogged();
  private final Superstructure superstructure;
  private final SwerveSubsystem drive;
  private final TurretSubsystem turret;
  private final OperatorBoardWebServer webServer;

  private String lastRequestedState = "";
  private boolean lastRequestAccepted = true;
  private String lastRequestReason = "";

  public OperatorBoardTracker(OperatorBoardIO io, Superstructure superstructure) {
    this(io, superstructure, null, null);
  }

  public OperatorBoardTracker(
      OperatorBoardIO io, Superstructure superstructure, SwerveSubsystem drive) {
    this(io, superstructure, drive, null);
  }

  public OperatorBoardTracker(
      OperatorBoardIO io,
      Superstructure superstructure,
      SwerveSubsystem drive,
      TurretSubsystem turret) {
    this.io = Objects.requireNonNull(io, "io");
    this.superstructure = superstructure;
    this.drive = drive;
    this.turret = turret;
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
    publishTelemetry();
  }

  private void handleStateRequests() {
    if (inputs.playSwerveMusicRequested) {
      if (drive != null) {
        drive.playSwerveMusic();
        lastRequestedState = "SWERVE_MUSIC_PLAY";
        lastRequestAccepted = true;
        lastRequestReason = "";
      } else {
        lastRequestedState = "SWERVE_MUSIC_PLAY";
        lastRequestAccepted = false;
        lastRequestReason = "Drive unavailable";
      }
    }
    if (inputs.stopSwerveMusicRequested) {
      if (drive != null) {
        drive.stopSwerveMusic();
        lastRequestedState = "SWERVE_MUSIC_STOP";
        lastRequestAccepted = true;
        lastRequestReason = "";
      } else {
        lastRequestedState = "SWERVE_MUSIC_STOP";
        lastRequestAccepted = false;
        lastRequestReason = "Drive unavailable";
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
      lastRequestReason = rolled ? "" : "Log rollover unavailable";
    }
    if (inputs.autoStateEnableRequested) {
      if (superstructure == null) {
        lastRequestedState = "AUTO";
        lastRequestAccepted = false;
        lastRequestReason = "Superstructure unavailable";
      } else {
        superstructure.setAutoStateEnabled(true);
        lastRequestedState = "AUTO";
        lastRequestAccepted = true;
        lastRequestReason = "";
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
      return;
    }

    lastRequestedState = parsed.name();

    if (superstructure == null) {
      lastRequestAccepted = false;
      lastRequestReason = "Superstructure unavailable";
      return;
    }

    Superstructure.StateRequestResult result = superstructure.requestStateFromDashboard(parsed);
    lastRequestAccepted = result.accepted();
    lastRequestReason = result.reason() == null ? "" : result.reason();
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
    String requestedState =
        !lastRequestedState.isBlank()
            ? lastRequestedState
            : superstructure != null ? superstructure.getRequestedState().name() : "UNKNOWN";
    io.setRequestedState(requestedState);
    io.setCurrentState(
        superstructure != null ? superstructure.getCurrentState().name() : "UNKNOWN");
    io.setRequestAccepted(lastRequestAccepted);
    io.setRequestReason(lastRequestReason);

    io.setClimbPhase(superstructure != null ? superstructure.getClimbPhaseName() : "UNKNOWN");
    io.setClimbLevel(superstructure != null ? superstructure.getClimbLevel() : 0);

    TargetSnapshot target = computeTargetSnapshot();
    io.setTargetType(target.type());
    io.setTargetPose(target.pose());
    io.setTargetPoseValid(target.valid());

    io.setRobotPose(computeRobotPose());

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
    io.setSysIdTurnPhase(drive != null ? drive.getTurnSysIdPhase() : "UNAVAILABLE");
    io.setSysIdTurnActive(drive != null && drive.isTurnSysIdActive());
    io.setSysIdTurnLastCompleted(drive != null ? drive.getTurnSysIdLastCompleted() : Double.NaN);
    io.setVisionStatus(Config.Subsystems.VISION_ENABLED ? "ENABLED" : "DISABLED");
  }

  private TargetSnapshot computeTargetSnapshot() {
    if (superstructure == null) {
      return TargetSnapshot.unknown();
    }
    SuperState state = superstructure.getCurrentState();
    Translation2d target = null;
    String targetType = "NONE";

    switch (state) {
      case SHOOTING -> {
        targetType = "HOPPER";
        target =
            (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                ? GlobalConstants.FieldConstants.Hub.topCenterPoint.toTranslation2d()
                : GlobalConstants.FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
      }
      case FERRYING -> {
        targetType = "FERRY";
        Pose2d pose = drive != null ? drive.getPose() : null;
        if (pose != null) {
          target = new Translation2d(0.0, 0.0); // TODO: find Ferry Target
        }
      }
      case ENDGAME_CLIMB, AUTO_CLIMB, CLIMB_DETACH -> {
        targetType = "CLIMB";
        target =
            (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                ? GlobalConstants.FieldConstants.Tower.centerPoint
                : GlobalConstants.FieldConstants.Tower.oppCenterPoint;
      }
      default -> targetType = "NONE";
    }

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

  private static final class OperatorBoardWebServer implements AutoCloseable {
    private final Path webRoot;
    private final Path musicDir;
    private final HttpServer server;
    private final ExecutorService executor;

    OperatorBoardWebServer(Path webRoot, Path musicDir, String bindAddress, int port)
        throws IOException {
      this.webRoot = Objects.requireNonNull(webRoot, "webRoot").toAbsolutePath().normalize();
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
      server.createContext("/music-upload", new MusicUploadHandler());
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
