package org.Griffins1884.frc2026.subsystems.objectivetracker;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import org.json.simple.parser.ParseException;

final class DeployAutoLibrary {
  private static final ObjectMapper JSON =
      new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
  private static final int MAX_ROUTE_WAYPOINTS = 12;

  private final Path pathPlannerAutosRoot;

  DeployAutoLibrary(Path pathPlannerAutosRoot) {
    this.pathPlannerAutosRoot =
        Objects.requireNonNull(pathPlannerAutosRoot, "pathPlannerAutosRoot")
            .toAbsolutePath()
            .normalize();
  }

  Optional<LoadedAuto> loadAuto(String autoId) {
    String normalizedId = blankToNull(autoId);
    if (normalizedId == null) {
      return Optional.empty();
    }
    return findManifestEntry(normalizedId).flatMap(this::loadAuto);
  }

  Optional<Command> buildAutoCommand(String autoId) {
    String normalizedId = blankToNull(autoId);
    if (normalizedId == null) {
      return Optional.empty();
    }
    if (!AutoBuilder.isConfigured()) {
      DriverStation.reportError(
          "PathPlanner AutoBuilder is not configured. Cannot build auto \"" + normalizedId + "\".",
          false);
      return Optional.empty();
    }
    try {
      return Optional.of(new PathPlannerAuto(normalizedId));
    } catch (RuntimeException ex) {
      DriverStation.reportError(
          "Failed to build PathPlanner auto \"" + normalizedId + "\": " + ex.getMessage(),
          ex.getStackTrace());
      return Optional.empty();
    }
  }

  String buildManifestJson() {
    try {
      return JSON.writeValueAsString(
          new LibraryIndexDto("2026.0", "DeployAutoLibrary", listAutos()));
    } catch (JsonProcessingException ex) {
      DriverStation.reportError("Failed to serialize deploy auto manifest", ex.getStackTrace());
      return "{\"autos\":[]}";
    }
  }

  Optional<String> loadAutoPreviewJson(String relativePath) {
    String normalized = blankToNull(relativePath);
    if (normalized == null) {
      return Optional.empty();
    }
    String autoId =
        normalized.toLowerCase(Locale.ROOT).endsWith(".json")
            ? normalized.substring(0, normalized.length() - 5)
            : normalized;
    Optional<LoadedAuto> loadedAuto = loadAuto(autoId);
    if (loadedAuto.isEmpty()) {
      return Optional.empty();
    }
    try {
      return Optional.of(JSON.writeValueAsString(loadedAuto.get()));
    } catch (JsonProcessingException ex) {
      DriverStation.reportError(
          "Failed to serialize deploy auto preview \"" + normalized + "\"", ex.getStackTrace());
      return Optional.empty();
    }
  }

  private List<AutoManifestEntryDto> listAutos() {
    if (!Files.isDirectory(pathPlannerAutosRoot)) {
      return List.of();
    }
    try (var files = Files.walk(pathPlannerAutosRoot)) {
      return files
          .filter(Files::isRegularFile)
          .filter(path -> path.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".auto"))
          .sorted(
              Comparator.comparing(path -> path.getFileName().toString().toLowerCase(Locale.ROOT)))
          .map(this::toManifestEntry)
          .filter(Objects::nonNull)
          .toList();
    } catch (IOException ex) {
      DriverStation.reportError(
          "Failed to enumerate PathPlanner autos: " + ex.getMessage(), ex.getStackTrace());
      return List.of();
    }
  }

  private AutoManifestEntryDto toManifestEntry(Path autoFile) {
    String autoId = autoName(autoFile);
    if (autoId == null) {
      return null;
    }
    RawAutoDto raw = readRawAuto(autoFile).orElse(null);
    String folder = raw != null ? blankToNull(raw.folder) : null;
    if (folder == null) {
      Path parent = autoFile.getParent();
      if (parent != null && !parent.equals(pathPlannerAutosRoot)) {
        folder = blankToNull(pathPlannerAutosRoot.relativize(parent).toString().replace('\\', '/'));
      }
    }
    long updatedAt = 0L;
    try {
      updatedAt = Files.getLastModifiedTime(autoFile).toMillis();
    } catch (IOException ex) {
      DriverStation.reportWarning(
          "Failed to read modified time for PathPlanner auto \"" + autoFile + "\".", false);
    }
    return new AutoManifestEntryDto(autoId, autoId, folder, autoId + ".json", updatedAt);
  }

  private Optional<AutoManifestEntryDto> findManifestEntry(String autoId) {
    return listAutos().stream().filter(entry -> autoId.equals(entry.id)).findFirst();
  }

  private Optional<LoadedAuto> loadAuto(AutoManifestEntryDto entry) {
    Path autoFile = pathPlannerAutosRoot.resolve(entry.id + ".auto").normalize().toAbsolutePath();
    if (!autoFile.startsWith(pathPlannerAutosRoot) || !Files.isRegularFile(autoFile)) {
      return Optional.empty();
    }
    try {
      List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(entry.id);
      PoseSpec startPose = resolveStartPose(pathGroup).orElse(null);
      List<StepSpec> steps = buildPreviewSteps(pathGroup);
      return Optional.of(
          new LoadedAuto(
              entry.id,
              entry.name,
              entry.folder,
              entry.relativePath,
              entry.updatedAt,
              startPose,
              List.of(),
              steps));
    } catch (IOException | ParseException ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner auto \"" + entry.id + "\": " + ex.getMessage(),
          ex.getStackTrace());
      return Optional.empty();
    }
  }

  private static Optional<RawAutoDto> readRawAuto(Path autoFile) {
    try {
      return Optional.of(JSON.readValue(autoFile.toFile(), RawAutoDto.class));
    } catch (IOException ex) {
      DriverStation.reportWarning(
          "Failed to parse PathPlanner auto metadata from \"" + autoFile + "\".", false);
      return Optional.empty();
    }
  }

  private static Optional<PoseSpec> resolveStartPose(List<PathPlannerPath> pathGroup) {
    for (PathPlannerPath path : pathGroup) {
      if (path == null) {
        continue;
      }
      Optional<Pose2d> startingPose = path.getStartingHolonomicPose();
      if (startingPose.isPresent()) {
        return Optional.of(fromPose(startingPose.get()));
      }
      List<Pose2d> poses = path.getPathPoses();
      if (!poses.isEmpty()) {
        return Optional.of(fromPose(poses.get(0)));
      }
    }
    return Optional.empty();
  }

  private static List<StepSpec> buildPreviewSteps(List<PathPlannerPath> pathGroup) {
    ArrayList<StepSpec> steps = new ArrayList<>();
    for (PathPlannerPath path : pathGroup) {
      if (path == null) {
        continue;
      }
      List<Pose2d> poses = path.getPathPoses();
      if (poses.isEmpty()) {
        continue;
      }
      Pose2d finalPose = poses.get(poses.size() - 1);
      steps.add(
          new StepSpec(
              null,
              null,
              blankToNull(path.name) != null ? blankToNull(path.name) : "Path Segment",
              "PATH",
              null,
              finalPose.getX(),
              finalPose.getY(),
              finalPose.getRotation().getDegrees(),
              null,
              null,
              null,
              null,
              null,
              sampleIntermediatePoses(poses)));
    }
    return List.copyOf(steps);
  }

  private static List<PoseSpec> sampleIntermediatePoses(List<Pose2d> poses) {
    if (poses.size() <= 2) {
      return List.of();
    }
    int interiorCount = poses.size() - 2;
    int sampleCount = Math.min(interiorCount, MAX_ROUTE_WAYPOINTS);
    ArrayList<PoseSpec> samples = new ArrayList<>(sampleCount);
    for (int i = 1; i <= sampleCount; i++) {
      int poseIndex = (int) Math.round(((double) i * (poses.size() - 1)) / (sampleCount + 1));
      poseIndex = Math.max(1, Math.min(poses.size() - 2, poseIndex));
      samples.add(fromPose(poses.get(poseIndex)));
    }
    return List.copyOf(samples);
  }

  private static PoseSpec fromPose(Pose2d pose) {
    return new PoseSpec(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  private static String autoName(Path autoFile) {
    String fileName = autoFile.getFileName().toString();
    if (!fileName.toLowerCase(Locale.ROOT).endsWith(".auto")) {
      return null;
    }
    return fileName.substring(0, fileName.length() - 5);
  }

  private static String blankToNull(String value) {
    return value == null || value.isBlank() ? null : value.trim();
  }

  record LoadedAuto(
      String id,
      String name,
      String folder,
      String relativePath,
      Long updatedAt,
      PoseSpec startPose,
      List<ZoneSpec> customZones,
      List<StepSpec> steps) {}

  record PoseSpec(double xMeters, double yMeters, double headingDeg) {}

  record ZoneSpec(
      String id,
      String label,
      double xMinMeters,
      double yMinMeters,
      double xMaxMeters,
      double yMaxMeters,
      boolean locked) {}

  record StepSpec(
      String spotId,
      String requestedState,
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
      List<PoseSpec> routeWaypoints) {}

  private record LibraryIndexDto(
      String version, String generator, List<AutoManifestEntryDto> autos) {}

  private record AutoManifestEntryDto(
      String id, String name, String folder, String relativePath, Long updatedAt) {}

  private static final class RawAutoDto {
    public String folder;
  }
}
