package org.Griffins1884.frc2026.subsystems.objectivetracker;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

final class PathPlanAAutoLibrary {
  private static final ObjectMapper JSON =
      new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

  private final Path autosRoot;

  PathPlanAAutoLibrary(Path autosRoot) {
    this.autosRoot = Objects.requireNonNull(autosRoot, "autosRoot").toAbsolutePath().normalize();
  }

  Optional<LoadedAuto> loadAuto(String autoId) {
    String normalizedId = blankToNull(autoId);
    if (normalizedId == null) {
      return Optional.empty();
    }
    try {
      LibraryIndexDto index = loadIndex();
      if (index.autos == null) {
        return Optional.empty();
      }
      for (AutoManifestEntryDto entry : index.autos) {
        if (normalizedId.equals(blankToNull(entry.id))) {
          return loadAuto(entry);
        }
      }
    } catch (IOException ex) {
      reportError("Failed to load PathPlanA auto library index", ex);
    }
    return Optional.empty();
  }

  private Optional<LoadedAuto> loadAuto(AutoManifestEntryDto entry) {
    String relativePath = blankToNull(entry.relativePath);
    if (relativePath == null) {
      return Optional.empty();
    }
    Path target = autosRoot.resolve(relativePath).normalize().toAbsolutePath();
    if (!target.startsWith(autosRoot) || !Files.isRegularFile(target)) {
      return Optional.empty();
    }
    try {
      AutoFileDto dto = JSON.readValue(target.toFile(), AutoFileDto.class);
      return Optional.of(
          new LoadedAuto(
              blankToNull(dto.id) != null ? blankToNull(dto.id) : blankToNull(entry.id),
              blankToNull(dto.name) != null ? blankToNull(dto.name) : blankToNull(entry.name),
              blankToNull(dto.folder) != null ? blankToNull(dto.folder) : blankToNull(entry.folder),
              relativePath,
              dto.updatedAt != null ? dto.updatedAt : entry.updatedAt,
              toPose(dto.startPose),
              toZones(dto.customZones),
              toSteps(dto.steps)));
    } catch (IOException ex) {
      reportError("Failed to load PathPlanA auto \"" + relativePath + "\"", ex);
      return Optional.empty();
    }
  }

  private LibraryIndexDto loadIndex() throws IOException {
    Path indexPath = autosRoot.resolve("index.json").normalize().toAbsolutePath();
    return JSON.readValue(indexPath.toFile(), LibraryIndexDto.class);
  }

  private static PoseSpec toPose(PoseDto dto) {
    if (dto == null
        || dto.xMeters == null
        || dto.yMeters == null
        || !Double.isFinite(dto.xMeters)
        || !Double.isFinite(dto.yMeters)) {
      return null;
    }
    double headingDeg =
        dto.headingDeg != null && Double.isFinite(dto.headingDeg) ? dto.headingDeg : 0.0;
    return new PoseSpec(dto.xMeters, dto.yMeters, headingDeg);
  }

  private static List<ZoneSpec> toZones(List<ZoneDto> dtos) {
    ArrayList<ZoneSpec> zones = new ArrayList<>();
    if (dtos == null) {
      return List.of();
    }
    for (ZoneDto dto : dtos) {
      if (dto == null
          || dto.xMinMeters == null
          || dto.yMinMeters == null
          || dto.xMaxMeters == null
          || dto.yMaxMeters == null
          || !Double.isFinite(dto.xMinMeters)
          || !Double.isFinite(dto.yMinMeters)
          || !Double.isFinite(dto.xMaxMeters)
          || !Double.isFinite(dto.yMaxMeters)) {
        continue;
      }
      zones.add(
          new ZoneSpec(
              blankToNull(dto.id),
              blankToNull(dto.label),
              dto.xMinMeters,
              dto.yMinMeters,
              dto.xMaxMeters,
              dto.yMaxMeters,
              dto.locked));
    }
    return List.copyOf(zones);
  }

  private static List<StepSpec> toSteps(List<StepDto> dtos) {
    ArrayList<StepSpec> steps = new ArrayList<>();
    if (dtos == null) {
      return List.of();
    }
    for (StepDto dto : dtos) {
      if (dto == null) {
        continue;
      }
      String spotId = blankToNull(dto.spotId);
      Double xMeters = finite(dto.xMeters);
      Double yMeters = finite(dto.yMeters);
      if (spotId == null && (xMeters == null || yMeters == null)) {
        continue;
      }
      ArrayList<PoseSpec> routeWaypoints = new ArrayList<>();
      if (dto.routeWaypoints != null) {
        for (PoseDto waypoint : dto.routeWaypoints) {
          PoseSpec pose = toPose(waypoint);
          if (pose != null) {
            routeWaypoints.add(pose);
          }
        }
      }
      steps.add(
          new StepSpec(
              spotId,
              blankToNull(dto.requestedState),
              blankToNull(dto.label),
              blankToNull(dto.group),
              blankToNull(dto.alliance),
              xMeters,
              yMeters,
              finite(dto.headingDeg),
              finite(dto.constraintFactor),
              finite(dto.toleranceMeters),
              finite(dto.timeoutSeconds),
              finite(dto.endVelocityMps),
              dto.stopOnEnd,
              List.copyOf(routeWaypoints)));
    }
    return List.copyOf(steps);
  }

  private static Double finite(Double value) {
    return value != null && Double.isFinite(value) ? value : null;
  }

  private static String blankToNull(String value) {
    return value == null || value.isBlank() ? null : value.trim();
  }

  private static void reportError(String message, IOException ex) {
    DriverStation.reportError(message + ": " + ex.getMessage(), ex.getStackTrace());
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

  private static final class LibraryIndexDto {
    public String version;
    public String generator;
    public List<AutoManifestEntryDto> autos = List.of();
  }

  private static final class AutoManifestEntryDto {
    public String id;
    public String name;
    public String folder;
    public String relativePath;
    public Long updatedAt;
  }

  private static final class AutoFileDto {
    public String id;
    public String name;
    public String folder;
    public Long updatedAt;
    public PoseDto startPose;
    public List<ZoneDto> customZones = List.of();
    public List<StepDto> steps = List.of();
  }

  private static final class PoseDto {
    public Double xMeters;
    public Double yMeters;
    public Double headingDeg;
  }

  private static final class ZoneDto {
    public String id;
    public String label;
    public Double xMinMeters;
    public Double yMinMeters;
    public Double xMaxMeters;
    public Double yMaxMeters;
    public boolean locked;
  }

  private static final class StepDto {
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
}
