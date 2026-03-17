package org.Griffins1884.frc2026.subsystems.objectivetracker;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

final class RebuiltSpotLibrary {
  private static final ObjectMapper JSON =
      new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
  private static final Path SPOTS_PATH =
      Filesystem.getDeployDirectory()
          .toPath()
          .resolve("operatorboard")
          .resolve("rebuilt-spots.json");

  private final Map<String, RebuiltSpot> spotsById;
  private final FieldSpec field;

  RebuiltSpotLibrary() {
    try {
      LayoutSpec spec = JSON.readValue(SPOTS_PATH.toFile(), LayoutSpec.class);
      field = spec.field != null ? spec.field : new FieldSpec();
      spotsById = new LinkedHashMap<>();
      if (spec.spots != null) {
        for (RebuiltSpot spot : spec.spots) {
          if (spot != null && spot.id != null && !spot.id.isBlank()) {
            spotsById.put(spot.id, spot);
          }
        }
      }
    } catch (IOException ex) {
      throw new RuntimeException("Failed to load REBUILT spots from " + SPOTS_PATH, ex);
    }
  }

  Optional<RebuiltSpot> getSpot(String id) {
    if (id == null || id.isBlank()) {
      return Optional.empty();
    }
    return Optional.ofNullable(spotsById.get(id));
  }

  double getFieldLengthMeters() {
    return field.lengthMeters;
  }

  double getFieldWidthMeters() {
    return field.widthMeters;
  }

  double getSafeAutoMaxXMeters() {
    return field.safeAutoMaxXMeters;
  }

  static final class RebuiltSpot {
    public String id;
    public String label;
    public String group;
    public String alliance;
    public double x;
    public double y;
    public double headingDeg;

    public Pose2d toPose() {
      return new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg));
    }

    public Optional<Alliance> getAlliance() {
      if (alliance == null || alliance.isBlank()) {
        return Optional.empty();
      }
      try {
        return Optional.of(Alliance.valueOf(alliance));
      } catch (IllegalArgumentException ex) {
        return Optional.empty();
      }
    }

    public String displayLabel() {
      return label == null || label.isBlank() ? id : label;
    }
  }

  private static final class LayoutSpec {
    public FieldSpec field = new FieldSpec();
    public List<RebuiltSpot> spots = List.of();
  }

  private static final class FieldSpec {
    public double lengthMeters = 16.54048;
    public double widthMeters = 8.06958;
    public double safeAutoMaxXMeters = 8.15;
  }
}
