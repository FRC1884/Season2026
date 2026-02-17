package org.Griffins1884.frc2026.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

public class AutoStartPoseProvider {
  private final Path autoDir;
  private final Path pathDir;
  private final Path choreoDir;
  private final JSONParser parser = new JSONParser();
  private String lastAutoName = null;
  private Optional<Pose2d> lastPose = Optional.empty();

  public AutoStartPoseProvider() {
    Path deployDir = Filesystem.getDeployDirectory().toPath();
    autoDir = deployDir.resolve("pathplanner").resolve("autos");
    pathDir = deployDir.resolve("pathplanner").resolve("paths");
    choreoDir = deployDir.resolve("Choreo");
  }

  public Optional<Pose2d> getStartPoseForAuto(String autoName) {
    if (autoName == null || autoName.isBlank()) {
      lastAutoName = null;
      lastPose = Optional.empty();
      Logger.recordOutput("AutoStartPose/AutoName", "");
      Logger.recordOutput("AutoStartPose/ResultValid", false);
      return lastPose;
    }
    String normalized =
        autoName.endsWith(".auto") ? autoName.substring(0, autoName.length() - 5) : autoName;
    if (normalized.equals(lastAutoName)) {
      return lastPose;
    }
    lastAutoName = normalized;
    Logger.recordOutput("AutoStartPose/AutoName", normalized);
    lastPose = loadStartPose(normalized);
    Logger.recordOutput("AutoStartPose/ResultValid", lastPose.isPresent());
    return lastPose;
  }

  private Optional<Pose2d> loadStartPose(String autoName) {
    Path autoPath = autoDir.resolve(autoName + ".auto");
    if (!Files.exists(autoPath)) {
      Logger.recordOutput("AutoStartPose/AutoFileExists", false);
      Logger.recordOutput("AutoStartPose/Reason", "MissingAutoFile");
      return Optional.empty();
    }
    Logger.recordOutput("AutoStartPose/AutoFileExists", true);
    JSONObject root = readJson(autoPath);
    if (root == null) {
      Logger.recordOutput("AutoStartPose/Reason", "AutoParseFailed");
      return Optional.empty();
    }
    boolean choreoAuto = Boolean.TRUE.equals(root.get("choreoAuto"));
    Logger.recordOutput("AutoStartPose/IsChoreoAuto", choreoAuto);
    JSONObject command = asObject(root.get("command"));
    String pathName = findFirstPathName(command);
    if (pathName == null || pathName.isBlank()) {
      Logger.recordOutput("AutoStartPose/Reason", "NoPathInAuto");
      return Optional.empty();
    }
    Logger.recordOutput("AutoStartPose/PathName", pathName);
    return choreoAuto ? loadChoreoStartPose(pathName) : loadPathPlannerStartPose(pathName);
  }

  private Optional<Pose2d> loadChoreoStartPose(String pathName) {
    Path trajPath = choreoDir.resolve(pathName + ".traj");
    if (!Files.exists(trajPath)) {
      Logger.recordOutput("AutoStartPose/TrajFileExists", false);
      Logger.recordOutput("AutoStartPose/Reason", "MissingTrajFile");
      return Optional.empty();
    }
    Logger.recordOutput("AutoStartPose/TrajFileExists", true);
    JSONObject root = readJson(trajPath);
    if (root == null) {
      Logger.recordOutput("AutoStartPose/Reason", "TrajParseFailed");
      return Optional.empty();
    }
    JSONObject snapshot = asObject(root.get("snapshot"));
    Pose2d pose = readChoreoWaypointPose(snapshot);
    if (pose != null) {
      Logger.recordOutput("AutoStartPose/Source", "TRAJ");
      return Optional.of(pose);
    }
    JSONObject params = asObject(root.get("params"));
    pose = readChoreoWaypointPose(params);
    Logger.recordOutput("AutoStartPose/Source", pose != null ? "TRAJ" : "TRAJ_MISSING_WAYPOINT");
    return Optional.ofNullable(pose);
  }

  private Pose2d readChoreoWaypointPose(JSONObject container) {
    if (container == null) {
      return null;
    }
    JSONArray waypoints = asArray(container.get("waypoints"));
    if (waypoints == null || waypoints.isEmpty()) {
      return null;
    }
    JSONObject first = asObject(waypoints.get(0));
    if (first == null) {
      return null;
    }
    double x = readDouble(first.get("x"), Double.NaN);
    double y = readDouble(first.get("y"), Double.NaN);
    double heading = readDouble(first.get("heading"), 0.0);
    if (!Double.isFinite(x) || !Double.isFinite(y)) {
      return null;
    }
    return new Pose2d(x, y, new Rotation2d(heading));
  }

  private Optional<Pose2d> loadPathPlannerStartPose(String pathName) {
    Path pathPath = pathDir.resolve(pathName + ".path");
    if (!Files.exists(pathPath)) {
      Logger.recordOutput("AutoStartPose/PathFileExists", false);
      Logger.recordOutput("AutoStartPose/Reason", "MissingPathFile");
      return Optional.empty();
    }
    Logger.recordOutput("AutoStartPose/PathFileExists", true);
    JSONObject root = readJson(pathPath);
    if (root == null) {
      Logger.recordOutput("AutoStartPose/Reason", "PathParseFailed");
      return Optional.empty();
    }
    JSONArray waypoints = asArray(root.get("waypoints"));
    if (waypoints == null || waypoints.isEmpty()) {
      Logger.recordOutput("AutoStartPose/Reason", "NoWaypointsInPath");
      return Optional.empty();
    }
    JSONObject first = asObject(waypoints.get(0));
    JSONObject anchor = first != null ? asObject(first.get("anchor")) : null;
    double x = anchor != null ? readDouble(anchor.get("x"), Double.NaN) : Double.NaN;
    double y = anchor != null ? readDouble(anchor.get("y"), Double.NaN) : Double.NaN;
    if (!Double.isFinite(x) || !Double.isFinite(y)) {
      Logger.recordOutput("AutoStartPose/Reason", "InvalidAnchor");
      return Optional.empty();
    }
    JSONObject idealStarting = asObject(root.get("idealStartingState"));
    double rotationDeg =
        idealStarting != null ? readDouble(idealStarting.get("rotation"), 0.0) : 0.0;
    Logger.recordOutput("AutoStartPose/Source", "PATH");
    return Optional.of(new Pose2d(x, y, Rotation2d.fromDegrees(rotationDeg)));
  }

  private String findFirstPathName(JSONObject command) {
    if (command == null) {
      return null;
    }
    String type = asString(command.get("type"));
    JSONObject data = asObject(command.get("data"));
    if ("path".equals(type) && data != null) {
      return asString(data.get("pathName"));
    }
    JSONArray commands = data != null ? asArray(data.get("commands")) : null;
    if (commands != null) {
      for (Object child : commands) {
        String pathName = findFirstPathName(asObject(child));
        if (pathName != null && !pathName.isBlank()) {
          return pathName;
        }
      }
    }
    return null;
  }

  private JSONObject readJson(Path path) {
    try {
      return (JSONObject) parser.parse(Files.readString(path));
    } catch (IOException | ParseException ex) {
      return null;
    }
  }

  private static JSONObject asObject(Object value) {
    return value instanceof JSONObject ? (JSONObject) value : null;
  }

  private static JSONArray asArray(Object value) {
    return value instanceof JSONArray ? (JSONArray) value : null;
  }

  private static String asString(Object value) {
    return value != null ? value.toString() : null;
  }

  private static double readDouble(Object value, double fallback) {
    if (value instanceof Number) {
      return ((Number) value).doubleValue();
    }
    if (value instanceof JSONObject) {
      JSONObject obj = (JSONObject) value;
      if (obj.containsKey("val")) {
        return readDouble(obj.get("val"), fallback);
      }
    }
    if (value instanceof String) {
      try {
        return Double.parseDouble((String) value);
      } catch (NumberFormatException ex) {
        return fallback;
      }
    }
    return fallback;
  }
}
