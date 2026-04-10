package org.Griffins1884.frc2026.subsystems.objectivetracker;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Set;
import java.util.UUID;
import java.util.stream.Collectors;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import sun.misc.Unsafe;

final class DeployAutoTestSupport {
  private static final Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();

  private DeployAutoTestSupport() {}

  static String uniqueName(String prefix) {
    return prefix + "-" + UUID.randomUUID().toString().replace("-", "");
  }

  static Path writeAuto(String autoName, String body) throws IOException {
    Path file = DEPLOY_DIRECTORY.resolve("pathplanner/autos/" + autoName + ".auto");
    Files.createDirectories(file.getParent());
    Files.writeString(file, body);
    return file;
  }

  static Path writePath(String pathName, double endVelocityMps, String markerCommandName)
      throws IOException {
    Path file = DEPLOY_DIRECTORY.resolve("pathplanner/paths/" + pathName + ".path");
    Files.createDirectories(file.getParent());
    String eventMarkers =
        markerCommandName == null
            ? "[]"
            : """
              [
                {
                  "name": "Marker",
                  "waypointRelativePos": 0.5,
                  "command": {
                    "type": "named",
                    "data": {
                      "name": "%s"
                    }
                  }
                }
              ]
              """
                .formatted(markerCommandName);
    String json =
        """
        {
          "version": "2025.0",
          "waypoints": [
            {
              "anchor": {"x": 1.0, "y": 1.0},
              "prevControl": null,
              "nextControl": {"x": 2.0, "y": 1.0},
              "isLocked": false,
              "linkedName": null
            },
            {
              "anchor": {"x": 4.0, "y": 1.0},
              "prevControl": {"x": 3.0, "y": 1.0},
              "nextControl": null,
              "isLocked": false,
              "linkedName": null
            }
          ],
          "rotationTargets": [],
          "constraintZones": [],
          "pointTowardsZones": [],
          "eventMarkers": %s,
          "globalConstraints": {
            "maxVelocity": 3.0,
            "maxAcceleration": 1.5,
            "maxAngularVelocity": 540.0,
            "maxAngularAcceleration": 720.0,
            "nominalVoltage": 12.0,
            "unlimited": false
          },
          "goalEndState": {
            "velocity": %.3f,
            "rotation": 0.0
          },
          "reversed": false,
          "folder": null,
          "idealStartingState": {
            "velocity": 0.0,
            "rotation": 0.0
          },
          "useDefaultConstraints": true
        }
        """
            .formatted(eventMarkers, endVelocityMps);
    Files.writeString(file, json);
    return file;
  }

  static String sequentialAutoJson(boolean choreoAuto, String... commandsJson) {
    return """
        {
          "version": "2025.0",
          "command": {
            "type": "sequential",
            "data": {
              "commands": [
                %s
              ]
            }
          },
          "resetOdom": true,
          "folder": "Test",
          "choreoAuto": %s
        }
        """
        .formatted(String.join(",\n", commandsJson), Boolean.toString(choreoAuto));
  }

  static String pathCommandJson(String pathName) {
    return """
        {
          "type": "path",
          "data": {
            "pathName": "%s"
          }
        }
        """
        .formatted(pathName);
  }

  static String namedCommandJson(String commandName) {
    return """
        {
          "type": "named",
          "data": {
            "name": "%s"
          }
        }
        """
        .formatted(commandName);
  }

  static String waitCommandJson(double waitSeconds) {
    return """
        {
          "type": "wait",
          "data": {
            "waitTime": %.3f
          }
        }
        """
        .formatted(waitSeconds);
  }

  static DeployAutoLibrary newLibrary() {
    return new DeployAutoLibrary(DEPLOY_DIRECTORY.resolve("pathplanner/autos"));
  }

  static RebuiltAutoQueue newQueue(SwerveSubsystem drive) {
    return new RebuiltAutoQueue(new RebuiltSpotLibrary(), null, drive, newLibrary());
  }

  static SwerveSubsystem fakeDrive() {
    try {
      Field unsafeField = Unsafe.class.getDeclaredField("theUnsafe");
      unsafeField.setAccessible(true);
      Unsafe unsafe = (Unsafe) unsafeField.get(null);
      return (SwerveSubsystem) unsafe.allocateInstance(SwerveSubsystem.class);
    } catch (ReflectiveOperationException ex) {
      throw new RuntimeException("Failed to allocate fake SwerveSubsystem", ex);
    }
  }

  static List<Command> collectCommands(Command root) {
    if (root == null) {
      return List.of();
    }
    ArrayList<Command> commands = new ArrayList<>();
    Set<Object> seen = Collections.newSetFromMap(new IdentityHashMap<>());
    walkObject(root, commands, seen);
    return commands;
  }

  static List<String> stepLabels(DeployAutoLibrary.LoadedAuto loadedAuto) {
    return loadedAuto.steps().stream()
        .map(DeployAutoLibrary.StepSpec::label)
        .collect(Collectors.toList());
  }

  static void deleteIfExists(Path file) throws IOException {
    Files.deleteIfExists(file);
  }

  private static void walkObject(Object value, List<Command> commands, Set<Object> seen) {
    if (value == null || seen.contains(value)) {
      return;
    }
    seen.add(value);
    Class<?> type = value.getClass();
    if (type.isArray()) {
      int length = Array.getLength(value);
      for (int i = 0; i < length; i++) {
        Object element = Array.get(value, i);
        if (element instanceof Command
            || element instanceof Iterable<?>
            || (element != null && element.getClass().isArray())) {
          walkObject(element, commands, seen);
        }
      }
      return;
    }
    if (value instanceof Iterable<?> iterable) {
      for (Object child : iterable) {
        if (child instanceof Command
            || child instanceof Iterable<?>
            || (child != null && child.getClass().isArray())) {
          walkObject(child, commands, seen);
        }
      }
      return;
    }
    if (!(value instanceof Command command)) {
      return;
    }
    commands.add(command);
    for (Class<?> current = type;
        current != null && current != Object.class;
        current = current.getSuperclass()) {
      for (Field field : current.getDeclaredFields()) {
        try {
          field.setAccessible(true);
          Object child = field.get(value);
          if (child instanceof Command
              || child instanceof Iterable<?>
              || (child != null && child.getClass().isArray())) {
            walkObject(child, commands, seen);
          }
        } catch (Exception ignored) {
          // Best-effort command graph traversal for tests only.
        }
      }
    }
  }
}
