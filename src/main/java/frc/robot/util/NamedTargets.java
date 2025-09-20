package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Helpers for resolving and navigating to operator-defined targets created in PathPlanner/Choreo.
 *
 * <p>Expected naming (Blue-side definitions only; red will be auto-flipped by AutoBuilder): -
 * Source_Blue - Reef_1_L_Blue or Reef_1_R_Blue (faces 1..6; L = left/A, R = right/B)
 */
public class NamedTargets {
  private NamedTargets() {}

  /**
   * Follow a PathPlanner/Choreo target by name if present. If not present, fallback to our built-in
   * alignment commands using the naming convention.
   */
  public static Command goTo(SwerveSubsystem drive, String name) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      if (path != null) {
        return AutoBuilder.followPath(path)
            .beforeStarting(() -> DriveCommands.setAlignContext("path:" + name, name))
            .finallyDo(DriveCommands::clearAlignTelemetry);
      }
    } catch (Throwable ignored) {
      // Fall through to convention-based fallback
    }

    // Try Choreo .traj waypoint (single target pose)
    try {
      Pose2d choreoPose = loadChoreoWaypoint(name);
      if (choreoPose != null) {
        try {
          org.littletonrobotics.junction.Logger.recordOutput(
              "Autonomy/ResolvedTargetPose", choreoPose);
        } catch (Throwable ignored) {
        }
        // Pathfind to pose, then PID finish
        return DriveCommands.pathfindThenPIDCommand(drive, () -> choreoPose, "named:" + name)
            .beforeStarting(() -> DriveCommands.setAlignContext("named:" + name, name))
            .finallyDo(DriveCommands::clearAlignTelemetry);
      }
    } catch (Throwable ignored) {
    }

    // Fallbacks based on naming convention
    String n = name == null ? "" : name.trim();
    if (n.equalsIgnoreCase("Source_Blue")) {
      return DriveCommands.alignToNearestCoralStationCommandAuto(drive)
          .beforeStarting(() -> DriveCommands.setAlignContext("source", name))
          .finallyDo(DriveCommands::clearAlignTelemetry);
    }

    // Reef_{1..6}_{L|R}_Blue
    try {
      String[] parts = n.split("_");
      if (parts.length >= 4 && parts[0].equalsIgnoreCase("Reef")) {
        int face = Integer.parseInt(parts[1]);
        boolean left = parts[2].equalsIgnoreCase("L");
        int branchOffset = left ? -1 : 1;
        return DriveCommands.alignToReefBranchCommandAuto(drive, face, branchOffset)
            .beforeStarting(() -> DriveCommands.setAlignContext("reef", name))
            .finallyDo(DriveCommands::clearAlignTelemetry);
      }
    } catch (Throwable ignored) {
    }

    // Nothing matched; no-op with a warning
    return Commands.print("Named target not found: " + n);
  }

  private static Pose2d loadChoreoWaypoint(String baseName) throws IOException {
    if (baseName == null || baseName.isBlank()) return null;
    Path traj =
        Filesystem.getDeployDirectory().toPath().resolve("choreo").resolve(baseName + ".traj");
    if (!Files.exists(traj)) return null;
    String json = Files.readString(traj, StandardCharsets.UTF_8);

    // Extract params.waypoints[0].x.val, y.val, heading.val
    Double x = extractFirstVal(json, "\"x\"\\s*:\\s*\\{[^}]*\\\"val\\\"\\s*:\\s*([-0-9.Ee]+)");
    Double y = extractFirstVal(json, "\"y\"\\s*:\\s*\\{[^}]*\\\"val\\\"\\s*:\\s*([-0-9.Ee]+)");
    Double h =
        extractFirstVal(json, "\"heading\"\\s*:\\s*\\{[^}]*\\\"val\\\"\\s*:\\s*([-0-9.Ee]+)");
    if (x == null || y == null || h == null) return null;
    return new Pose2d(x, y, new Rotation2d(h));
  }

  private static Double extractFirstVal(String text, String regex) {
    Pattern p = Pattern.compile(regex);
    Matcher m = p.matcher(text);
    if (m.find()) {
      try {
        return Double.parseDouble(m.group(1));
      } catch (NumberFormatException ignored) {
      }
    }
    return null;
  }
}
