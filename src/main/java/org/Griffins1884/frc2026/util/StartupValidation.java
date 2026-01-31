package org.Griffins1884.frc2026.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.subsystems.climber.ClimberConstants;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerConstants;
import org.Griffins1884.frc2026.subsystems.intake.IntakeConstants;
import org.Griffins1884.frc2026.subsystems.intake.IntakePivotConstants;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterConstants;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotConstants;
import org.Griffins1884.frc2026.subsystems.turret.TurretConstants;

public final class StartupValidation {
  private static boolean reported = false;

  private StartupValidation() {}

  public static void checkAndReport() {
    if (reported) {
      return;
    }
    reported = true;
    for (String warning : collectWarnings(RobotBase.isReal())) {
      DriverStation.reportWarning(warning, false);
      new Alert(warning, AlertType.kWarning).set(true);
    }
  }

  static List<String> collectWarnings(boolean isReal) {
    List<String> warnings = new ArrayList<>();
    if (!isReal) {
      return warnings;
    }

    if (GlobalConstants.MODE != GlobalConstants.RobotMode.REAL) {
      warnings.add(
          "GlobalConstants.MODE is "
              + GlobalConstants.MODE
              + " while running on real hardware.");
    }
    if (GlobalConstants.ROBOT == GlobalConstants.RobotType.SIMBOT) {
      warnings.add(
          "GlobalConstants.ROBOT is SIMBOT while running on real hardware.");
    }

    List<CanIdCheck> canIds = List.of(
        new CanIdCheck("Turret", TurretConstants.TURRET_ID, false),
        new CanIdCheck("Intake", IntakeConstants.INTAKE_ID, false),
        new CanIdCheck("Shooter", ShooterConstants.SHOOTER_ID, false),
        new CanIdCheck("Indexer Leader", IndexerConstants.LEADER_ID, false),
        new CanIdCheck("Indexer Follower", IndexerConstants.FOLLOWER_ID, true),
        new CanIdCheck("Shooter Pivot", ShooterPivotConstants.MOTOR_ID, false),
        new CanIdCheck("Intake Pivot", IntakePivotConstants.PIVOT_ID, false),
        new CanIdCheck("Climber Left", ClimberConstants.LEFT_CLIMBER, false),
        new CanIdCheck("Climber Right", ClimberConstants.RIGHT_CLIMBER, false));

    Map<Integer, List<String>> usedIds = new HashMap<>();
    for (CanIdCheck check : canIds) {
      if (check.id <= 0) {
        warnings.add(
            (check.optional
                ? "Optional CAN ID for "
                    + check.name
                    + " is "
                    + check.id
                    + " (verify if unused)."
                : "CAN ID for " + check.name + " is " + check.id + " (default/invalid)."));
        continue;
      }
      usedIds.computeIfAbsent(check.id, id -> new ArrayList<>()).add(check.name);
    }

    for (Map.Entry<Integer, List<String>> entry : usedIds.entrySet()) {
      if (entry.getValue().size() > 1) {
        warnings.add(
            "Duplicate CAN ID "
                + entry.getKey()
                + " detected for: "
                + String.join(", ", entry.getValue())
                + ".");
      }
    }

    return warnings;
  }

  private static final class CanIdCheck {
    private final String name;
    private final int id;
    private final boolean optional;

    private CanIdCheck(String name, int id, boolean optional) {
      this.name = name;
      this.id = id;
      this.optional = optional;
    }
  }
}
