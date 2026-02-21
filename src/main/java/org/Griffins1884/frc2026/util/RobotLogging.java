package org.Griffins1884.frc2026.util;

import edu.wpi.first.wpilibj.DriverStation;
import org.Griffins1884.frc2026.GlobalConstants;

/** Shared runtime logging helpers for debug/competition behavior. */
public final class RobotLogging {
  private RobotLogging() {}

  public static boolean isDebugMode() {
    return GlobalConstants.isDebugMode();
  }

  public static boolean isCompMode() {
    return GlobalConstants.isCompMode();
  }

  public static void debug(String message) {
    if (isDebugMode()) {
      DriverStation.reportWarning("[DEBUG] " + message, false);
    }
  }

  public static void info(String message) {
    DriverStation.reportWarning(message, false);
  }

  public static void warn(String message) {
    DriverStation.reportWarning("[WARN] " + message, false);
  }

  public static void error(String message) {
    DriverStation.reportError(message, false);
  }

  public static void error(String message, Throwable throwable) {
    if (throwable == null) {
      error(message);
      return;
    }
    DriverStation.reportError(message + ": " + throwable.getMessage(), throwable.getStackTrace());
  }
}
