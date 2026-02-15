package org.Griffins1884.frc2026.util;

import org.littletonrobotics.junction.Logger;

public final class LogRollover {
  private static RollingWPILOGWriter rollingWriter;

  private LogRollover() {}

  public static void init(RollingWPILOGWriter writer) {
    rollingWriter = writer;
  }

  public static boolean roll() {
    if (rollingWriter == null) {
      Logger.recordOutput("Log/RollStatus", "UNAVAILABLE");
      return false;
    }
    try {
      rollingWriter.roll();
      Logger.recordOutput("Log/RollStatus", "ROLLED");
      Logger.recordOutput("Log/RollTimestamp", Logger.getTimestamp());
      return true;
    } catch (RuntimeException ex) {
      Logger.recordOutput("Log/RollStatus", "FAILED");
      return false;
    }
  }
}
