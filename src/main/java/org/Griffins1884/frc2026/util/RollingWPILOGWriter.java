package org.Griffins1884.frc2026.util;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class RollingWPILOGWriter implements LogDataReceiver {
  private final Object lock = new Object();
  private WPILOGWriter writer;
  private boolean started;
  private String lastStatus = "INIT";
  private String lastError = "";
  private int startFailureCount = 0;
  private int rollFailureCount = 0;

  @Override
  public void start() {
    synchronized (lock) {
      if (writer == null) {
        writer = new WPILOGWriter();
      }
      try {
        writer.start();
        started = true;
        lastStatus = "STARTED";
        lastError = "";
      } catch (RuntimeException ex) {
        started = false;
        lastStatus = "START_FAILED";
        lastError = ex.getClass().getSimpleName() + ": " + String.valueOf(ex.getMessage());
        startFailureCount++;
      }
      publishStatus();
    }
  }

  @Override
  public void end() {
    synchronized (lock) {
      if (writer != null && started) {
        try {
          writer.end();
          lastStatus = "ENDED";
        } catch (RuntimeException ex) {
          // Avoid crashing if WPILOGWriter was never started or failed to open a file.
          lastStatus = "END_FAILED";
          lastError = ex.getClass().getSimpleName() + ": " + String.valueOf(ex.getMessage());
        } finally {
          started = false;
          publishStatus();
        }
      }
    }
  }

  @Override
  public void putTable(LogTable table) throws InterruptedException {
    synchronized (lock) {
      if (writer != null && started) {
        writer.putTable(table);
      }
    }
  }

  private void publishStatus() {
    Logger.recordOutput("Log/WPILOGWriter/Started", started);
    Logger.recordOutput("Log/WPILOGWriter/Status", lastStatus);
    Logger.recordOutput("Log/WPILOGWriter/LastError", lastError);
    Logger.recordOutput("Log/WPILOGWriter/StartFailureCount", startFailureCount);
    Logger.recordOutput("Log/WPILOGWriter/RollFailureCount", rollFailureCount);
  }

  public boolean roll() {
    synchronized (lock) {
      if (writer != null && started) {
        try {
          writer.end();
        } catch (RuntimeException ex) {
          // Ignore failures during rollover to keep logging alive.
        } finally {
          started = false;
        }
      }
      writer = new WPILOGWriter();
      try {
        writer.start();
        started = true;
        lastStatus = "ROLLED";
        lastError = "";
        publishStatus();
        return true;
      } catch (RuntimeException ex) {
        started = false;
        lastStatus = "ROLL_FAILED";
        lastError = ex.getClass().getSimpleName() + ": " + String.valueOf(ex.getMessage());
        rollFailureCount++;
        publishStatus();
        return false;
      }
    }
  }
}
