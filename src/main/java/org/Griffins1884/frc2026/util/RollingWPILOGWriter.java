package org.Griffins1884.frc2026.util;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class RollingWPILOGWriter implements LogDataReceiver {
  private final Object lock = new Object();
  private WPILOGWriter writer;
  private boolean started;

  @Override
  public void start() {
    synchronized (lock) {
      if (writer == null) {
        writer = new WPILOGWriter();
      }
      try {
        writer.start();
        started = true;
      } catch (RuntimeException ex) {
        started = false;
      }
    }
  }

  @Override
  public void end() {
    synchronized (lock) {
      if (writer != null && started) {
        try {
          writer.end();
        } catch (RuntimeException ex) {
          // Avoid crashing if WPILOGWriter was never started or failed to open a file.
        } finally {
          started = false;
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

  public void roll() {
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
      } catch (RuntimeException ex) {
        started = false;
      }
    }
  }
}
