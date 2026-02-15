package org.Griffins1884.frc2026.util;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class RollingWPILOGWriter implements LogDataReceiver {
  private final Object lock = new Object();
  private WPILOGWriter writer;

  @Override
  public void start() {
    synchronized (lock) {
      if (writer == null) {
        writer = new WPILOGWriter();
      }
      writer.start();
    }
  }

  @Override
  public void end() {
    synchronized (lock) {
      if (writer != null) {
        writer.end();
      }
    }
  }

  @Override
  public void putTable(LogTable table) throws InterruptedException {
    synchronized (lock) {
      if (writer != null) {
        writer.putTable(table);
      }
    }
  }

  public void roll() {
    synchronized (lock) {
      if (writer != null) {
        writer.end();
      }
      writer = new WPILOGWriter();
      writer.start();
    }
  }
}
