package org.Griffins1884.frc2026.util;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public final class LogRollover {
  private static final String STATUS_READY = "READY";
  private static final String STATUS_ROLLED = "ROLLED";
  private static final String STATUS_FAILED = "FAILED";
  private static final String STATUS_UNAVAILABLE = "UNAVAILABLE";
  private static final String CLEAN_STATUS_READY = "READY";
  private static final String CLEAN_STATUS_CLEANED = "CLEANED";
  private static final String CLEAN_STATUS_FAILED = "FAILED";
  private static final String CLEAN_STATUS_UNAVAILABLE = "UNAVAILABLE";
  private static final String CLEAN_STATUS_CLEANING = "CLEANING";

  private static RollingWPILOGWriter rollingWriter;
  private static String status = STATUS_UNAVAILABLE;
  private static double lastRollTimestampSec = Double.NaN;
  private static int rollCount = 0;
  private static String cleanStatus = CLEAN_STATUS_UNAVAILABLE;
  private static double lastCleanTimestampSec = Double.NaN;
  private static int cleanCount = 0;
  private static int lastCleanDeletedEntries = 0;

  private LogRollover() {}

  public static synchronized void init(RollingWPILOGWriter writer) {
    rollingWriter = writer;
    status = writer == null ? STATUS_UNAVAILABLE : STATUS_READY;
    cleanStatus = writer == null ? CLEAN_STATUS_UNAVAILABLE : CLEAN_STATUS_READY;
    publishStatus();
  }

  public static synchronized boolean roll() {
    if (rollingWriter == null) {
      status = STATUS_UNAVAILABLE;
      publishStatus();
      return false;
    }
    try {
      boolean rolled = rollingWriter.roll();
      if (rolled) {
        status = STATUS_ROLLED;
        rollCount++;
        lastRollTimestampSec = Timer.getFPGATimestamp();
      } else {
        status = STATUS_FAILED;
      }
      publishStatus();
      return rolled;
    } catch (RuntimeException ex) {
      status = STATUS_FAILED;
      publishStatus();
      return false;
    }
  }

  public static synchronized String getStatus() {
    return status;
  }

  public static synchronized double getLastRollTimestampSec() {
    return lastRollTimestampSec;
  }

  public static synchronized int getRollCount() {
    return rollCount;
  }

  public static synchronized boolean cleanLogsFolder() {
    if (rollingWriter == null) {
      cleanStatus = CLEAN_STATUS_UNAVAILABLE;
      publishStatus();
      return false;
    }
    cleanStatus = CLEAN_STATUS_CLEANING;
    publishStatus();
    try {
      // Roll first so we keep the newest active file and only clear older artifacts.
      roll();
      int deleted = 0;
      deleted += cleanDirectory(resolveOperatingLogsDir());
      deleted += cleanDirectory(Path.of("/U/logs"));

      cleanStatus = CLEAN_STATUS_CLEANED;
      lastCleanTimestampSec = Timer.getFPGATimestamp();
      lastCleanDeletedEntries = deleted;
      cleanCount++;
      publishStatus();
      return true;
    } catch (Exception ex) {
      cleanStatus = CLEAN_STATUS_FAILED;
      publishStatus();
      return false;
    }
  }

  public static synchronized String getCleanStatus() {
    return cleanStatus;
  }

  public static synchronized double getLastCleanTimestampSec() {
    return lastCleanTimestampSec;
  }

  public static synchronized int getCleanCount() {
    return cleanCount;
  }

  public static synchronized int getLastCleanDeletedEntries() {
    return lastCleanDeletedEntries;
  }

  private static Path resolveOperatingLogsDir() {
    return Filesystem.getOperatingDirectory().toPath().resolve("logs");
  }

  private static int cleanDirectory(Path logsDir) throws IOException {
    if (logsDir == null || !Files.exists(logsDir) || !Files.isDirectory(logsDir)) {
      return 0;
    }

    List<Path> entries;
    try (Stream<Path> stream = Files.list(logsDir)) {
      entries = stream.toList();
    }
    if (entries.isEmpty()) {
      return 0;
    }

    Path keepWpilibLog = newestPath(entries, path -> isWpilibLog(path));
    Path keepNtSession = newestPath(entries, path -> isNetworkTablesSessionDir(path));

    int deletedEntries = 0;
    for (Path entry : entries) {
      if (entry.equals(keepWpilibLog) || entry.equals(keepNtSession)) {
        continue;
      }
      if (!looksLikeLogArtifact(entry)) {
        continue;
      }
      deleteRecursively(entry);
      deletedEntries++;
    }

    // Touch logs directory so we can easily see "cleanup happened" in file metadata.
    Files.writeString(
        logsDir.resolve(".cleanup-marker"),
        "cleaned@" + Timer.getFPGATimestamp(),
        StandardOpenOption.CREATE,
        StandardOpenOption.TRUNCATE_EXISTING);
    return deletedEntries;
  }

  private static Path newestPath(List<Path> entries, java.util.function.Predicate<Path> predicate)
      throws IOException {
    List<Path> filtered = new ArrayList<>();
    for (Path entry : entries) {
      if (predicate.test(entry)) {
        filtered.add(entry);
      }
    }
    if (filtered.isEmpty()) {
      return null;
    }
    filtered.sort(
        Comparator.comparingLong(
                (Path path) -> {
                  try {
                    return Files.getLastModifiedTime(path).toMillis();
                  } catch (IOException ex) {
                    return Long.MIN_VALUE;
                  }
                })
            .reversed());
    return filtered.get(0);
  }

  private static boolean isWpilibLog(Path path) {
    if (path == null || Files.isDirectory(path)) {
      return false;
    }
    String name = path.getFileName().toString().toLowerCase(Locale.ROOT);
    return name.endsWith(".wpilog");
  }

  private static boolean isNetworkTablesSessionDir(Path path) {
    if (path == null || !Files.isDirectory(path)) {
      return false;
    }
    String name = path.getFileName().toString().toLowerCase(Locale.ROOT);
    return name.startsWith("nt_");
  }

  private static boolean looksLikeLogArtifact(Path path) {
    if (path == null) {
      return false;
    }
    String name = path.getFileName().toString().toLowerCase(Locale.ROOT);
    if (Files.isDirectory(path)) {
      return name.startsWith("nt_") || name.startsWith("session") || name.equals("networktables");
    }
    return name.endsWith(".wpilog")
        || name.endsWith(".wpilog.tmp")
        || name.endsWith(".log")
        || name.endsWith(".dslog")
        || name.endsWith(".dsevents");
  }

  private static void deleteRecursively(Path path) throws IOException {
    if (path == null || !Files.exists(path)) {
      return;
    }
    try (Stream<Path> walk = Files.walk(path)) {
      walk.sorted(Comparator.reverseOrder())
          .forEach(
              p -> {
                try {
                  Files.deleteIfExists(p);
                } catch (IOException ex) {
                  throw new RuntimeException(ex);
                }
              });
    } catch (RuntimeException runtimeEx) {
      if (runtimeEx.getCause() instanceof IOException ioEx) {
        throw ioEx;
      }
      throw runtimeEx;
    }
  }

  private static void publishStatus() {
    Logger.recordOutput("Log/RollStatus", status);
    Logger.recordOutput("Log/RollTimestamp", lastRollTimestampSec);
    Logger.recordOutput("Log/RollTimestampSec", lastRollTimestampSec);
    Logger.recordOutput("Log/RollCount", rollCount);
    Logger.recordOutput("Log/CleanStatus", cleanStatus);
    Logger.recordOutput("Log/CleanTimestampSec", lastCleanTimestampSec);
    Logger.recordOutput("Log/CleanCount", cleanCount);
    Logger.recordOutput("Log/CleanDeletedEntries", lastCleanDeletedEntries);
  }
}
