package org.Griffins1884.frc2026.subsystems.objectivetracker;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

public final class OperatorBoardDiagnosticBundleWriter {
  private static final DateTimeFormatter BUNDLE_TIME =
      DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss").withZone(ZoneOffset.UTC);

  private final OperatorBoardPersistence persistence;
  private String lastFingerprint = "";

  public OperatorBoardDiagnosticBundleWriter(OperatorBoardPersistence persistence) {
    this.persistence = persistence;
  }

  public synchronized OperatorBoardDataModels.DiagnosticBundleManifest maybeWrite(
      OperatorBoardDiagnosticSnapshot snapshot) {
    String fingerprint = snapshot.fingerprint();
    if (fingerprint.equals(lastFingerprint)) {
      return persistence.readLatestDiagnosticManifest().orElseGet(() -> write(snapshot));
    }
    lastFingerprint = fingerprint;
    return write(snapshot);
  }

  public synchronized OperatorBoardDataModels.DiagnosticBundleManifest forceWrite(
      OperatorBoardDiagnosticSnapshot snapshot) {
    lastFingerprint = snapshot.fingerprint();
    return write(snapshot);
  }

  private OperatorBoardDataModels.DiagnosticBundleManifest write(
      OperatorBoardDiagnosticSnapshot snapshot) {
    persistence.initialize();
    String bundleId = BUNDLE_TIME.format(Instant.now()) + "-" + snapshot.overallStatus();
    Path bundleDir = persistence.diagnosticBundlesRoot().resolve(bundleId);
    try {
      Files.createDirectories(bundleDir);
      writeJson(
          bundleDir.resolve("summary.json"),
          new OperatorBoardDataModels.DiagnosticBundleSummary(
              OperatorBoardDataModels.SCHEMA_VERSION,
              OperatorBoardDataModels.metadata("diagnosticSummary", bundleId, "Diagnostic Summary"),
              snapshot.overallStatus(),
              snapshot.systemSummary(),
              snapshot.mode(),
              snapshot.actionStatus(),
              snapshot.autoSummary(),
              snapshot.mechanismSummary(),
              snapshot.runtimeProfileStatus()));
      writeJson(
          bundleDir.resolve("observed-data.json"),
          new OperatorBoardDataModels.DiagnosticObservedData(
              snapshot.systemCheck(),
              snapshot.autoCheck(),
              snapshot.ntDiagnostics(),
              snapshot.mechanismStatus(),
              snapshot.actionTrace(),
              snapshot.runtimeProfile(),
              snapshot.selectedAuto(),
              snapshot.autoQueue(),
              snapshot.subsystemDescriptions()));
      writeRawJson(bundleDir.resolve("networktables_snapshot.json"), snapshot.ntDiagnostics());
      writeRawJson(bundleDir.resolve("subsystems.json"), snapshot.mechanismStatus());
      writeRawJson(bundleDir.resolve("alerts.json"), snapshot.systemCheck());
      writeRawJson(bundleDir.resolve("auto_context.json"), snapshot.autoCheck());
      writeRawJson(
          bundleDir.resolve("subsystem_descriptions.json"), snapshot.subsystemDescriptions());
      Files.writeString(
          bundleDir.resolve("report.md"), buildMarkdownReport(snapshot), StandardCharsets.UTF_8);

      List<String> files =
          List.of(
              "summary.json",
              "observed-data.json",
              "networktables_snapshot.json",
              "subsystems.json",
              "alerts.json",
              "auto_context.json",
              "subsystem_descriptions.json",
              "report.md");
      OperatorBoardDataModels.DiagnosticBundleManifest manifest =
          new OperatorBoardDataModels.DiagnosticBundleManifest(
              OperatorBoardDataModels.SCHEMA_VERSION,
              OperatorBoardDataModels.metadata("diagnosticBundle", bundleId, "Diagnostic Bundle"),
              bundleId,
              bundleDir.toString(),
              snapshot.overallStatus(),
              files);
      writeJson(bundleDir.resolve("manifest.json"), manifest);
      persistence.writeLatestDiagnosticManifest(manifest);
      return manifest;
    } catch (IOException ex) {
      DriverStation.reportError(
          "Failed to write diagnostic bundle: " + ex.getMessage(), ex.getStackTrace());
      throw new UncheckedIOException(ex);
    }
  }

  private void writeJson(Path target, Object value) throws IOException {
    OperatorBoardPersistence.JSON.writeValue(target.toFile(), value);
  }

  private void writeRawJson(Path target, String value) throws IOException {
    String normalized = value == null || value.isBlank() ? "{}" : value;
    Files.writeString(target, normalized, StandardCharsets.UTF_8);
  }

  private String buildMarkdownReport(OperatorBoardDiagnosticSnapshot snapshot) {
    List<String> lines = new ArrayList<>();
    lines.add("# Operator Board Diagnostic Bundle");
    lines.add("");
    lines.add("- Generated: " + Instant.now());
    lines.add("- Overall status: " + snapshot.overallStatus());
    lines.add("- System summary: " + snapshot.systemSummary());
    lines.add("- Auto summary: " + snapshot.autoSummary());
    lines.add("- Mechanism summary: " + snapshot.mechanismSummary());
    lines.add("- Runtime profile status: " + snapshot.runtimeProfileStatus());
    lines.add("- Mode: " + snapshot.mode());
    lines.add("");
    lines.add("## Action Trace");
    lines.add("");
    lines.add("```json");
    lines.add(
        snapshot.actionTrace() == null || snapshot.actionTrace().isBlank()
            ? "{}"
            : snapshot.actionTrace());
    lines.add("```");
    lines.add("");
    lines.add("## Recommended Next Checks");
    lines.add("");
    lines.add("- Review `summary.json` for the machine-readable verdict.");
    lines.add(
        "- Compare `networktables_snapshot.json` against expected mode, queue, and health state.");
    lines.add(
        "- Use `subsystem_descriptions.json` to compare expected subsystem motion with the observed report.");
    return String.join("\n", lines);
  }

  public record OperatorBoardDiagnosticSnapshot(
      String overallStatus,
      String systemSummary,
      String autoSummary,
      String mechanismSummary,
      String runtimeProfileStatus,
      String actionStatus,
      String mode,
      String systemCheck,
      String autoCheck,
      String ntDiagnostics,
      String mechanismStatus,
      String actionTrace,
      String runtimeProfile,
      String selectedAuto,
      String autoQueue,
      String subsystemDescriptions) {

    public String fingerprint() {
      return String.join(
          "|",
          normalize(overallStatus),
          normalize(systemSummary),
          normalize(autoSummary),
          normalize(mechanismSummary),
          normalize(runtimeProfileStatus),
          normalize(actionStatus),
          normalize(mode),
          normalize(systemCheck),
          normalize(autoCheck),
          normalize(ntDiagnostics),
          normalize(mechanismStatus),
          normalize(actionTrace),
          normalize(runtimeProfile),
          normalize(selectedAuto),
          normalize(autoQueue),
          normalize(subsystemDescriptions));
    }

    private static String normalize(String value) {
      return value == null ? "" : value;
    }
  }
}
