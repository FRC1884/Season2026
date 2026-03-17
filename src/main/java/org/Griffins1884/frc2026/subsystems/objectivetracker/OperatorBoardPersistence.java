package org.Griffins1884.frc2026.subsystems.objectivetracker;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;

public final class OperatorBoardPersistence {
  public static final ObjectMapper JSON =
      new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
  private static final DateTimeFormatter BACKUP_TIME =
      DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss").withZone(ZoneOffset.UTC);

  private static final String JOYSTICK_MAPPINGS_FILE = "joystick-mappings.json";
  private static final String SUBSYSTEM_DESCRIPTIONS_FILE = "subsystem-descriptions.json";
  private static final String LATEST_DIAGNOSTIC_MANIFEST_FILE = "latest-diagnostic.json";

  private final Path projectRoot;
  private final Path runtimeRoot;
  private final Path backupsRoot;
  private final Path diagnosticsRoot;
  private final Path diagnosticBundlesRoot;
  private final Path deployDefaultsRoot;
  private final Path pathPlanAutosRoot;

  public OperatorBoardPersistence() {
    this.projectRoot = Path.of(System.getProperty("user.dir")).toAbsolutePath().normalize();
    this.runtimeRoot = projectRoot.resolve("operatorboard-data");
    this.backupsRoot = runtimeRoot.resolve("backups");
    this.diagnosticsRoot = runtimeRoot.resolve("diagnostics");
    this.diagnosticBundlesRoot = diagnosticsRoot.resolve("bundles");
    this.deployDefaultsRoot =
        Filesystem.getDeployDirectory().toPath().resolve("operatorboard").resolve("default-data");
    this.pathPlanAutosRoot =
        Filesystem.getDeployDirectory().toPath().resolve("pathplana").resolve("autos");
  }

  public synchronized void initialize() {
    try {
      Files.createDirectories(runtimeRoot);
      Files.createDirectories(backupsRoot);
      Files.createDirectories(diagnosticBundlesRoot);
      seedIfMissing(
          joystickMappingsPath(),
          deployDefaultsRoot.resolve(JOYSTICK_MAPPINGS_FILE),
          OperatorBoardDataModels.emptyDefaultJoystickMappings());
      seedIfMissing(
          subsystemDescriptionsPath(),
          deployDefaultsRoot.resolve(SUBSYSTEM_DESCRIPTIONS_FILE),
          OperatorBoardDataModels.emptyDefaultSubsystemDescriptions());
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  public synchronized OperatorBoardDataModels.JoystickMappingsDocument readJoystickMappings() {
    initialize();
    try {
      return JSON.readValue(
          joystickMappingsPath().toFile(), OperatorBoardDataModels.JoystickMappingsDocument.class);
    } catch (IOException ex) {
      DriverStation.reportError(
          "Failed to read joystick mappings: " + ex.getMessage(), ex.getStackTrace());
      return OperatorBoardDataModels.emptyDefaultJoystickMappings();
    }
  }

  public synchronized OperatorBoardDataModels.JoystickMappingsDocument saveJoystickMappings(
      OperatorBoardDataModels.JoystickMappingsDocument document) {
    initialize();
    OperatorBoardDataModels.JoystickMappingsDocument normalized =
        new OperatorBoardDataModels.JoystickMappingsDocument(
            OperatorBoardDataModels.SCHEMA_VERSION,
            OperatorBoardDataModels.metadata(
                "joystickMappings", "joystick-mappings", "Joystick Mappings"),
            document.activeProfileId(),
            document.targets(),
            document.profiles());
    writeWithBackup(joystickMappingsPath(), normalized);
    return normalized;
  }

  public synchronized OperatorBoardDataModels.SubsystemDescriptionsDocument
      readSubsystemDescriptions() {
    initialize();
    try {
      return JSON.readValue(
          subsystemDescriptionsPath().toFile(),
          OperatorBoardDataModels.SubsystemDescriptionsDocument.class);
    } catch (IOException ex) {
      DriverStation.reportError(
          "Failed to read subsystem descriptions: " + ex.getMessage(), ex.getStackTrace());
      return OperatorBoardDataModels.emptyDefaultSubsystemDescriptions();
    }
  }

  public synchronized OperatorBoardDataModels.SubsystemDescriptionsDocument
      saveSubsystemDescriptions(OperatorBoardDataModels.SubsystemDescriptionsDocument document) {
    initialize();
    OperatorBoardDataModels.SubsystemDescriptionsDocument normalized =
        new OperatorBoardDataModels.SubsystemDescriptionsDocument(
            OperatorBoardDataModels.SCHEMA_VERSION,
            OperatorBoardDataModels.metadata(
                "subsystemDescriptions", "subsystem-descriptions", "Subsystem Descriptions"),
            document.subsystems());
    writeWithBackup(subsystemDescriptionsPath(), normalized);
    return normalized;
  }

  public synchronized OperatorBoardDataModels.StorageInventoryDocument buildStorageInventory() {
    initialize();
    String runtimePath = runtimeRoot.toString();
    String robotPath = "/home/lvuser/operatorboard-data";
    String deployPath = deployDefaultsRoot.toString();
    return new OperatorBoardDataModels.StorageInventoryDocument(
        OperatorBoardDataModels.SCHEMA_VERSION,
        OperatorBoardDataModels.metadata(
            "storageInventory", "storage-inventory", "Storage Inventory"),
        List.of(
            new OperatorBoardDataModels.StoredAsset(
                "joystickMappings",
                "dashboard-config",
                "json",
                "local-canonical-with-roborio-mirror",
                "newer timestamp wins unless schema version differs",
                joystickMappingsPath().toString(),
                robotPath + "/" + JOYSTICK_MAPPINGS_FILE,
                deployPath + "/" + JOYSTICK_MAPPINGS_FILE,
                true,
                true,
                "Persistent joystick/controller bindings edited from the dashboard."),
            new OperatorBoardDataModels.StoredAsset(
                "subsystemDescriptions",
                "dashboard-docs",
                "json",
                "local-canonical-with-roborio-mirror",
                "manual edits merge by review when both sides changed",
                subsystemDescriptionsPath().toString(),
                robotPath + "/" + SUBSYSTEM_DESCRIPTIONS_FILE,
                deployPath + "/" + SUBSYSTEM_DESCRIPTIONS_FILE,
                true,
                true,
                "Subsystem and state descriptions shown in the operator board."),
            new OperatorBoardDataModels.StoredAsset(
                "pathPlanAAutos",
                "auto-paths",
                "json-package",
                "local-canonical-with-roborio-mirror",
                "pull robot copy before deploy, then merge missing files by path",
                pathPlanAutosRoot.toString(),
                "/home/lvuser/deploy/pathplana/autos",
                pathPlanAutosRoot.toString(),
                false,
                true,
                "Autonomous path packages served by the operator board."),
            new OperatorBoardDataModels.StoredAsset(
                "diagnosticBundles",
                "diagnostics",
                "directory",
                "roborio-authoritative-with-local-mirror",
                "never overwrite without backup; newest bundle appended",
                diagnosticBundlesRoot.toString(),
                robotPath + "/diagnostics/bundles",
                "",
                false,
                true,
                "Structured diagnostic bundles generated from system checks.")),
        new OperatorBoardDataModels.SyncWorkflow(
            "Pull robot-saved data, back up both sides, merge deterministically, then deploy code.",
            "Re-push persistent data and verify paths exist after deploy.",
            List.of(
                "If only one side has a file, copy it to the other side.",
                "If both sides changed and schema versions match, prefer the newer metadata timestamp and preserve the older copy in backups.",
                "If schema versions differ, do not auto-merge. Keep both files and mark the sync result for manual review.",
                "Diagnostic bundles append; they do not participate in destructive merge."),
            List.of(
                "timestamp",
                "buildVersion",
                "gitSha",
                "gitBranch",
                "driver/operator profile name when present")));
  }

  public synchronized Optional<OperatorBoardDataModels.DiagnosticBundleManifest>
      readLatestDiagnosticManifest() {
    initialize();
    Path manifestPath = diagnosticsRoot.resolve(LATEST_DIAGNOSTIC_MANIFEST_FILE);
    if (!Files.exists(manifestPath)) {
      return Optional.empty();
    }
    try {
      return Optional.of(
          JSON.readValue(
              manifestPath.toFile(), OperatorBoardDataModels.DiagnosticBundleManifest.class));
    } catch (IOException ex) {
      DriverStation.reportError(
          "Failed to read latest diagnostic manifest: " + ex.getMessage(), ex.getStackTrace());
      return Optional.empty();
    }
  }

  public synchronized List<OperatorBoardDataModels.DiagnosticBundleManifest> listDiagnosticBundles(
      int limit) {
    initialize();
    int boundedLimit = Math.max(limit, 1);
    try (Stream<Path> children = Files.list(diagnosticBundlesRoot)) {
      return children
          .filter(Files::isDirectory)
          .sorted(Comparator.reverseOrder())
          .limit(boundedLimit)
          .map(path -> path.resolve("manifest.json"))
          .filter(Files::exists)
          .map(this::readDiagnosticManifestUnchecked)
          .toList();
    } catch (IOException ex) {
      DriverStation.reportError(
          "Failed to list diagnostic bundles: " + ex.getMessage(), ex.getStackTrace());
      return List.of();
    }
  }

  public synchronized void writeLatestDiagnosticManifest(
      OperatorBoardDataModels.DiagnosticBundleManifest manifest) {
    initialize();
    Path target = diagnosticsRoot.resolve(LATEST_DIAGNOSTIC_MANIFEST_FILE);
    writeJson(target, manifest);
  }

  public Path runtimeRoot() {
    return runtimeRoot;
  }

  public Path diagnosticBundlesRoot() {
    return diagnosticBundlesRoot;
  }

  private OperatorBoardDataModels.DiagnosticBundleManifest readDiagnosticManifestUnchecked(
      Path path) {
    try {
      return JSON.readValue(path.toFile(), OperatorBoardDataModels.DiagnosticBundleManifest.class);
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  private void seedIfMissing(Path runtimePath, Path seedPath, Object fallbackValue)
      throws IOException {
    if (Files.exists(runtimePath)) {
      return;
    }
    Files.createDirectories(runtimePath.getParent());
    if (Files.exists(seedPath)) {
      Files.copy(seedPath, runtimePath, StandardCopyOption.REPLACE_EXISTING);
      return;
    }
    writeJson(runtimePath, fallbackValue);
  }

  private void writeWithBackup(Path target, Object value) {
    try {
      backupIfPresent(target);
      writeJson(target, value);
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  private void backupIfPresent(Path source) throws IOException {
    if (!Files.exists(source)) {
      return;
    }
    Files.createDirectories(backupsRoot);
    String fileName = source.getFileName().toString();
    String timestamp = BACKUP_TIME.format(Instant.now());
    Path target = backupsRoot.resolve(fileName + "." + timestamp + ".bak");
    Files.copy(source, target, StandardCopyOption.REPLACE_EXISTING);
  }

  private void writeJson(Path target, Object value) {
    try {
      Files.createDirectories(target.getParent());
      JSON.writeValue(target.toFile(), value);
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  private Path joystickMappingsPath() {
    return runtimeRoot.resolve(JOYSTICK_MAPPINGS_FILE);
  }

  private Path subsystemDescriptionsPath() {
    return runtimeRoot.resolve(SUBSYSTEM_DESCRIPTIONS_FILE);
  }
}
