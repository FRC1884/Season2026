package org.Griffins1884.frc2026.subsystems.objectivetracker;

import java.time.Instant;
import java.util.List;
import org.Griffins1884.frc2026.BuildConstants;

public final class OperatorBoardDataModels {
  public static final String SCHEMA_VERSION = "2026.04";

  private OperatorBoardDataModels() {}

  public static AssetMetadata metadata(String type, String id, String displayName) {
    return new AssetMetadata(
        id,
        type,
        displayName,
        Instant.now().toString(),
        System.currentTimeMillis(),
        BuildConstants.VERSION,
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_BRANCH);
  }

  public static SubsystemDescriptionsDocument emptyDefaultSubsystemDescriptions() {
    return new SubsystemDescriptionsDocument(
        SCHEMA_VERSION,
        metadata("subsystemDescriptions", "subsystem-descriptions", "Subsystem Descriptions"),
        List.of(
            new SubsystemDescription(
                "superstructure",
                "Superstructure",
                "Coordinates intake, indexer, shooter, pivots, and turret into named operating states.",
                List.of(
                    "Intake rollers",
                    "Indexer",
                    "Shooter",
                    "Intake pivot",
                    "Shooter pivot",
                    "Turret"),
                List.of(
                    "Ball detection",
                    "Pivot position feedback",
                    "Turret position feedback",
                    "Vision pose visibility"),
                List.of(
                    "Requests can be rejected when the target state is unsafe or unavailable.",
                    "Teleop override and auto-state selection must not fight each other."),
                List.of("Dashboard state requests", "Auto state selection", "Driver quick actions"),
                List.of(
                    "Drivers see the robot settle into a predictable intake/shoot/ferry behavior.",
                    "Debuggers can compare requested state versus actual state."),
                List.of(
                    new StateDescription(
                        "IDLING",
                        "Neutral ready state.",
                        "Rollers idle, pivots hold safe positions, turret holds target or safe orientation.",
                        "No queued motion or unsafe actuator motion remains.",
                        "Entered when no higher-priority action is active.",
                        "Any hard fault or unavailable subsystem can force fallback behavior."),
                    new StateDescription(
                        "INTAKING",
                        "Acquire game pieces.",
                        "Intake deploys, intake rollers and indexer run to bring fuel into storage.",
                        "Ball path is clear and intake pivot reaches deploy pose.",
                        "Leaves when fuel is secured or operator requests another state.",
                        "Sensor disagreements, intake pivot zeroing, or jams can block it."),
                    new StateDescription(
                        "SHOOTING",
                        "Prepare and execute scoring shots.",
                        "Shooter spins to target, pivots move to shot angle, turret tracks shot target.",
                        "Shooter and turret are at goal and shoot gate is enabled.",
                        "Leaves when operator disables shooting or state changes.",
                        "Missing vision, turret faults, or shooter not at speed prevent readiness."),
                    new StateDescription(
                        "SHOOT_INTAKE",
                        "Hybrid scoring and intake assist behavior.",
                        "Intake path stays available while shooter path prepares for close-range feed.",
                        "Shooter and intake path agree on commanded handoff state.",
                        "Leaves when handoff is complete or operator requests another mode.",
                        "Ball path jams or conflicting pivot requirements block action."),
                    new StateDescription(
                        "FERRYING",
                        "Move pieces to a field target outside the main hub zone.",
                        "Turret and shooter track ferry target, drivetrain stays field-oriented.",
                        "Turret target resolves and shooter is within ferry tolerance.",
                        "Leaves when robot returns to hub workflow or idles.",
                        "Target ambiguity or missing pose estimate blocks action."),
                    new StateDescription(
                        "TESTING",
                        "Controlled diagnostic state.",
                        "Subsystems expose manual or scripted test behavior for validation.",
                        "Requested test command is active and subsystem is healthy enough to move.",
                        "Leaves immediately on manual cancel or safety failure.",
                        "Any fault, zeroing routine, or disabled mode should block movement."))),
            new SubsystemDescription(
                "intake",
                "Intake",
                "Collects fuel from the floor and feeds the indexer.",
                List.of("Intake roller motor", "Intake pivot"),
                List.of("Motor current", "Pivot encoder", "Ball present signal"),
                List.of(
                    "Do not drive rollers aggressively when the intake is stowed.",
                    "Keep zeroing routines exclusive with manual motion."),
                List.of("Intaking", "Shoot intake", "Manual deploy/hold commands"),
                List.of(
                    "Fuel enters the robot cleanly when deployed.",
                    "Drivers see deploy and roller hold indicators change together."),
                List.of(
                    new StateDescription(
                        "DEPLOYED",
                        "Floor collection geometry active.",
                        "Pivot moves to deploy angle and rollers can spin inward.",
                        "Deploy angle reached and no zeroing routine is active.",
                        "Triggered by intake deploy toggle or intake-capable superstates.",
                        "Zeroing, sensor faults, or soft-limit violations block deployment."),
                    new StateDescription(
                        "STOWED",
                        "Safe travel position.",
                        "Pivot retracts and rollers either idle or briefly stow-roll if commanded.",
                        "Pivot reaches stow target without active collision risk.",
                        "Triggered when intake is released or robot returns to idle.",
                        "Faulted pivot feedback or blocked travel prevents confirmation."))),
            new SubsystemDescription(
                "shooter",
                "Shooter",
                "Accelerates fuel to the commanded scoring or ferry velocity.",
                List.of("Shooter flywheels", "Shooter pivot", "Turret"),
                List.of("Velocity feedback", "Pivot encoder", "Turret feedback"),
                List.of(
                    "Only fire when shooter velocity, pivot, and turret readiness agree.",
                    "Respect vision/pose validity before long-range shots."),
                List.of("Shooting", "Ferrying", "Characterization"),
                List.of(
                    "Drivers see a ready-to-shoot condition instead of guessing.",
                    "System checks can explain not-at-speed or not-at-angle issues."),
                List.of(
                    new StateDescription(
                        "SPINUP",
                        "Reach target velocity before feed.",
                        "Flywheels accelerate toward commanded RPM while turret and pivot settle.",
                        "Measured velocity is inside the configured tolerance band.",
                        "Starts when a shooting-capable state is requested.",
                        "Motor disconnects, overcurrent, or target changes hold readiness low."),
                    new StateDescription(
                        "READY",
                        "All shot prerequisites satisfied.",
                        "Velocity, angle, and turret alignment stay within tolerance.",
                        "Shooter ready latch is active and feed path is allowed.",
                        "Transitions to feed when operator or auto command requests fire.",
                        "Any subsystem dropping out of tolerance should cancel readiness."))),
            new SubsystemDescription(
                "turret",
                "Turret",
                "Rotates the shooter toward hub, ferry, or testing targets.",
                List.of("Turret motor"),
                List.of("Position encoder", "Field pose", "Vision target estimate"),
                List.of(
                    "Soft limits must remain enforced.",
                    "Unsafe manual commands must yield to zeroing/fault handling."),
                List.of("Auto aim", "Ferry aim", "Testing"),
                List.of(
                    "Operators get stable aiming feedback.",
                    "Readiness depends on at-goal confirmation."),
                List.of(
                    new StateDescription(
                        "TRACKING",
                        "Closed-loop target following.",
                        "Turret rotates toward a computed target and holds there.",
                        "Position error is within turret goal tolerance.",
                        "Starts during hub/ferry aim commands.",
                        "Pose uncertainty or turret health faults can block tracking."),
                    new StateDescription(
                        "HOLD",
                        "Safe stationary hold.",
                        "Turret resists drift at the current or parked heading.",
                        "Control loop remains stable without new target updates.",
                        "Used in idle, disabled, or fallback behavior.",
                        "Encoder loss or faulted health prevents trustworthy hold.")))));
  }

  public record AssetMetadata(
      String id,
      String type,
      String displayName,
      String updatedAtIso,
      long updatedAtEpochMs,
      String buildVersion,
      String gitSha,
      String gitBranch) {}

  public record SubsystemDescriptionsDocument(
      String schemaVersion, AssetMetadata metadata, List<SubsystemDescription> subsystems) {}

  public record SubsystemDescription(
      String key,
      String name,
      String purpose,
      List<String> actuators,
      List<String> sensors,
      List<String> safetyConditions,
      List<String> commands,
      List<String> operatorOutcomes,
      List<StateDescription> states) {}

  public record StateDescription(
      String name,
      String purpose,
      String expectedMotion,
      String readyCondition,
      String transitionConditions,
      String faultBlockers) {}

  public record StorageInventoryDocument(
      String schemaVersion,
      AssetMetadata metadata,
      List<StoredAsset> assets,
      SyncWorkflow syncWorkflow) {}

  public record StoredAsset(
      String key,
      String category,
      String format,
      String authority,
      String conflictPolicy,
      String localPath,
      String robotPath,
      String deployPath,
      boolean editableFromDashboard,
      boolean includedInDeployPreserve,
      String description) {}

  public record SyncWorkflow(
      String preDeploySummary,
      String postDeploySummary,
      List<String> conflictRules,
      List<String> versionTags) {}

  public record DiagnosticBundleManifest(
      String schemaVersion,
      AssetMetadata metadata,
      String bundleId,
      String bundleDirectory,
      String overallStatus,
      List<String> files) {}

  public record DiagnosticBundleSummary(
      String schemaVersion,
      AssetMetadata metadata,
      String overallStatus,
      String summary,
      String mode,
      String actionStatus,
      String autoSummary,
      String mechanismSummary,
      String runtimeProfileStatus) {}

  public record DiagnosticObservedData(
      String systemCheck,
      String autoCheck,
      String ntDiagnostics,
      String mechanismStatus,
      String actionTrace,
      String runtimeProfile,
      String selectedAuto,
      String autoQueue,
      String subsystemDescriptions) {}
}
