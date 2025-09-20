package frc.robot.auto;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.NamedTargets;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Minimal autonomy coordinator: waits for a coral load signal (by intake current rise), then drives
 * to a selected reef branch and completes placement.
 *
 * <p>Notes: This is intentionally simple and safe. It only triggers when autonomous is enabled in
 * the Web UI and the robot is enabled. It uses a NetworkTables current subscriber (table="Intake",
 * topic="CurrentAmps") to detect a load event. Hook your intake motor current to publish on that
 * topic for automatic triggering.
 */
public class AutonomyManager {
  public enum State {
    IDLE,
    WAIT_FOR_LOAD,
    DRIVING_TO_REEF,
    COMPLETE
  }

  private final TaskRegistry registry;
  private final SwerveSubsystem drive;
  private final Superstructure superstructure;

  // Simple detection
  private final DoubleSubscriber currentAmpsSub;
  private double lastCurrent = 0.0;
  // Simple EMA baseline for debounce
  private double emaCurrent = 0.0;
  private boolean emaInit = false;
  private final double emaAlpha = 0.2; // 0..1, higher reacts faster
  private final double detectDeltaAmps;
  private final Timer debounceTimer = new Timer();
  private final double debounceSeconds = 0.15; // require sustained delta

  private State state = State.IDLE;
  private TaskRegistry.ReefBranch activeBranch = null;
  private Command activeCommand = null;
  private Command toSourceCommand = null;
  private boolean activeWasScheduled = false;
  private boolean toSourceWasScheduled = false;
  private boolean routedToSourceOnce = false; // avoid re-scheduling loops while waiting
  private boolean hasCoral = false;
  private int placedCount = 0;
  private boolean simAwaitingPlaceConfirm = false; // SIM: wait for periodic place completion

  // --- Fill-the-reef autonomous mode ---
  // When enabled, ignore branch claim/done state and systematically place coral
  // by level (L4 -> L1) across all reef faces, alternating sides A/B per face.
  private final boolean fillReefMode = true; // default enabled per request
  private final boolean loopAfterFill = false; // when false, stop after one full fill
  private TaskRegistry.CoralLevel currentLevel = TaskRegistry.CoralLevel.L4;
  private int currentFaceIdx = 1; // 1..6
  private int currentSideIdx = 0; // 0 = LEFT (A), 1 = RIGHT (B)
  private boolean fillComplete = false;

  private static class ReefTarget {
    int faceIndex; // 1..6
    TaskRegistry.ApproachSide side; // LEFT = A, RIGHT = B
    TaskRegistry.CoralLevel level; // L4..L1
  }

  private ReefTarget nextFillTarget() {
    if (fillComplete) return null;
    ReefTarget t = new ReefTarget();
    t.faceIndex = currentFaceIdx;
    t.side =
        (currentSideIdx == 0) ? TaskRegistry.ApproachSide.LEFT : TaskRegistry.ApproachSide.RIGHT;
    t.level = currentLevel;

    // Advance pointer for next time
    currentSideIdx++;
    if (currentSideIdx > 1) { // wrap sides -> advance face
      currentSideIdx = 0;
      currentFaceIdx++;
      if (currentFaceIdx > 6) { // wrap faces -> advance level
        currentFaceIdx = 1;
        switch (currentLevel) {
          case L4 -> currentLevel = TaskRegistry.CoralLevel.L3;
          case L3 -> currentLevel = TaskRegistry.CoralLevel.L2;
          case L2 -> currentLevel = TaskRegistry.CoralLevel.L1;
          case L1 -> {
            if (loopAfterFill) {
              currentLevel = TaskRegistry.CoralLevel.L4; // loop
            } else {
              fillComplete = true;
            }
          }
        }
      }
    }
    return t;
  }

  public AutonomyManager(TaskRegistry registry, SwerveSubsystem drive) {
    this(registry, drive, null, 8.0); // default 8A delta
  }

  public AutonomyManager(
      TaskRegistry registry,
      SwerveSubsystem drive,
      Superstructure superstructure,
      double detectDeltaAmps) {
    this.registry = registry;
    this.drive = drive;
    this.superstructure = superstructure;
    this.detectDeltaAmps = detectDeltaAmps;
    var table = NetworkTableInstance.getDefault().getTable("Intake");
    this.currentAmpsSub = table.getDoubleTopic("CurrentAmps").subscribe(0.0);
    debounceTimer.start();
  }

  public void periodic() {
    // Only run if DS is enabled and UI autonomous toggle is on
    if (!DriverStation.isEnabled() || !registry.isAutonomousEnabled()) {
      // Only act on the transition into the disabled/auto-off condition
      if (state != State.IDLE) {
        switchState(State.IDLE, "DisabledOrAutoOff");
      }
      return;
    }

    // Operator-issued send request: cancel everything and immediately go to named/selected target
    if (registry.consumeSendRequest()) {
      cancelAllAutos("SendRequest");
      String name = registry.getTargetName();
      if (name == null || name.isBlank()) {
        // Derive from selection Reef_{1..6}_{L|R}_Blue
        int faceIdx = mapFaceToIndex(registry.getSelectedFace());
        String sideLabel =
            (registry.getSelectedSide() == TaskRegistry.ApproachSide.LEFT) ? "L" : "R";
        name = String.format("Reef_%d_%s_Blue", faceIdx, sideLabel);
        registry.setTargetName(name);
      }
      switchState(State.DRIVING_TO_REEF, "SendRequest");
      activeCommand = NamedTargets.goTo(drive, name);
      safeRecord("Autonomy/Schedule", "GoTo(Send): " + name);
      activeWasScheduled = false;
      activeCommand.schedule();
      return;
    }

    switch (state) {
      case IDLE -> {
        // Transition to WAIT_FOR_LOAD immediately when enabled
        switchState(State.WAIT_FOR_LOAD, "Enabled");
      }

      case WAIT_FOR_LOAD -> {
        // If not already routed to source once, prefer named Choreo target "Source_Blue"
        if (!routedToSourceOnce && (toSourceCommand == null || !toSourceCommand.isScheduled())) {
          cancelAllAutos("WAIT_FOR_LOAD -> RouteToSource");
          registry.setTargetName("Source_Blue");
          toSourceCommand = NamedTargets.goTo(drive, "Source_Blue");
          safeRecord("Autonomy/Schedule", "GoTo: Source_Blue");
          toSourceWasScheduled = false;
          toSourceCommand.schedule();
          routedToSourceOnce = true;
          // Start intake while heading to source (handled in superstructure periodic)
          if (superstructure != null) {
            superstructure.startIntake();
          }
        }

        // Promote flag once command is actually scheduled
        if (toSourceCommand != null && toSourceCommand.isScheduled()) {
          toSourceWasScheduled = true;
        }
        boolean reachedSource =
            (toSourceCommand != null && toSourceWasScheduled && !toSourceCommand.isScheduled());
        boolean loadDetected = false;
        // Manual override via WebUI works in all modes
        if (registry.consumeLoadTrigger()) {
          loadDetected = true;
        } else if (frc.robot.GlobalConstants.MODE == frc.robot.GlobalConstants.RobotMode.SIM) {
          if (reachedSource) {
            // Let Superstructure.periodic manage the SIM 0.2s intake window
            if (superstructure == null || superstructure.consumeSimIntakeComplete()) {
              loadDetected = true;
            } else {
              return; // keep waiting in this state until intake completes
            }
          }
        } else {
          loadDetected = detectLoadEvent();
        }

        if (loadDetected) {
          // Hard-cancel all running autos before switching to reef target
          cancelAllAutos("LoadDetected -> ReefTarget");
          hasCoral = true;
          safeRecord("Autonomy/HasCoral", true);
          toSourceCommand = null; // ensure we don't keep routing to source
          if (superstructure != null) superstructure.stopIntake();
          if (fillReefMode) {
            ReefTarget t = nextFillTarget();
            if (t == null) {
              // Finished full fill sequence
              state = State.COMPLETE;
              safeRecord("Autonomy/State", "COMPLETE");
              return;
            }
            // Build target name, publish to UI, and go to it via Choreo/PathPlanner if available
            String sideLabel = (t.side == TaskRegistry.ApproachSide.LEFT) ? "L" : "R";
            String targetName = String.format("Reef_%d_%s_Blue", t.faceIndex, sideLabel);
            registry.setSelectedFace(faceFromIndex(t.faceIndex));
            registry.setSelectedSide(t.side);
            registry.setSelectedLevel(t.level);
            registry.setTargetName(targetName);
            safeRecord("Autonomy/ChoreoTarget", targetName);
            try {
              // Preview the next target pose on a separate key to avoid duplicate setpoints
              org.littletonrobotics.junction.Logger.recordOutput(
                  "Autonomy/TargetPreviewPose",
                  frc.robot.GlobalConstants.FieldMap.Coordinates.valueOf("REEF_" + t.faceIndex)
                      .getPose());
            } catch (Throwable ignored) {
            }
            // Schedule after the state switch below to avoid cancelling our new command
            // via the state transition cleanup
            // activeCommand created after switchState
          } else {
            // Legacy claim/done mode
            Optional<TaskRegistry.ReefBranch> claimed = registry.getFirstClaimed();
            TaskRegistry.ReefBranch target =
                claimed.isPresent() ? claimed.get() : registry.claimNextFree().orElse(null);
            if (target == null) return; // nothing to do yet

            activeBranch = target;
            safeRecord("Autonomy/TargetBranch", activeBranch.name());

            int faceIndex = mapFaceToIndex(activeBranch.face);
            int branchOffsetIndex = mapBranchToOffset(activeBranch);

            // Defer building/scheduling until after the state switch below to avoid self-cancel
            final int deferredFace = faceIndex;
            final int deferredOffset = branchOffsetIndex;
            // Stash in registry for visibility (optional)
            safeRecord(
                "Autonomy/Schedule",
                String.format(
                    "AlignToReefBranch: face=%d, offset=%d", deferredFace, deferredOffset));
            // Create a placeholder marker by clearing activeCommand; it will be created below
            activeCommand = null;

            // After switchState, we will build: DriveCommands.alignToReefBranchCommandAuto(...)
            // using deferredFace/deferredOffset
            // We'll reconstruct below where we schedule.
            // To access deferred values, we replicate them after switch via local final copies.
            // Values are captured in this scope.
          }
          switchState(State.DRIVING_TO_REEF, "LoadDetected");
          if (fillReefMode) {
            String sideLabel =
                (registry.getSelectedSide() == TaskRegistry.ApproachSide.LEFT) ? "L" : "R";
            String tn = registry.getTargetName(); // already set above
            if (tn == null || tn.isBlank()) {
              int faceIdx = mapFaceToIndex(registry.getSelectedFace());
              tn = String.format("Reef_%d_%s_Blue", faceIdx, sideLabel);
              registry.setTargetName(tn);
            }
            activeCommand = NamedTargets.goTo(drive, tn);
            safeRecord("Autonomy/Schedule", "GoTo: " + tn);
          } else if (activeBranch != null) {
            int faceIndex = mapFaceToIndex(activeBranch.face);
            int branchOffsetIndex = mapBranchToOffset(activeBranch);
            activeCommand =
                DriveCommands.alignToReefBranchCommandAuto(drive, faceIndex, branchOffsetIndex);
          }
          if (activeCommand != null) {
            activeWasScheduled = false;
            activeCommand.schedule();
          }
        }
      }

      case DRIVING_TO_REEF -> {
        // Manual release request from WebUI
        if (registry.consumeReleaseTrigger()) {
          cancelAllAutos("ManualRelease Triggered");
          if (superstructure != null) superstructure.startPlace();
          safeRecord("Autonomy/Event", "PlaceStart: ManualRelease");
          if (hasCoral) {
            hasCoral = false;
            placedCount++;
            safeRecord("Autonomy/HasCoral", false);
            safeRecord("Autonomy/PlacedCount", placedCount);
          }
          switchState(State.COMPLETE, "ManualRelease");
          return;
        }
        // Track if the active command was truly scheduled at least once
        if (activeCommand != null && activeCommand.isScheduled()) {
          activeWasScheduled = true;
        }
        // When finished (or interrupted), mark placement complete
        if (activeWasScheduled && (activeCommand == null || !activeCommand.isScheduled())) {
          if (!fillReefMode && activeBranch != null) {
            registry.markDone(activeBranch);
          }
          activeBranch = null;
          activeCommand = null;
          toSourceCommand = null;
          activeWasScheduled = false;
          if (frc.robot.GlobalConstants.MODE == frc.robot.GlobalConstants.RobotMode.SIM) {
            // Let Superstructure.periodic handle the 0.2s release; wait until it reports done
            if (!simAwaitingPlaceConfirm) {
              simAwaitingPlaceConfirm = true;
              if (superstructure != null) superstructure.startPlace();
            }
            if (superstructure == null || superstructure.consumeSimPlaceComplete()) {
              if (hasCoral) {
                hasCoral = false;
                placedCount++;
                safeRecord("Autonomy/HasCoral", false);
                safeRecord("Autonomy/PlacedCount", placedCount);
              }
              simAwaitingPlaceConfirm = false;
              switchState(State.COMPLETE, "AutoPlaceComplete(SIM)");
            } else {
              return; // wait another periodic until place completes
            }
          } else {
            // Real robot: trigger place and complete immediately
            if (superstructure != null) superstructure.startPlace();
            if (hasCoral) {
              hasCoral = false;
              placedCount++;
              safeRecord("Autonomy/HasCoral", false);
              safeRecord("Autonomy/PlacedCount", placedCount);
            }
            switchState(State.COMPLETE, "AutoPlaceComplete");
          }
        }
      }

      case COMPLETE -> {
        // Immediately return to waiting for the next load
        switchState(State.WAIT_FOR_LOAD, "CycleComplete");
      }
    }
  }

  public void cancelActive() {
    if (activeCommand != null && activeCommand.isScheduled()) {
      safeRecord("Autonomy/Event", "CancelActive command");
      activeCommand.cancel();
    }
    activeCommand = null;
    activeBranch = null;
  }

  private void cancelAllAutos(String reason) {
    try {
      safeRecord("Autonomy/Event", "CancelAll: " + reason);
      CommandScheduler.getInstance().cancelAll();
      // Also clear align telemetry to avoid stale UI values lingering
      try {
        frc.robot.commands.DriveCommands.clearAlignTelemetry();
      } catch (Throwable ignored) {
      }
      // Clear references so logic doesn't think something is still in-flight
      activeCommand = null;
      toSourceCommand = null;
      activeBranch = null;
      activeWasScheduled = false;
      toSourceWasScheduled = false;
    } catch (Throwable ignored) {
    }
  }

  private boolean detectLoadEvent() {
    // UI-triggered (sim/test) load event
    if (registry.consumeLoadTrigger()) return true;

    double amps = currentAmpsSub.get();
    if (!emaInit) {
      emaCurrent = amps;
      emaInit = true;
    }
    double delta = amps - emaCurrent;
    boolean rising = delta >= detectDeltaAmps;
    boolean sustained = rising && debounceTimer.hasElapsed(debounceSeconds);
    if (rising) {
      if (sustained) debounceTimer.reset();
    } else {
      // reset debounce window when not rising to require sustained rise
      debounceTimer.reset();
    }
    lastCurrent = amps;
    emaCurrent = emaCurrent * (1.0 - emaAlpha) + emaAlpha * amps;
    safeRecord("Autonomy/IntakeCurrent", amps);
    safeRecord("Autonomy/LoadDetected", sustained);
    return sustained;
  }

  private static int mapFaceToIndex(TaskRegistry.ReefFace face) {
    return switch (face) {
      case REEF_1 -> 1;
      case REEF_2 -> 2;
      case REEF_3 -> 3;
      case REEF_4 -> 4;
      case REEF_5 -> 5;
      case REEF_6 -> 6;
    };
  }

  private static int mapBranchToOffset(TaskRegistry.ReefBranch b) {
    // Two positions per face in 2025: A (left) and B (right)
    String name = b.name();
    if (name.endsWith("_A")) return -1;
    return 1; // _B
  }

  private static TaskRegistry.ReefFace faceFromIndex(int idx) {
    return switch (idx) {
      case 1 -> TaskRegistry.ReefFace.REEF_1;
      case 2 -> TaskRegistry.ReefFace.REEF_2;
      case 3 -> TaskRegistry.ReefFace.REEF_3;
      case 4 -> TaskRegistry.ReefFace.REEF_4;
      case 5 -> TaskRegistry.ReefFace.REEF_5;
      case 6 -> TaskRegistry.ReefFace.REEF_6;
      default -> TaskRegistry.ReefFace.REEF_1;
    };
  }

  // Avoid crashes on desktop when logger/HAL isn't initialized
  private static void safeRecord(String key, String value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
    }
  }

  private static void safeRecord(String key, boolean value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
    }
  }

  private static void safeRecord(String key, double value) {
    try {
      Logger.recordOutput(key, value);
    } catch (Throwable t) {
    }
  }

  private void switchState(State newState, String reason) {
    if (this.state == newState) return;
    // Cancel everything on state switch to guarantee no overlap
    cancelAllAutos(
        "StateSwitch "
            + this.state
            + " -> "
            + newState
            + (reason == null ? "" : " (" + reason + ")"));
    this.state = newState;
    // Reset per-state flags
    if (newState == State.WAIT_FOR_LOAD) {
      routedToSourceOnce = false;
      toSourceWasScheduled = false;
      activeWasScheduled = false;
    }
    safeRecord("Autonomy/State", newState.name());
  }
}
