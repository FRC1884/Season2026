package frc.robot.auto;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

/**
 * Tracks reef branch completion, selection/claiming, and autonomous enable state. Exposes state on
 * NetworkTables so UI or other tools can observe/control.
 */
public class TaskRegistry {
  public enum BranchState {
    FREE,
    CLAIMED,
    DONE
  }

  public enum ReefFace {
    REEF_1,
    REEF_2,
    REEF_3,
    REEF_4,
    REEF_5,
    REEF_6;
  }

  public enum ReefBranch {
    // Two scoring positions per face -> 12 total
    F1_A(ReefFace.REEF_1),
    F1_B(ReefFace.REEF_1),
    F2_A(ReefFace.REEF_2),
    F2_B(ReefFace.REEF_2),
    F3_A(ReefFace.REEF_3),
    F3_B(ReefFace.REEF_3),
    F4_A(ReefFace.REEF_4),
    F4_B(ReefFace.REEF_4),
    F5_A(ReefFace.REEF_5),
    F5_B(ReefFace.REEF_5),
    F6_A(ReefFace.REEF_6),
    F6_B(ReefFace.REEF_6);

    public final ReefFace face;

    ReefBranch(ReefFace face) {
      this.face = face;
    }
  }

  private final EnumMap<ReefBranch, BranchState> states = new EnumMap<>(ReefBranch.class);
  private final Lock lock = new ReentrantLock();
  private final AtomicBoolean autonomousEnabled = new AtomicBoolean(false);
  private final AtomicBoolean simulatedLoadTrigger = new AtomicBoolean(false);
  private final AtomicBoolean simulatedReleaseTrigger = new AtomicBoolean(false);

  public enum CoralSourcePreference {
    NEAREST,
    LEFT,
    RIGHT
  }

  @Getter @Setter
  private volatile CoralSourcePreference sourcePreference = CoralSourcePreference.NEAREST;
  /** Which reef face the operator has targeted (optional hint for autonomy). */
  public enum ApproachSide {
    LEFT,
    RIGHT
  }

  public enum CoralLevel {
    L1,
    L2,
    L3,
    L4
  }

  @Getter private volatile ReefFace selectedFace = ReefFace.REEF_1;
  @Getter private volatile ApproachSide selectedSide = ApproachSide.LEFT; // A = LEFT, B = RIGHT
  @Getter private volatile CoralLevel selectedLevel = CoralLevel.L2;
  private volatile boolean sendRequested = false;
  private final NetworkTable table; // May be null on desktop tests
  private final EnumMap<ReefBranch, BooleanPublisher> branchDonePubs =
      new EnumMap<>(ReefBranch.class);
  private BooleanPublisher autonomousEnabledPub = null;
  private StringPublisher targetFacePub = null;
  private StringPublisher targetSidePub = null;
  private StringPublisher targetLevelPub = null;
  @Getter private volatile String targetName = "";
  private StringPublisher targetNamePub = null;

  // UI->Robot control (subscribers)
  private BooleanSubscriber ctlAutonomousEnabledSub = null;
  private StringSubscriber ctlTargetFaceSub = null;
  private StringSubscriber ctlTargetSideSub = null;
  private StringSubscriber ctlTargetLevelSub = null;
  private StringSubscriber ctlTargetNameSub = null;
  private BooleanSubscriber ctlTriggerLoadSub = null;
  private BooleanSubscriber ctlTriggerReleaseSub = null;
  private BooleanSubscriber ctlSendSub = null;
  private boolean lastCtlTriggerLoad = false;
  private boolean lastCtlTriggerRelease = false;
  private boolean lastCtlSend = false;

  public TaskRegistry() {
    this(true);
  }

  public TaskRegistry(boolean enableNetworkTables) {
    NetworkTable t = null;
    if (enableNetworkTables) {
      try {
        t = NetworkTableInstance.getDefault().getTable("CoralRegistry");
      } catch (Throwable ex) {
        t = null; // NetworkTables native lib not available on desktop
      }
    }
    table = t;
    for (var b : ReefBranch.values()) {
      states.put(b, BranchState.FREE);
    }
    if (table != null) {
      // Create persistent publishers (avoid churn/leaks)
      for (var b : ReefBranch.values()) {
        BooleanPublisher pub = table.getBooleanTopic(topicFor(b)).publish();
        pub.set(false);
        branchDonePubs.put(b, pub);
      }
      autonomousEnabledPub = table.getBooleanTopic("AutonomousEnabled").publish();
      autonomousEnabledPub.set(false);
      targetFacePub = table.getStringTopic("TargetFace").publish();
      targetSidePub = table.getStringTopic("TargetSide").publish();
      targetLevelPub = table.getStringTopic("TargetLevel").publish();
      targetNamePub = table.getStringTopic("TargetName").publish();

      // UI->Robot control subscribers under a Control namespace to avoid echo loops
      NetworkTable ctl = table.getSubTable("Control");
      ctlAutonomousEnabledSub = ctl.getBooleanTopic("AutonomousEnabled").subscribe(false);
      ctlTargetFaceSub = ctl.getStringTopic("TargetFace").subscribe("");
      ctlTargetSideSub = ctl.getStringTopic("TargetSide").subscribe("");
      ctlTargetLevelSub = ctl.getStringTopic("TargetLevel").subscribe("");
      ctlTargetNameSub = ctl.getStringTopic("TargetName").subscribe("");
      ctlTriggerLoadSub = ctl.getBooleanTopic("TriggerLoad").subscribe(false);
      ctlTriggerReleaseSub = ctl.getBooleanTopic("TriggerRelease").subscribe(false);
      ctlSendSub = ctl.getBooleanTopic("Send").subscribe(false);
    }
    publishSelection();
  }

  public void setAutonomousEnabled(boolean enabled) {
    autonomousEnabled.set(enabled);
    if (autonomousEnabledPub != null) autonomousEnabledPub.set(enabled);
    safeRecord("CoralRegistry/AutonomousEnabled", enabled);
  }

  public boolean isAutonomousEnabled() {
    return autonomousEnabled.get();
  }

  // Trigger a simulated coral load event (useful in sim/testing)
  public void triggerLoad() {
    simulatedLoadTrigger.set(true);
  }

  // Returns true if a simulated load was triggered since last check (and resets the flag)
  public boolean consumeLoadTrigger() {
    return simulatedLoadTrigger.getAndSet(false);
  }

  // Trigger a simulated coral release/placement event (useful in sim/testing)
  public void triggerRelease() {
    simulatedReleaseTrigger.set(true);
  }

  // Returns true if a simulated release was triggered since last check (and resets the flag)
  public boolean consumeReleaseTrigger() {
    return simulatedReleaseTrigger.getAndSet(false);
  }

  public void setSelectedFace(ReefFace face) {
    this.selectedFace = face;
    publishSelection();
  }

  public void setSelectedSide(ApproachSide side) {
    this.selectedSide = side;
    publishSelection();
  }

  public void setSelectedLevel(CoralLevel level) {
    this.selectedLevel = level;
    publishSelection();
  }

  public void setTargetName(String name) {
    this.targetName = name == null ? "" : name;
    if (targetNamePub != null) targetNamePub.set(this.targetName);
    safeRecord("CoralRegistry/Target/Name", this.targetName);
  }

  public void requestSend() {
    this.sendRequested = true;
    safeRecord("CoralRegistry/SendRequested", true);
  }

  public boolean consumeSendRequest() {
    boolean v = sendRequested;
    sendRequested = false;
    return v;
  }

  public void markDone(ReefBranch branch) {
    setState(branch, BranchState.DONE);
  }

  public void markFree(ReefBranch branch) {
    setState(branch, BranchState.FREE);
  }

  public void markClaimed(ReefBranch branch) {
    setState(branch, BranchState.CLAIMED);
  }

  public BranchState getState(ReefBranch branch) {
    lock.lock();
    try {
      return states.get(branch);
    } finally {
      lock.unlock();
    }
  }

  public Optional<ReefBranch> claimNextFree() {
    lock.lock();
    try {
      for (var entry : states.entrySet()) {
        if (entry.getValue() == BranchState.FREE) {
          entry.setValue(BranchState.CLAIMED);
          publish(entry.getKey());
          safeRecord("CoralRegistry/Claimed", entry.getKey().name());
          return Optional.of(entry.getKey());
        }
      }
      return Optional.empty();
    } finally {
      lock.unlock();
    }
  }

  public Optional<ReefBranch> getFirstClaimed() {
    lock.lock();
    try {
      for (var entry : states.entrySet()) {
        if (entry.getValue() == BranchState.CLAIMED) {
          return Optional.of(entry.getKey());
        }
      }
      return Optional.empty();
    } finally {
      lock.unlock();
    }
  }

  public List<ReefBranch> getAllByFace(ReefFace face) {
    List<ReefBranch> list = new ArrayList<>();
    for (var b : ReefBranch.values()) if (b.face == face) list.add(b);
    return list;
  }

  private String topicFor(ReefBranch b) {
    return "Branch/" + b.name();
  }

  private void setState(ReefBranch branch, BranchState newState) {
    lock.lock();
    try {
      states.put(branch, newState);
      publish(branch);
    } finally {
      lock.unlock();
    }
  }

  private void publish(ReefBranch b) {
    BranchState s = states.get(b);
    boolean done = s == BranchState.DONE;
    if (table != null) {
      BooleanPublisher pub = branchDonePubs.get(b);
      if (pub != null) pub.set(done);
    }
    // Also log as AdvantageKit outputs unless disabled via -Dnt.coral=false
    safeRecord("CoralRegistry/" + b.name(), s.name());
  }

  public String toJson() {
    StringBuilder sb = new StringBuilder();
    sb.append('{');
    sb.append("\"autonomousEnabled\":").append(autonomousEnabled.get());
    sb.append(",\"sourcePreference\":\"").append(sourcePreference.name()).append("\"");
    sb.append(",\"selection\":{");
    sb.append("\"face\":\"").append(selectedFace.name()).append("\"");
    sb.append(",\"side\":\"").append(selectedSide.name()).append("\"");
    sb.append(",\"level\":\"").append(selectedLevel.name()).append("\"}");
    sb.append(",\"targetName\":\"").append(targetName).append("\"");
    sb.append(",\"sendRequested\":").append(sendRequested);
    sb.append(",\"branches\":{");
    boolean first = true;
    for (var b : ReefBranch.values()) {
      if (!first) sb.append(',');
      first = false;
      sb.append('"').append(b.name()).append('"').append(':');
      sb.append('"').append(getState(b).name()).append('"');
    }
    sb.append("}}");
    return sb.toString();
  }

  // Avoid crashes on desktop when logger/HAL isn't initialized
  private static void safeRecord(String key, boolean value) {
    try {
      if (isCoralLoggerEnabled()) Logger.recordOutput(remapLogKey(key), value);
    } catch (Throwable t) {
      // ignore in desktop test
    }
  }

  private static void safeRecord(String key, String value) {
    try {
      if (isCoralLoggerEnabled()) Logger.recordOutput(remapLogKey(key), value);
    } catch (Throwable t) {
      // ignore in desktop test
    }
  }

  private void publishSelection() {
    try {
      safeRecord("CoralRegistry/Target/Face", selectedFace.name());
      safeRecord("CoralRegistry/Target/Side", selectedSide.name());
      safeRecord("CoralRegistry/Target/Level", selectedLevel.name());
      if (targetFacePub != null) targetFacePub.set(selectedFace.name());
      if (targetSidePub != null) targetSidePub.set(selectedSide.name());
      if (targetLevelPub != null) targetLevelPub.set(selectedLevel.name());
    } catch (Throwable t) {
      // ignore
    }
  }

  /**
   * Polls NetworkTables Control topics and applies any changes. Call periodically from robot code
   * (e.g., RobotContainer.periodic()). This enables an external UI to control autonomy without the
   * embedded HTTP server.
   */
  public void pollNetworkControl() {
    if (table == null) return;
    try {
      if (ctlAutonomousEnabledSub != null) {
        boolean v = ctlAutonomousEnabledSub.get();
        if (v != isAutonomousEnabled()) setAutonomousEnabled(v);
      }

      if (ctlTargetFaceSub != null) {
        String v = ctlTargetFaceSub.get();
        if (v != null && !v.isBlank()) {
          try {
            setSelectedFace(ReefFace.valueOf(v));
          } catch (IllegalArgumentException ignored) {
          }
        }
      }
      if (ctlTargetSideSub != null) {
        String v = ctlTargetSideSub.get();
        if (v != null && !v.isBlank()) {
          try {
            setSelectedSide(ApproachSide.valueOf(v));
          } catch (IllegalArgumentException ignored) {
          }
        }
      }
      if (ctlTargetLevelSub != null) {
        String v = ctlTargetLevelSub.get();
        if (v != null && !v.isBlank()) {
          try {
            setSelectedLevel(CoralLevel.valueOf(v));
          } catch (IllegalArgumentException ignored) {
          }
        }
      }
      if (ctlTargetNameSub != null) {
        String v = ctlTargetNameSub.get();
        if (v != null && !v.isBlank() && !v.equals(targetName)) {
          setTargetName(v);
        }
      }

      // One-shot triggers on rising edge
      if (ctlTriggerLoadSub != null) {
        boolean cur = ctlTriggerLoadSub.get();
        if (cur && !lastCtlTriggerLoad) triggerLoad();
        lastCtlTriggerLoad = cur;
      }
      if (ctlTriggerReleaseSub != null) {
        boolean cur = ctlTriggerReleaseSub.get();
        if (cur && !lastCtlTriggerRelease) triggerRelease();
        lastCtlTriggerRelease = cur;
      }
      if (ctlSendSub != null) {
        boolean cur = ctlSendSub.get();
        if (cur && !lastCtlSend) requestSend();
        lastCtlSend = cur;
      }
    } catch (Throwable ignored) {
    }
  }

  private static boolean isCoralLoggerEnabled() {
    try {
      String prop = System.getProperty("nt.coral", "");
      String env = System.getenv("NT_CORAL");
      String val = (prop != null && !prop.isEmpty()) ? prop : env;
      if (val == null || val.isEmpty()) return true; // default enabled
      return !val.equalsIgnoreCase("false") && !val.equals("0");
    } catch (Throwable ignored) {
      return true;
    }
  }

  private static String remapLogKey(String key) {
    // Move logger mirror out of the CoralRegistry tree so opening that
    // subtree in NT viewers doesn't pull in the AdvantageKit topics.
    if (key != null && key.startsWith("CoralRegistry/")) {
      return "Log/" + key; // e.g. CoralRegistry/X -> Log/CoralRegistry/X
    }
    return key;
  }
}
