package org.Griffins1884.frc2026.runtime;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import org.Griffins1884.frc2026.mechanisms.MechanismTelemetry;

/**
 * Central runtime profile holder.
 *
 * <p>This is the mode/config seam that the future config dashboard will update instead of every
 * subsystem reading globals independently.
 */
public final class RuntimeModeManager {
  private static final AtomicReference<RuntimeModeProfile> activeProfile =
      new AtomicReference<>(RuntimeModeProfile.fromGlobals());

  private RuntimeModeManager() {}

  public static RuntimeModeProfile getActiveProfile() {
    return activeProfile.get();
  }

  public static void setActiveProfile(RuntimeModeProfile profile) {
    activeProfile.set(Objects.requireNonNull(profile, "profile"));
  }

  public static void resetToDefaults() {
    activeProfile.set(RuntimeModeProfile.fromGlobals());
  }

  public static boolean isDebugEnabled() {
    return getActiveProfile().isDebugMode();
  }

  public static boolean isDebugEnabled(String subsystemKey) {
    return getActiveProfile().isDebugEnabled(subsystemKey);
  }

  public static boolean allowsTuning(boolean allowInCompMode) {
    return getActiveProfile().allowsTuning(allowInCompMode);
  }

  public static boolean shouldLog(MechanismTelemetry.Signal signal, String subsystemKey) {
    return getActiveProfile().shouldLog(signal, subsystemKey);
  }

  public static boolean shouldPublish(MechanismTelemetry.Signal signal, String subsystemKey) {
    return getActiveProfile().shouldPublish(signal, subsystemKey);
  }
}
