package org.Griffins1884.frc2026.runtime;

import java.util.Collections;
import java.util.EnumSet;
import java.util.HashSet;
import java.util.Set;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.mechanisms.MechanismTelemetry;

/** Runtime logging/tuning profile shared by mechanisms and future dashboard config UI. */
public record RuntimeModeProfile(
    GlobalConstants.LoggingMode loggingMode,
    boolean tuningEnabled,
    Set<String> debugSubsystems,
    Set<MechanismTelemetry.Signal> loggedSignals,
    Set<MechanismTelemetry.Signal> publishedSignals) {

  public RuntimeModeProfile {
    loggingMode = loggingMode != null ? loggingMode : GlobalConstants.LoggingMode.COMP;
    debugSubsystems = normalizeSubsystems(debugSubsystems);
    loggedSignals = normalizeSignals(loggedSignals, loggingMode);
    publishedSignals = normalizeSignals(publishedSignals, loggingMode);
  }

  public static RuntimeModeProfile fromGlobals() {
    return new RuntimeModeProfile(
        GlobalConstants.LOGGING_MODE,
        GlobalConstants.TUNING_MODE,
        Collections.emptySet(),
        null,
        null);
  }

  public boolean isDebugMode() {
    return loggingMode == GlobalConstants.LoggingMode.DEBUG;
  }

  public boolean isDebugEnabled(String subsystemKey) {
    if (isDebugMode()) {
      return true;
    }
    if (subsystemKey == null || subsystemKey.isBlank()) {
      return false;
    }
    return debugSubsystems.contains(subsystemKey.trim().toLowerCase());
  }

  public boolean allowsTuning(boolean allowInCompMode) {
    return tuningEnabled || (allowInCompMode && loggingMode == GlobalConstants.LoggingMode.COMP);
  }

  public boolean shouldLog(MechanismTelemetry.Signal signal, String subsystemKey) {
    return loggedSignals.contains(signal)
        || (isDebugEnabled(subsystemKey) && debugSignalSet().contains(signal));
  }

  public boolean shouldPublish(MechanismTelemetry.Signal signal, String subsystemKey) {
    return publishedSignals.contains(signal)
        || (isDebugEnabled(subsystemKey) && debugSignalSet().contains(signal));
  }

  private static Set<String> normalizeSubsystems(Set<String> subsystems) {
    if (subsystems == null || subsystems.isEmpty()) {
      return Collections.emptySet();
    }
    Set<String> normalized = new HashSet<>();
    for (String subsystem : subsystems) {
      if (subsystem != null && !subsystem.isBlank()) {
        normalized.add(subsystem.trim().toLowerCase());
      }
    }
    return Collections.unmodifiableSet(normalized);
  }

  private static Set<MechanismTelemetry.Signal> normalizeSignals(
      Set<MechanismTelemetry.Signal> signals, GlobalConstants.LoggingMode loggingMode) {
    if (signals == null || signals.isEmpty()) {
      return loggingMode == GlobalConstants.LoggingMode.DEBUG
          ? Collections.unmodifiableSet(debugSignalSet())
          : Collections.unmodifiableSet(compSignalSet());
    }
    return Collections.unmodifiableSet(EnumSet.copyOf(signals));
  }

  private static EnumSet<MechanismTelemetry.Signal> compSignalSet() {
    return EnumSet.of(
        MechanismTelemetry.Signal.IDENTITY,
        MechanismTelemetry.Signal.CONNECTION,
        MechanismTelemetry.Signal.FAULTS,
        MechanismTelemetry.Signal.HEALTH,
        MechanismTelemetry.Signal.TARGET);
  }

  private static EnumSet<MechanismTelemetry.Signal> debugSignalSet() {
    return EnumSet.allOf(MechanismTelemetry.Signal.class);
  }
}
