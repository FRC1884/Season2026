package org.Griffins1884.frc2026.runtime;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.util.Collections;
import java.util.EnumSet;
import java.util.LinkedHashSet;
import java.util.Set;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.mechanisms.MechanismTelemetry;

/** JSON codec for runtime profile exchange with the dashboard config page. */
public final class RuntimeProfileCodec {
  private static final ObjectMapper mapper = new ObjectMapper();

  private RuntimeProfileCodec() {}

  public static RuntimeModeProfile fromJson(String json) throws JsonProcessingException {
    RuntimeProfileDto dto = mapper.readValue(json, RuntimeProfileDto.class);
    GlobalConstants.LoggingMode loggingMode =
        dto.loggingMode != null
            ? GlobalConstants.LoggingMode.valueOf(dto.loggingMode.trim().toUpperCase())
            : GlobalConstants.LoggingMode.COMP;
    return new RuntimeModeProfile(
        loggingMode,
        dto.tuningEnabled,
        dto.debugSubsystems != null ? dto.debugSubsystems : Collections.emptySet(),
        parseSignals(dto.loggedSignals),
        parseSignals(dto.publishedSignals));
  }

  public static String toJson(RuntimeModeProfile profile) {
    RuntimeProfileDto dto = new RuntimeProfileDto();
    dto.loggingMode = profile.loggingMode().name();
    dto.tuningEnabled = profile.tuningEnabled();
    dto.debugSubsystems = new LinkedHashSet<>(profile.debugSubsystems());
    dto.loggedSignals = stringifySignals(profile.loggedSignals());
    dto.publishedSignals = stringifySignals(profile.publishedSignals());
    try {
      return mapper.writeValueAsString(dto);
    } catch (JsonProcessingException ex) {
      throw new IllegalStateException("Failed to serialize runtime profile", ex);
    }
  }

  private static Set<MechanismTelemetry.Signal> parseSignals(Set<String> names) {
    if (names == null || names.isEmpty()) {
      return Collections.emptySet();
    }
    EnumSet<MechanismTelemetry.Signal> signals = EnumSet.noneOf(MechanismTelemetry.Signal.class);
    for (String name : names) {
      if (name == null || name.isBlank()) {
        continue;
      }
      signals.add(MechanismTelemetry.Signal.valueOf(name.trim().toUpperCase()));
    }
    return signals;
  }

  private static Set<String> stringifySignals(Set<MechanismTelemetry.Signal> signals) {
    LinkedHashSet<String> values = new LinkedHashSet<>();
    for (MechanismTelemetry.Signal signal : signals) {
      values.add(signal.name());
    }
    return values;
  }

  private static final class RuntimeProfileDto {
    public String loggingMode;
    public boolean tuningEnabled;
    public Set<String> debugSubsystems = Collections.emptySet();
    public Set<String> loggedSignals = Collections.emptySet();
    public Set<String> publishedSignals = Collections.emptySet();
  }
}
