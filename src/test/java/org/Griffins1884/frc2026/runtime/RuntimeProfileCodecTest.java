package org.Griffins1884.frc2026.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.EnumSet;
import java.util.Set;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.mechanisms.MechanismTelemetry;
import org.junit.jupiter.api.Test;

class RuntimeProfileCodecTest {
  @Test
  void fromJson_normalizesNamesAndSignals() throws Exception {
    RuntimeModeProfile profile =
        RuntimeProfileCodec.fromJson(
            """
            {
              "loggingMode": " debug ",
              "tuningEnabled": true,
              "debugSubsystems": [" Drive ", "VISION"],
              "loggedSignals": [" voltage ", "position"],
              "publishedSignals": [" target "]
            }
            """);

    assertEquals(GlobalConstants.LoggingMode.DEBUG, profile.loggingMode());
    assertTrue(profile.tuningEnabled());
    assertEquals(Set.of("drive", "vision"), profile.debugSubsystems());
    assertEquals(
        EnumSet.of(MechanismTelemetry.Signal.VOLTAGE, MechanismTelemetry.Signal.POSITION),
        profile.loggedSignals());
    assertEquals(Set.of(MechanismTelemetry.Signal.TARGET), profile.publishedSignals());
  }

  @Test
  void fromJson_usesCompetitionDefaultsWhenOptionalFieldsAreOmitted() throws Exception {
    RuntimeModeProfile profile =
        RuntimeProfileCodec.fromJson(
            """
            {
              "tuningEnabled": false
            }
            """);

    Set<MechanismTelemetry.Signal> expectedSignals =
        EnumSet.of(
            MechanismTelemetry.Signal.IDENTITY,
            MechanismTelemetry.Signal.CONNECTION,
            MechanismTelemetry.Signal.FAULTS,
            MechanismTelemetry.Signal.HEALTH,
            MechanismTelemetry.Signal.TARGET);
    assertEquals(GlobalConstants.LoggingMode.COMP, profile.loggingMode());
    assertEquals(expectedSignals, profile.loggedSignals());
    assertEquals(expectedSignals, profile.publishedSignals());
  }

  @Test
  void toJson_roundTripPreservesRuntimeProfile() throws Exception {
    RuntimeModeProfile original =
        new RuntimeModeProfile(
            GlobalConstants.LoggingMode.COMP,
            true,
            Set.of("swerve", "vision"),
            EnumSet.of(MechanismTelemetry.Signal.VOLTAGE, MechanismTelemetry.Signal.CURRENT),
            EnumSet.of(MechanismTelemetry.Signal.TARGET));

    RuntimeModeProfile decoded = RuntimeProfileCodec.fromJson(RuntimeProfileCodec.toJson(original));

    assertEquals(original, decoded);
  }
}
