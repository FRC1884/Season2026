package org.Griffins1884.frc2026.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class StartupValidationTest {
  @Test
  void collectsWarningsOnRealHardware() {
    List<String> warnings = StartupValidation.collectWarnings(true);

    assertFalse(warnings.isEmpty());
    assertTrue(
        warnings.stream()
            .anyMatch(message -> message.contains("GlobalConstants.MODE") && message.contains("SIM")));
    assertTrue(
        warnings.stream()
            .anyMatch(message -> message.contains("GlobalConstants.ROBOT") && message.contains("SIMBOT")));
    assertTrue(warnings.stream().anyMatch(message -> message.contains("CAN ID")));
  }

  @Test
  void noWarningsInSim() {
    assertTrue(StartupValidation.collectWarnings(false).isEmpty());
  }
}
