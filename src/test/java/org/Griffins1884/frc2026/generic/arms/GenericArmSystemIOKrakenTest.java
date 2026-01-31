package org.Griffins1884.frc2026.generic.arms;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.junit.jupiter.api.Test;

class GenericArmSystemIOKrakenTest {
  @Test
  void softLimitEnableFlagIsRespected() {
    TalonFXConfiguration disabledConfig =
        GenericArmSystemIOKraken.buildConfiguration(
            true, 40, false, 1.0, -1.0, 1.0, false);
    assertFalse(disabledConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable);
    assertFalse(disabledConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable);

    TalonFXConfiguration enabledConfig =
        GenericArmSystemIOKraken.buildConfiguration(
            true, 40, false, 1.0, -1.0, 1.0, true);
    assertTrue(enabledConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable);
    assertTrue(enabledConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable);
  }
}
