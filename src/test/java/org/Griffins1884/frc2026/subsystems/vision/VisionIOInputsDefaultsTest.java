package org.Griffins1884.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class VisionIOInputsDefaultsTest {
  @Test
  void limelightProfileDefaults_areInitialized() {
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    assertEquals(VisionIO.LimelightProfile.LL4.name(), inputs.limelightProfile);
    assertEquals("DEFAULT", inputs.limelightProfileSource);
  }
}
