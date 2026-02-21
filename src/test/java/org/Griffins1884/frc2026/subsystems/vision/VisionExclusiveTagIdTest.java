package org.Griffins1884.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;

class VisionExclusiveTagIdTest {
  @Test
  void clearExclusiveTagId_resetsFilter() throws Exception {
    Vision vision = new Vision((pose, timestamp, stdDevs) -> {});
    vision.setExclusiveTagId(6);
    vision.clearExclusiveTagId();

    Field exclusiveTagIdField = Vision.class.getDeclaredField("exclusiveTagId");
    exclusiveTagIdField.setAccessible(true);
    assertNull(exclusiveTagIdField.get(vision));
  }

  @Test
  void setExclusiveTagId_setsRequestedValue() throws Exception {
    Vision vision = new Vision((pose, timestamp, stdDevs) -> {});
    vision.setExclusiveTagId(17);

    Field exclusiveTagIdField = Vision.class.getDeclaredField("exclusiveTagId");
    exclusiveTagIdField.setAccessible(true);
    assertEquals(17, exclusiveTagIdField.get(vision));
  }
}
