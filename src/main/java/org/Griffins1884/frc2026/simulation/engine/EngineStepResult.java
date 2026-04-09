package org.Griffins1884.frc2026.simulation.engine;

import java.util.List;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;

/** Output of a single authoritative engine step. */
public record EngineStepResult(List<SensorFrame> sensorFrames, WorldSnapshot worldSnapshot) {
  public EngineStepResult {
    sensorFrames = sensorFrames != null ? List.copyOf(sensorFrames) : List.of();
  }
}
