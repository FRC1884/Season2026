package org.Griffins1884.frc2026.simulation.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import org.Griffins1884.frc2026.simulation.replay.DeterministicReplayTrace;
import org.Griffins1884.frc2026.simulation.replay.DeterministicShotSimulationRunner;
import org.Griffins1884.frc2026.simulation.replay.ShotReplayScenario;
import org.junit.jupiter.api.Test;

class ReplayLogRoundTripTest {
  @Test
  void replayWriterAndReaderPreserveDeterministicTrace() throws Exception {
    DeterministicReplayTrace original =
        new DeterministicShotSimulationRunner().run(ShotReplayScenario.defaultScenario());

    ByteArrayOutputStream output = new ByteArrayOutputStream();
    try (ReplayLogWriter writer = new ReplayLogWriter(output)) {
      for (var frame : original.actuatorFrames()) {
        writer.appendActuatorFrame(frame);
      }
      for (var frame : original.sensorFrames()) {
        writer.appendSensorFrame(frame);
      }
      for (var snapshot : original.worldSnapshots()) {
        writer.appendWorldSnapshot(snapshot);
      }
    }

    DeterministicReplayTrace decoded =
        new ReplayLogReader().read(new ByteArrayInputStream(output.toByteArray()));

    assertEquals(original.actuatorFrames().size(), decoded.actuatorFrames().size());
    assertEquals(original.sensorFrames().size(), decoded.sensorFrames().size());
    assertEquals(original.worldSnapshots().size(), decoded.worldSnapshots().size());
    assertNotNull(decoded.worldSnapshots().get(0).releasePose());
    assertEquals(
        original.worldSnapshots().get(0).releasePose().getX(),
        decoded.worldSnapshots().get(0).releasePose().getX(),
        1e-12);
    assertEquals(
        original.worldSnapshots().get(0).releasePose().getRotation().getQuaternion().getW(),
        decoded.worldSnapshots().get(0).releasePose().getRotation().getQuaternion().getW(),
        1e-12);
  }
}
