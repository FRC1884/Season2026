package org.Griffins1884.frc2026.simulation.replay;

import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.HexFormat;
import java.util.List;
import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;
import org.Griffins1884.frc2026.simulation.contracts.FrameBinaryCodec;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;

/** Immutable deterministic replay trace with canonical hashing support. */
public record DeterministicReplayTrace(
    List<ActuatorFrame> actuatorFrames,
    List<SensorFrame> sensorFrames,
    List<WorldSnapshot> worldSnapshots) {
  public DeterministicReplayTrace {
    actuatorFrames = actuatorFrames != null ? List.copyOf(actuatorFrames) : List.of();
    sensorFrames = sensorFrames != null ? List.copyOf(sensorFrames) : List.of();
    worldSnapshots = worldSnapshots != null ? List.copyOf(worldSnapshots) : List.of();
  }

  public String sha256Hex() {
    try {
      MessageDigest digest = MessageDigest.getInstance("SHA-256");
      for (ActuatorFrame frame : actuatorFrames) {
        digest.update(FrameBinaryCodec.encodeActuatorFrame(frame));
      }
      for (SensorFrame frame : sensorFrames) {
        digest.update(FrameBinaryCodec.encodeSensorFrame(frame));
      }
      for (WorldSnapshot snapshot : worldSnapshots) {
        digest.update(FrameBinaryCodec.encodeWorldSnapshot(snapshot));
      }
      return HexFormat.of().formatHex(digest.digest());
    } catch (NoSuchAlgorithmException ex) {
      throw new IllegalStateException("Missing SHA-256 MessageDigest", ex);
    }
  }
}
