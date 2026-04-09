package org.Griffins1884.frc2026.simulation.replay;

import java.util.Objects;
import org.Griffins1884.frc2026.simulation.contracts.FrameBinaryCodec;

/** Compares deterministic replay traces and reports the first mismatch. */
public final class DeterministicReplayVerifier {
  private DeterministicReplayVerifier() {}

  public static VerificationResult compare(
      DeterministicReplayTrace expected, DeterministicReplayTrace actual) {
    Objects.requireNonNull(expected, "expected");
    Objects.requireNonNull(actual, "actual");

    String expectedHash = expected.sha256Hex();
    String actualHash = actual.sha256Hex();
    if (expectedHash.equals(actualHash)) {
      return new VerificationResult(true, expectedHash, actualHash, "identical");
    }

    if (expected.actuatorFrames().size() != actual.actuatorFrames().size()) {
      return new VerificationResult(
          false,
          expectedHash,
          actualHash,
          "Actuator frame count mismatch: "
              + expected.actuatorFrames().size()
              + " vs "
              + actual.actuatorFrames().size());
    }
    if (expected.sensorFrames().size() != actual.sensorFrames().size()) {
      return new VerificationResult(
          false,
          expectedHash,
          actualHash,
          "Sensor frame count mismatch: "
              + expected.sensorFrames().size()
              + " vs "
              + actual.sensorFrames().size());
    }
    if (expected.worldSnapshots().size() != actual.worldSnapshots().size()) {
      return new VerificationResult(
          false,
          expectedHash,
          actualHash,
          "World snapshot count mismatch: "
              + expected.worldSnapshots().size()
              + " vs "
              + actual.worldSnapshots().size());
    }

    for (int i = 0; i < expected.actuatorFrames().size(); i++) {
      if (!java.util.Arrays.equals(
          FrameBinaryCodec.encodeActuatorFrame(expected.actuatorFrames().get(i)),
          FrameBinaryCodec.encodeActuatorFrame(actual.actuatorFrames().get(i)))) {
        return new VerificationResult(
            false, expectedHash, actualHash, "Actuator frame mismatch at step " + i);
      }
    }
    for (int i = 0; i < expected.sensorFrames().size(); i++) {
      if (!java.util.Arrays.equals(
          FrameBinaryCodec.encodeSensorFrame(expected.sensorFrames().get(i)),
          FrameBinaryCodec.encodeSensorFrame(actual.sensorFrames().get(i)))) {
        return new VerificationResult(
            false, expectedHash, actualHash, "Sensor frame mismatch at step " + i);
      }
    }
    for (int i = 0; i < expected.worldSnapshots().size(); i++) {
      if (!java.util.Arrays.equals(
          FrameBinaryCodec.encodeWorldSnapshot(expected.worldSnapshots().get(i)),
          FrameBinaryCodec.encodeWorldSnapshot(actual.worldSnapshots().get(i)))) {
        return new VerificationResult(
            false, expectedHash, actualHash, "World snapshot mismatch at step " + i);
      }
    }

    return new VerificationResult(
        false, expectedHash, actualHash, "hash mismatch without isolated frame mismatch");
  }

  public record VerificationResult(
      boolean identical, String expectedHash, String actualHash, String detail) {}
}
