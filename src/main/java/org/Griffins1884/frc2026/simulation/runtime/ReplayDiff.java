package org.Griffins1884.frc2026.simulation.runtime;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import org.Griffins1884.frc2026.simulation.replay.DeterministicReplayTrace;
import org.Griffins1884.frc2026.simulation.replay.DeterministicReplayVerifier;

/** Diff helper for deterministic replay files. */
public final class ReplayDiff {
  private final ReplayLogReader reader = new ReplayLogReader();

  public DeterministicReplayVerifier.VerificationResult compare(Path left, Path right)
      throws IOException {
    try (InputStream leftStream = Files.newInputStream(left);
        InputStream rightStream = Files.newInputStream(right)) {
      DeterministicReplayTrace leftTrace = reader.read(leftStream);
      DeterministicReplayTrace rightTrace = reader.read(rightStream);
      return DeterministicReplayVerifier.compare(leftTrace, rightTrace);
    }
  }
}
