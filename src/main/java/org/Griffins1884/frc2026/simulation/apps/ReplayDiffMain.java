package org.Griffins1884.frc2026.simulation.apps;

import java.io.PrintWriter;
import java.nio.file.Path;
import org.Griffins1884.frc2026.simulation.runtime.ReplayDiff;

/** Compares two deterministic replay logs. */
public final class ReplayDiffMain {
  private ReplayDiffMain() {}

  public static void main(String[] args) throws Exception {
    if (args.length != 2) {
      throw new IllegalArgumentException("usage: replay-diff <left-log> <right-log>");
    }
    ReplayDiff replayDiff = new ReplayDiff();
    var result = replayDiff.compare(Path.of(args[0]), Path.of(args[1]));
    PrintWriter writer = new PrintWriter(System.out, true);
    writer.println("expectedHash=" + result.expectedHash());
    writer.println("actualHash=" + result.actualHash());
    writer.println("detail=" + result.detail());
    if (!result.identical()) {
      throw new IllegalStateException("Replay logs differ: " + result.detail());
    }
  }
}
