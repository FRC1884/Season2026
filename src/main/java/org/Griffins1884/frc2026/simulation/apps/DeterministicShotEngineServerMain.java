package org.Griffins1884.frc2026.simulation.apps;

import java.io.PrintWriter;
import org.Griffins1884.frc2026.simulation.transport.DeterministicShotEngineServer;

/** Starts the deterministic socket-backed engine server. */
public final class DeterministicShotEngineServerMain {
  private DeterministicShotEngineServerMain() {}

  public static void main(String[] args) throws Exception {
    int port = args.length > 0 ? Integer.parseInt(args[0]) : 5815;
    try (DeterministicShotEngineServer server = new DeterministicShotEngineServer(port)) {
      new PrintWriter(System.out, true).println("griffinsim-engine-port=" + server.port());
      server.serveOneClient();
    }
  }
}
