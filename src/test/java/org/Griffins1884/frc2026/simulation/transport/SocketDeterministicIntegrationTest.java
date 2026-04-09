package org.Griffins1884.frc2026.simulation.transport;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.Griffins1884.frc2026.simulation.control.SocketLockstepRunner;
import org.Griffins1884.frc2026.simulation.replay.ShotReplayScenario;
import org.junit.jupiter.api.Test;

class SocketDeterministicIntegrationTest {
  @Test
  void socketBackedEngineProducesStableReplayBytesAcrossRuns() throws Exception {
    ShotReplayScenario scenario = ShotReplayScenario.defaultScenario();

    SocketLockstepRunner.Result first = runOnce(scenario);
    SocketLockstepRunner.Result second = runOnce(scenario);

    assertEquals(first.trace().sha256Hex(), second.trace().sha256Hex());
    assertArrayEquals(first.replayBytes(), second.replayBytes());
  }

  private static SocketLockstepRunner.Result runOnce(ShotReplayScenario scenario) throws Exception {
    try (DeterministicShotEngineServer server = new DeterministicShotEngineServer(0)) {
      CountDownLatch serverDone = new CountDownLatch(1);
      Thread serverThread =
          new Thread(
              () -> {
                try {
                  server.serveOneClient();
                } catch (Exception ex) {
                  throw new RuntimeException(ex);
                } finally {
                  serverDone.countDown();
                }
              },
              "deterministic-shot-engine-test");
      serverThread.start();

      SocketLockstepRunner.Result result =
          new SocketLockstepRunner().run("127.0.0.1", server.port(), scenario);

      serverDone.await(2, TimeUnit.SECONDS);
      server.close();
      serverThread.join(2000);
      return result;
    }
  }
}
