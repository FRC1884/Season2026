package org.Griffins1884.frc2026.simulation.replay;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class DeterministicReplayVerifierTest {
  @Test
  void identicalRunsProduceMatchingHashesAndNoDiff() {
    DeterministicShotSimulationRunner runner = new DeterministicShotSimulationRunner();
    ShotReplayScenario scenario = ShotReplayScenario.defaultScenario();

    DeterministicReplayTrace first = runner.run(scenario);
    DeterministicReplayTrace second = runner.run(scenario);
    DeterministicReplayVerifier.VerificationResult result =
        DeterministicReplayVerifier.compare(first, second);

    assertTrue(result.identical());
    assertEquals(first.sha256Hex(), second.sha256Hex());
  }
}
