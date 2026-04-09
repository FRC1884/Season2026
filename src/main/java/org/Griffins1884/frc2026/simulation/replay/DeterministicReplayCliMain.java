package org.Griffins1884.frc2026.simulation.replay;

import org.Griffins1884.frc2026.simulation.contracts.SimulationProtocolVersion;

/** Command-line entry point for deterministic replay verification. */
public final class DeterministicReplayCliMain {
  private DeterministicReplayCliMain() {}

  public static void main(String[] args) {
    DeterministicShotSimulationRunner runner = new DeterministicShotSimulationRunner();
    ShotReplayScenario scenario = ShotReplayScenario.defaultScenario();

    DeterministicReplayTrace first = runner.run(scenario);
    DeterministicReplayTrace second = runner.run(scenario);
    DeterministicReplayVerifier.VerificationResult result =
        DeterministicReplayVerifier.compare(first, second);

    System.out.println("schema=" + SimulationProtocolVersion.SCHEMA_ID);
    System.out.println("expectedHash=" + result.expectedHash());
    System.out.println("actualHash=" + result.actualHash());
    System.out.println("detail=" + result.detail());

    if (!result.identical()) {
      throw new IllegalStateException(
          "Deterministic replay verification failed: " + result.detail());
    }
  }
}
