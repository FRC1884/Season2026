package org.Griffins1884.frc2026.simulation.replay;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.Griffins1884.frc2026.simulation.contracts.WheelContactState;
import org.junit.jupiter.api.Test;

class ContactRichPhysicsReplayTest {
  @Test
  void robotTerrainAndGamepieceScenarioIsDeterministic() {
    DeterministicPhysicsScenarioRunner runner = new DeterministicPhysicsScenarioRunner();
    DeterministicReplayTrace first = runner.runDefaultScenario();
    DeterministicReplayTrace second = runner.runDefaultScenario();

    assertEquals(first.sha256Hex(), second.sha256Hex());
    assertTrue(
        first.worldSnapshots().stream()
            .anyMatch(snapshot -> !snapshot.contactPointStates().isEmpty()));
    assertTrue(
        first.worldSnapshots().stream()
            .anyMatch(
                snapshot ->
                    snapshot.wheelContactStates().stream()
                        .anyMatch(WheelContactState::contacting)));
  }
}
