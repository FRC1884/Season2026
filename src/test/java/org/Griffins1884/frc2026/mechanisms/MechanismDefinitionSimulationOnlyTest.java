package org.Griffins1884.frc2026.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class MechanismDefinitionSimulationOnlyTest {
  @Test
  void existingSpindexerCatalogEntryIsClassifiedForSimulation() {
    MechanismDefinition definition = RobotMechanismDefinitions.SPINDEXER;

    assertEquals(
        MechanismDefinition.MotorControllerType.SIMULATION_ONLY,
        definition.config().motorGroup().controllerType());
    assertTrue(definition.simulation().enabled());
  }
}
