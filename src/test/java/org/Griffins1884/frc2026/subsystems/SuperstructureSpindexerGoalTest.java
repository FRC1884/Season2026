package org.Griffins1884.frc2026.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.Griffins1884.frc2026.subsystems.Superstructure.SuperState;
import org.Griffins1884.frc2026.subsystems.indexer.SpindexerSubsystem.SpindexerGoal;
import org.junit.jupiter.api.Test;

class SuperstructureSpindexerGoalTest {
  @Test
  void mapsEveryStateToTheConfirmedGoal() {
    assertEquals(SpindexerGoal.IDLING, Superstructure.spindexerGoalForState(SuperState.IDLING));
    assertEquals(SpindexerGoal.INDEXING, Superstructure.spindexerGoalForState(SuperState.INTAKING));
    assertEquals(SpindexerGoal.INDEXING, Superstructure.spindexerGoalForState(SuperState.SHOOTING));
    assertEquals(
        SpindexerGoal.INDEXING, Superstructure.spindexerGoalForState(SuperState.SHOOT_INTAKE));
    assertEquals(SpindexerGoal.INDEXING, Superstructure.spindexerGoalForState(SuperState.FERRYING));
    assertEquals(SpindexerGoal.TESTING, Superstructure.spindexerGoalForState(SuperState.TESTING));
  }
}
