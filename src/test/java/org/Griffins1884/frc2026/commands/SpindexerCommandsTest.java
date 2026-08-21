package org.Griffins1884.frc2026.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.Griffins1884.frc2026.subsystems.indexer.SpindexerIO;
import org.Griffins1884.frc2026.subsystems.indexer.SpindexerSubsystem;
import org.Griffins1884.frc2026.subsystems.indexer.SpindexerSubsystem.SpindexerGoal;
import org.junit.jupiter.api.Test;

class SpindexerCommandsTest {
  @Test
  void factoriesSelectIndexReverseAndStopGoals() {
    SpindexerSubsystem spindexer =
        new SpindexerSubsystem("SpindexerCommandTest", new SpindexerIO() {});

    SpindexerCommands.index(spindexer).initialize();
    assertEquals(SpindexerGoal.INDEXING, spindexer.getGoal());
    SpindexerCommands.reverse(spindexer).initialize();
    assertEquals(SpindexerGoal.REVERSE, spindexer.getGoal());
    SpindexerCommands.stop(spindexer).initialize();
    assertEquals(SpindexerGoal.IDLING, spindexer.getGoal());
  }
}
