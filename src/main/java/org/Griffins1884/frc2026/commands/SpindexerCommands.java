package org.Griffins1884.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.Griffins1884.frc2026.subsystems.indexer.SpindexerSubsystem;
import org.Griffins1884.frc2026.subsystems.indexer.SpindexerSubsystem.SpindexerGoal;

public final class SpindexerCommands {
  private SpindexerCommands() {}

  public static Command index(SpindexerSubsystem spindexer) {
    return Commands.runOnce(() -> spindexer.setGoal(SpindexerGoal.INDEXING), spindexer);
  }

  public static Command reverse(SpindexerSubsystem spindexer) {
    return Commands.runOnce(() -> spindexer.setGoal(SpindexerGoal.REVERSE), spindexer);
  }

  public static Command stop(SpindexerSubsystem spindexer) {
    return Commands.runOnce(() -> spindexer.setGoal(SpindexerGoal.IDLING), spindexer);
  }
}
