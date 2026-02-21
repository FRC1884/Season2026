package org.Griffins1884.frc2026.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;

public class AutoCommands {
  public static void registerAutoCommands(Superstructure superstructure, SwerveSubsystem drive) {
    NamedCommands.registerCommand(
        "Shoot", superstructure.setSuperStateCmd(Superstructure.SuperState.SHOOTING));

    NamedCommands.registerCommand(
        "OverSecondBump",
        Commands.sequence(
            superstructure.setSuperStateCmd(Superstructure.SuperState.IDLING),
            DriveCommands.alignToAfterSecondBumpCommand(drive)));

    NamedCommands.registerCommand(
        "FirstOverBump",
        Commands.sequence(
            superstructure.setSuperStateCmd(Superstructure.SuperState.IDLING),
            DriveCommands.alignToAfterBumpStartCommand(drive)));

    NamedCommands.registerCommand(
        "OverBumpAndShoot",
        Commands.sequence(
            superstructure.setSuperStateCmd(Superstructure.SuperState.IDLING),
            DriveCommands.alignToAfterSecondBumpCommand(drive),
            superstructure.setSuperStateCmd(Superstructure.SuperState.SHOOTING)));

    NamedCommands.registerCommand(
        "BumpToNeutral",
        Commands.sequence(
            superstructure.setSuperStateCmd(Superstructure.SuperState.IDLING),
            DriveCommands.alignToAfterBumpToNeutralCommand(drive)));

    NamedCommands.registerCommand(
        "Collect",
        Commands.sequence(
            superstructure.setSuperStateCmd(Superstructure.SuperState.INTAKING),
            AutoAlignToFuelCommand.alignToFuelCommand(drive),
            DriveCommands.alignToAfterCollectStartCommand(drive)));

    NamedCommands.registerCommand(
        "DepotCollect", superstructure.setSuperStateCmd(Superstructure.SuperState.INTAKING));
  }
}
