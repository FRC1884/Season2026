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
        "OverBump", superstructure.setSuperStateCmd(Superstructure.SuperState.IDLING));

    NamedCommands.registerCommand(
        "OverBumpAndShoot",
        Commands.sequence(
            superstructure.setSuperStateCmd(Superstructure.SuperState.IDLING),
            superstructure.setSuperStateCmd(Superstructure.SuperState.SHOOTING)));

    NamedCommands.registerCommand(
        "Collect",
        Commands.sequence(
            superstructure.setSuperStateCmd(Superstructure.SuperState.INTAKING),
            AutoAlignToFuelCommand.alignToFuelCommand(drive)));

    NamedCommands.registerCommand(
        "Climb",
        Commands.sequence(
            superstructure.setSuperStateCmd(Superstructure.SuperState.IDLING),
            superstructure.setSuperStateCmd(Superstructure.SuperState.AUTO_CLIMB)));

    NamedCommands.registerCommand(
        "ShootAndClimb",
        Commands.sequence(
            superstructure.setSuperStateCmd(Superstructure.SuperState.SHOOTING),
            DriveCommands.alignToClimbCommand(drive),
            superstructure.setSuperStateCmd(Superstructure.SuperState.AUTO_CLIMB)));
  }
}
