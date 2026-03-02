package org.Griffins1884.frc2026.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;

public class AutoCommands {
  public static void registerAutoCommands(Superstructure superstructure, SwerveSubsystem drive) {
    NamedCommands.registerCommand(
        "Shoot",
        Commands.sequence(superstructure.setSuperStateCmd(Superstructure.SuperState.SHOOTING)));

    NamedCommands.registerCommand(
        "GetReady", superstructure.setSuperStateCmd(Superstructure.SuperState.SHOOTING));

    NamedCommands.registerCommand(
        "ShootBalls", superstructure.setSuperStateCmd(Superstructure.SuperState.SHOOTING));

    NamedCommands.registerCommand(
        "OverSecondBump",
        Commands.sequence(superstructure.setSuperStateCmd(Superstructure.SuperState.IDLING)));

    NamedCommands.registerCommand(
        "Intake", superstructure.setSuperStateCmd(Superstructure.SuperState.INTAKING));

    NamedCommands.registerCommand(
        "DepotCollect", superstructure.setSuperStateCmd(Superstructure.SuperState.INTAKING));

    NamedCommands.registerCommand(
        "StandStillShoot", DriveCommands.alignToStandStillShootCommand(drive));
  }
}
