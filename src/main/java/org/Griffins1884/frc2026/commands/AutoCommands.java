package org.Griffins1884.frc2026.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import org.Griffins1884.frc2026.subsystems.Superstructure;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;

public class AutoCommands {
  private AutoCommands() {}

  public static void registerAutoCommands(Superstructure superstructure, SwerveSubsystem drive) {
    NamedCommands.registerCommand(
        "GetRShoot",
        Commands.sequence(superstructure.setSuperStateCmd(Superstructure.SuperState.SHOOTING)));

    NamedCommands.registerCommand("AlignToHP", DriveCommands.alignToHPd(drive));

    NamedCommands.registerCommand("AlignToDepot", DriveCommands.alignToDepot(drive));

    NamedCommands.registerCommand("Shoot", Commands.runOnce(superstructure::toggleShootEnabled));

    NamedCommands.registerCommand(
        "Idling",
        Commands.sequence(superstructure.setSuperStateCmd(Superstructure.SuperState.IDLING)));

    NamedCommands.registerCommand(
        "Intake", superstructure.setSuperStateCmd(Superstructure.SuperState.INTAKING));

    NamedCommands.registerCommand(
        "ShootIntake", superstructure.setSuperStateCmd(Superstructure.SuperState.SHOOT_INTAKE));
  }
}
