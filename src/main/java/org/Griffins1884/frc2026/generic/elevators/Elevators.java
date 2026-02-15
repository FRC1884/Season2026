package org.Griffins1884.frc2026.generic.elevators;

import static org.Griffins1884.frc2026.Config.Subsystems.CLIMBER_ENABLED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.subsystems.climber.ClimberConstants;
import org.Griffins1884.frc2026.subsystems.climber.ClimberIOFlex;
import org.Griffins1884.frc2026.subsystems.climber.ClimberIOKraken;
import org.Griffins1884.frc2026.subsystems.climber.ClimberIOMax;
import org.Griffins1884.frc2026.subsystems.climber.ClimberIOSim;
import org.Griffins1884.frc2026.subsystems.climber.ClimberSubsystem;

public class Elevators extends SubsystemBase {
  public ClimberSubsystem climber =
      (CLIMBER_ENABLED)
          ? new ClimberSubsystem(
              "Climber",
              (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
                  ? new ClimberIOSim()
                  : switch (ClimberConstants.MOTOR_CONTROLLER) {
                    case SPARK_MAX -> new ClimberIOMax();
                    case SPARK_FLEX -> new ClimberIOFlex();
                    case KRAKEN_X60, KRAKEN_X40 -> new ClimberIOKraken();
                  })
          : null;

  @Override
  public void periodic() {}

  public Command climberOpenLoop(ClimberSubsystem climber, DoubleSupplier percentSupplier) {
    return Commands.runEnd(
        () -> climber.setOpenLoop(percentSupplier.getAsDouble()), climber::stopOpenLoop, climber);
  }
}
