package frc.robot.generic.elevators;

import static frc.robot.Config.Subsystems.CLIMBER_ENABLED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIOFlex;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.climber.ClimberIOMax;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class Elevators extends SubsystemBase {
  public ClimberSubsystem climber =
      (CLIMBER_ENABLED)
          ? new ClimberSubsystem(
              "Climber",
              (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
                  ? new ClimberIOSim()
                  : switch (ClimberConstants.MOTOR_CONTROLLER) {
                    case SPARK_MAX -> new ClimberIOMax();
                    case SPARK_FLEX, VORTEX -> new ClimberIOFlex();
                    case KRAKEN_X60, KRAKEN_X40 -> new ClimberIOKraken();
                  })
          : null;

  @Override
  public void periodic() {}
}
