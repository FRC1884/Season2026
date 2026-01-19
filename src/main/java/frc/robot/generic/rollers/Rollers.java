package frc.robot.generic.rollers;

import static frc.robot.Config.Subsystems.INDEXER_ENABLED;
import static frc.robot.Config.Subsystems.INTAKE_ENABLED;
import static frc.robot.Config.Subsystems.SHOOTER_ENABLED;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIOFlex;
import frc.robot.subsystems.indexer.IndexerIOMax;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOFlex;
import frc.robot.subsystems.intake.IntakeIOMax;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.*;

public class Rollers extends SubsystemBase {
  public IntakeSubsystem intake =
      (INTAKE_ENABLED)
          ? new IntakeSubsystem(
              "Intake",
              (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
                  ? new IntakeIOSim(DCMotor.getNeoVortex(2), 1, 1)
                  : (IntakeConstants.isFlex) ? new IntakeIOFlex() : new IntakeIOMax())
          : null;
  public ShooterSubsystem shooter =
      (SHOOTER_ENABLED)
          ? new ShooterSubsystem(
              "Shooter",
              (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
                  ? new ShooterIOSim(DCMotor.getNeoVortex(2), 1, 1)
                  : (ShooterConstants.isFlex) ? new ShooterIOFlex() : new ShooterIOMax())
          : null;
  public IndexerSubsystem indexer =
      (INDEXER_ENABLED)
          ? new IndexerSubsystem(
              "Indexer",
              (GlobalConstants.MODE == GlobalConstants.RobotMode.SIM)
                  ? new IndexerIOSim(DCMotor.getNeoVortex(2), 1, 1)
                  : (IndexerConstants.IS_FLEX) ? new IndexerIOFlex() : new IndexerIOMax())
          : null;

  @Override
  public void periodic() {}
}
