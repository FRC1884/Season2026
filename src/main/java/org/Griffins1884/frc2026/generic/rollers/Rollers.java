package org.Griffins1884.frc2026.generic.rollers;

import static org.Griffins1884.frc2026.Config.Subsystems.INDEXER_ENABLED;
import static org.Griffins1884.frc2026.Config.Subsystems.INTAKE_ENABLED;
import static org.Griffins1884.frc2026.Config.Subsystems.SHOOTER_ENABLED;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerConstants;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerIOFlex;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerIOMax;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerIOSim;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerSubsystem;
import org.Griffins1884.frc2026.subsystems.intake.IntakeConstants;
import org.Griffins1884.frc2026.subsystems.intake.IntakeIOFlex;
import org.Griffins1884.frc2026.subsystems.intake.IntakeIOMax;
import org.Griffins1884.frc2026.subsystems.intake.IntakeIOSim;
import org.Griffins1884.frc2026.subsystems.intake.IntakeSubsystem;
import org.Griffins1884.frc2026.subsystems.shooter.*;

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
                  : switch (ShooterConstants.MOTOR_CONTROLLER) {
                    case SPARK_FLEX -> new ShooterIOFlex();
                    case SPARK_MAX -> new ShooterIOMax();
                    case KRAKEN_X60, KRAKEN_X40 -> new ShooterIOKraken();
                  })
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
