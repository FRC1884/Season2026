package org.Griffins1884.frc2026.subsystems.indexer;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.mechanisms.RobotMechanismDefinitions;
import org.Griffins1884.frc2026.mechanisms.rollers.VelocityRollerMechanism;
import org.Griffins1884.frc2026.mechanisms.rollers.VelocityRollerMechanism.VelocityGoal;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

@Setter
@Getter
public class IndexerSubsystem extends VelocityRollerMechanism<IndexerSubsystem.IndexerGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum IndexerGoal implements VelocityGoal {
    IDLING(() -> 0.0),
    FORWARD(() -> IndexerConstants.FORWARD_RPM.get()),
    REVERSE(() -> IndexerConstants.REVERSE_RPM.get()),
    TESTING(new LoggedTunableNumber("Indexer/Testing", 0.0));

    private final DoubleSupplier velocitySupplier;

    @Override
    public DoubleSupplier getVelocitySupplier() {
      return velocitySupplier;
    }
  }

  @Setter private IndexerGoal goal = IndexerGoal.IDLING;

  public IndexerSubsystem(String name, IndexerIO io) {
    super(
        name,
        RobotMechanismDefinitions.INDEXER,
        io,
        new VelocityRollerConfig(
            IndexerConstants.GAINS,
            IndexerConstants.VELOCITY_TOLERANCE,
            IndexerConstants.MAX_VOLTAGE));
  }

  @Override
  protected double getAdditionalCompensationVolts(
      double goalVelocityRpm, double measuredVelocityRpm) {
    double run = 0.6 - Math.max(inputs.appliedVoltage / IndexerConstants.MAX_VOLTAGE, 0.6);
    return IndexerConstants.MAX_VOLTAGE * run;
  }
}
