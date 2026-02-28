package org.Griffins1884.frc2026.subsystems.indexer;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.generic.rollers.GenericVelocityRollerSystem;
import org.Griffins1884.frc2026.generic.rollers.GenericVelocityRollerSystem.VelocityGoal;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

@Setter
@Getter
public class IndexerSubsystem extends GenericVelocityRollerSystem<IndexerSubsystem.IndexerGoal> {
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
        io,
        new VelocityRollerConfig(
            IndexerConstants.gains,
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
