package frc.robot.subsystems.indexer;

import frc.robot.generic.rollers.GenericVoltageRollerSystem;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class IndexerSubsystem extends GenericVoltageRollerSystem<IndexerSubsystem.IndexerGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum IndexerGoal implements VoltageGoal {
    IDLING(() -> 0.0),
    FORWARD(() -> 6.0),
    REVERSE(() -> -6.0);

    private final DoubleSupplier voltageSupplier;
  }

  @Setter private IndexerGoal goal = IndexerGoal.IDLING;

  public IndexerSubsystem(String name, IndexerIO io) {
    super(name, io, new VoltageRollerConfig(IndexerConstants.MAX_VOLTAGE));
  }
}
