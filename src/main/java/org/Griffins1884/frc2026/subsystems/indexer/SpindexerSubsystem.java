package org.Griffins1884.frc2026.subsystems.indexer;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.mechanisms.RobotMechanismDefinitions;
import org.Griffins1884.frc2026.mechanisms.rollers.VelocityRollerMechanism;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

@Setter
@Getter
public class SpindexerSubsystem extends VelocityRollerMechanism<SpindexerSubsystem.SpindexerGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum SpindexerGoal implements VelocityGoal {
    IDLING(() -> 0.0),
    INDEXING(() -> SpindexerConstants.INDEX_RPM.get()),
    REVERSE(() -> SpindexerConstants.REVERSE_RPM.get()),
    TESTING(new LoggedTunableNumber("Spindexer/TestingRpm", 0.0));

    private final DoubleSupplier velocitySupplier;

    @Override
    public DoubleSupplier getVelocitySupplier() {
      return velocitySupplier;
    }
  }

  private SpindexerGoal goal = SpindexerGoal.IDLING;

  public SpindexerSubsystem(String name, SpindexerIO io) {
    super(
        name,
        RobotMechanismDefinitions.SPINDEXER,
        io,
        new VelocityRollerConfig(
            SpindexerConstants.GAINS,
            SpindexerConstants.VELOCITY_TOLERANCE,
            SpindexerConstants.MAX_VOLTAGE));
  }
}
