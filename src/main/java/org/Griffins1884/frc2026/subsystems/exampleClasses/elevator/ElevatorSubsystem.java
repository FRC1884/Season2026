package org.Griffins1884.frc2026.subsystems.exampleClasses.elevator;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.generic.elevators.GenericPositionElevatorSystem;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

@Setter
@Getter
public class ElevatorSubsystem
    extends GenericPositionElevatorSystem<ElevatorSubsystem.ElevatorGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ElevatorGoal implements ExtensionGoal {
    IDLING(() -> 1),
    MIDPOINT(() -> 6),
    TESTING(new LoggedTunableNumber("Elevator/Test", 0.0));

    private final DoubleSupplier heightSupplier;

    @Override
    public DoubleSupplier getHeight() {
      return () -> heightSupplier.getAsDouble();
    }
  }

  private ElevatorGoal goal = ElevatorGoal.IDLING;

  public ElevatorSubsystem(String name, ElevatorIO io) {
    super(
        name,
        io,
        new ElevatorConfig(
            ElevatorConstants.GAINS,
            ElevatorConstants.POSITION_TOLERANCE,
            ElevatorConstants.SOFT_LIMITS_ENABLED,
            ElevatorConstants.SOFT_LIMIT_MIN,
            ElevatorConstants.SOFT_LIMIT_MAX,
            ElevatorConstants.MAX_VOLTAGE));
  }
}
