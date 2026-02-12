package org.Griffins1884.frc2026.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.generic.rollers.GenericVoltageRollerSystem;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

@Setter
@Getter
public class IntakeSubsystem extends GenericVoltageRollerSystem<IntakeSubsystem.IntakeGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum IntakeGoal implements VoltageGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> 12), // Maximum forward velocity
    REVERSE(() -> -12), // Maximum reverse velocity
    TESTING(new LoggedTunableNumber("Intake/Testing", 0.0));

    private final DoubleSupplier velocitySupplier;

    @Override
    public DoubleSupplier getVoltageSupplier() {
      return velocitySupplier;
    }
  }

  @Setter private IntakeGoal goal = IntakeGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.1);

  public IntakeSubsystem(String name, IntakeIO io) {
    super(name, io, new VoltageRollerConfig(IntakeConstants.MAX_VOLTAGE));
  }
}
