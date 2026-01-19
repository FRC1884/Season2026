package org.Griffins1884.frc2026.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.generic.rollers.GenericVoltageRollerSystem;

@Setter
@Getter
public class IntakeSubsystem extends GenericVoltageRollerSystem<IntakeSubsystem.intakeGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum intakeGoal implements VoltageGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> 6), // Maximum forward voltage
    REVERSE(() -> -8); // Maximum reverse voltage

    private final DoubleSupplier voltageSupplier;
  }

  @Setter private intakeGoal goal = intakeGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.1);

  public IntakeSubsystem(String name, IntakeIO io) {
    super(name, io, new VoltageRollerConfig(IntakeConstants.MAX_VOLTAGE));
  }
}
