package org.Griffins1884.frc2026.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.generic.rollers.GenericVoltageRollerSystem;
import org.Griffins1884.frc2026.generic.rollers.GenericVoltageRollerSystem.VoltageGoal;
import org.Griffins1884.frc2026.generic.rollers.GenericVoltageRollerSystem.VoltageRollerConfig;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

@Setter
@Getter
public class IntakeSubsystem extends GenericVoltageRollerSystem<IntakeSubsystem.IntakeGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum IntakeGoal implements VoltageGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> IntakeConstants.FORWARD_RPM.get()), // Maximum forward velocity
    REVERSE(() -> IntakeConstants.REVERSE_RPM.get()), // Maximum reverse velocity
    TESTING(new LoggedTunableNumber("Intake/Testing", 0.0));

    private final DoubleSupplier voltageSupplier;

    @Override
    public DoubleSupplier getVoltageSupplier() {
      return voltageSupplier;
    }
  }

  @Setter private IntakeGoal goal = IntakeGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.1);

  public IntakeSubsystem(String name, IntakeIO io) {
    super(name, io, new VoltageRollerConfig(IntakeConstants.MAX_VOLTAGE));
  }
}
