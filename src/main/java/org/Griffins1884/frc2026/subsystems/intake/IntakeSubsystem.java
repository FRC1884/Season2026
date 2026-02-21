package org.Griffins1884.frc2026.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.generic.rollers.GenericVelocityRollerSystem;
import org.Griffins1884.frc2026.generic.rollers.GenericVelocityRollerSystem.VelocityGoal;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

@Setter
@Getter
public class IntakeSubsystem extends GenericVelocityRollerSystem<IntakeSubsystem.IntakeGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum IntakeGoal implements VelocityGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> IntakeConstants.FORWARD_RPM.get()), // Maximum forward velocity
    REVERSE(() -> IntakeConstants.REVERSE_RPM.get()), // Maximum reverse velocity
    TESTING(new LoggedTunableNumber("Intake/Testing", 0.0));

    private final DoubleSupplier velocitySupplier;

    @Override
    public DoubleSupplier getVelocitySupplier() {
      return velocitySupplier;
    }
  }

  @Setter private IntakeGoal goal = IntakeGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.1);

  public IntakeSubsystem(String name, IntakeIO io) {
    super(
        name,
        io,
        new VelocityRollerConfig(
            IntakeConstants.gains,
            IntakeConstants.VELOCITY_TOLERANCE,
            IntakeConstants.MAX_VOLTAGE));
  }
}
