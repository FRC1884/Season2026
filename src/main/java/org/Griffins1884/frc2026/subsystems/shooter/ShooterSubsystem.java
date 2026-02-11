package org.Griffins1884.frc2026.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.generic.rollers.GenericVoltageRollerSystem;

@Setter
@Getter
public class ShooterSubsystem extends GenericVoltageRollerSystem<ShooterSubsystem.ShooterGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ShooterGoal implements VoltageGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> 12), // Maximum forward velocity
    REVERSE(() -> -12); // Maximum reverse velocity

    private final DoubleSupplier velocitySupplier;

    @Override
    public DoubleSupplier getVoltageSupplier() {
      return velocitySupplier;
    }
  }

  @Setter private ShooterGoal goal = ShooterGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.1);

  public ShooterSubsystem(String name, ShooterIO io) {
    super(name, io, new VoltageRollerConfig(ShooterConstants.MAX_VOLTAGE));
  }
}
