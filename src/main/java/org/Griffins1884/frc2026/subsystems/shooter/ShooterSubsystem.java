package org.Griffins1884.frc2026.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.generic.rollers.GenericVelocityRollerSystem;

@Setter
@Getter
public class ShooterSubsystem extends GenericVelocityRollerSystem<ShooterSubsystem.ShooterGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ShooterGoal implements VelocityGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> 8), // Maximum forward velocity
    REVERSE(() -> -8); // Maximum reverse velocity

    private final DoubleSupplier velocitySupplier;

    @Override
    public DoubleSupplier getVelocitySupplier() {
      return velocitySupplier;
    }
  }

  @Setter private ShooterGoal goal = ShooterGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.1);

  public ShooterSubsystem(String name, ShooterIO io) {
    super(
        name,
        io,
        new VelocityRollerConfig(
            ShooterConstants.gains,
            ShooterConstants.VELOCITY_TOLERANCE,
            ShooterConstants.MAX_VOLTAGE));
  }
}
