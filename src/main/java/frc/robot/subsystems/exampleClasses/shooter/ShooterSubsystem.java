package frc.robot.subsystems.exampleClasses.shooter;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.generic.rollers.GenericVelocityRollerSystem;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ShooterSubsystem extends GenericVelocityRollerSystem<ShooterSubsystem.ShooterGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ShooterGoal implements VelocityGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> 6), // Maximum forward velocity
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
