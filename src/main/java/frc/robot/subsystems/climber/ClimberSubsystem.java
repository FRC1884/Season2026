package frc.robot.subsystems.climber;

import frc.robot.generic.elevators.GenericPositionElevatorSystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ClimberSubsystem extends GenericPositionElevatorSystem<ClimberSubsystem.ClimberGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ClimberGoal implements ExtensionGoal {
    IDLING(() -> ClimberConstants.STOW_HEIGHT_METERS),
    TESTING(new LoggedTunableNumber("Climber/Test", 0.0));

    private final DoubleSupplier heightSupplier;

    @Override
    public DoubleSupplier getHeight() {
      return () -> heightSupplier.getAsDouble();
    }
  }

  private ClimberGoal goal = ClimberGoal.IDLING;

  public ClimberSubsystem(String name, ClimberIO io) {
    super(
        name,
        io,
        new ElevatorConfig(
            ClimberConstants.GAINS,
            ClimberConstants.POSITION_TOLERANCE,
            ClimberConstants.SOFT_LIMITS_ENABLED,
            ClimberConstants.SOFT_LIMIT_MIN,
            ClimberConstants.SOFT_LIMIT_MAX,
            ClimberConstants.MAX_VOLTAGE));
  }

  public void setManualVoltage(double volts) {
    setOpenLoop(volts / ClimberConstants.MAX_VOLTAGE);
  }

  public double getPositionMeters() {
    return getPosition();
  }
}
