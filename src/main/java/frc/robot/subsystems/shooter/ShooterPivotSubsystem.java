package frc.robot.subsystems.shooter;

import frc.robot.generic.arms.GenericPositionArmSystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ShooterPivotSubsystem
    extends GenericPositionArmSystem<ShooterPivotSubsystem.ShooterPivotGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ShooterPivotGoal implements PivotGoal {
    IDLING(() -> ShooterPivotConstants.IDLE_ANGLE_RAD),
    TESTING(new LoggedTunableNumber("ShooterPivot/Test", 0.0));

    private final DoubleSupplier angleSupplier;

    @Override
    public DoubleSupplier getAngle() {
      return () -> angleSupplier.getAsDouble();
    }
  }

  private ShooterPivotGoal goal = ShooterPivotGoal.IDLING;

  public ShooterPivotSubsystem(String name, ShooterPivotIO io) {
    super(
        name,
        io,
        new ArmConfig(
            ShooterPivotConstants.GAINS,
            ShooterPivotConstants.POSITION_TOLERANCE,
            ShooterPivotConstants.SOFT_LIMITS_ENABLED,
            ShooterPivotConstants.SOFT_LIMIT_MIN,
            ShooterPivotConstants.SOFT_LIMIT_MAX,
            ShooterPivotConstants.MAX_VOLTAGE));
  }
}
