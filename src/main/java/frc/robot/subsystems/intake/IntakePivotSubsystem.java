package frc.robot.subsystems.intake;

import frc.robot.generic.arms.GenericPositionArmSystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class IntakePivotSubsystem
    extends GenericPositionArmSystem<IntakePivotSubsystem.IntakePivotGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum IntakePivotGoal implements PivotGoal {
    IDLING(() -> IntakePivotConstants.STOW_ANGLE_RAD),
    TESTING(new LoggedTunableNumber("IntakePivot/Test", 0.0));

    private final DoubleSupplier angleSupplier;

    @Override
    public DoubleSupplier getAngle() {
      return () -> angleSupplier.getAsDouble();
    }
  }

  private IntakePivotGoal goal = IntakePivotGoal.IDLING;

  public IntakePivotSubsystem(String name, IntakePivotIO io) {
    super(
        name,
        io,
        new ArmConfig(
            IntakePivotConstants.GAINS,
            IntakePivotConstants.POSITION_TOLERANCE,
            IntakePivotConstants.SOFT_LIMITS_ENABLED,
            IntakePivotConstants.SOFT_LIMIT_MIN,
            IntakePivotConstants.SOFT_LIMIT_MAX,
            IntakePivotConstants.MAX_VOLTAGE));
  }
}
