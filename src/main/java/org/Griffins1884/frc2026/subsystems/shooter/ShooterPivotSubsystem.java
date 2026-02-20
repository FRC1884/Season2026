package org.Griffins1884.frc2026.subsystems.shooter;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.generic.arms.GenericPositionArmSystem;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

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
            ShooterPivotConstants.GAINS.kP(),
            ShooterPivotConstants.GAINS.kI(),
            ShooterPivotConstants.GAINS.kD(),
            ShooterPivotConstants.GAINS.kS(),
            ShooterPivotConstants.GAINS.kG(),
            ShooterPivotConstants.GAINS.kV(),
            ShooterPivotConstants.GAINS.kA(),
            ShooterPivotConstants.MOTION_MAGIC_CRUISE_VEL,
            ShooterPivotConstants.MOTION_MAGIC_ACCEL,
            ShooterPivotConstants.MOTION_MAGIC_JERK,
            ShooterPivotConstants.POSITION_TOLERANCE,
            ShooterPivotConstants.SOFT_LIMITS_ENABLED,
            ShooterPivotConstants.SOFT_LIMIT_MIN,
            ShooterPivotConstants.SOFT_LIMIT_MAX,
            ShooterPivotConstants.MAX_VOLTAGE));
  }
}
