package org.Griffins1884.frc2026.subsystems.intake;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.generic.arms.GenericArmSystemIO;
import org.Griffins1884.frc2026.generic.arms.GenericPositionArmSystem;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public class IntakePivotSubsystem extends SubsystemBase {
  @RequiredArgsConstructor
  @Getter
  public enum IntakePivotGoal implements GenericPositionArmSystem.PivotGoal {
    IDLING(() -> IntakePivotConstants.IDLE_ANGLE_RAD),
    PICKUP(() -> IntakePivotConstants.PICKUP_RAD),
    TESTING(new LoggedTunableNumber("IntakePivot/Test", 0.0));

    private final DoubleSupplier angleSupplier;

    @Override
    public DoubleSupplier getAngle() {
      return () -> angleSupplier.getAsDouble();
    }
  }

  @Setter @Getter private IntakePivotGoal goal = IntakePivotGoal.IDLING;
  private final IntakePivotArm primary;
  private final IntakePivotArm secondary;

  public IntakePivotSubsystem(String name, IntakePivotIO primaryIO, IntakePivotIO secondaryIO) {
    primary = new IntakePivotArm(name, primaryIO, () -> goal);
    secondary = new IntakePivotArm(name + "Follower", secondaryIO, () -> goal);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.parallel(
        primary.sysIdQuasistatic(direction), secondary.sysIdQuasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.parallel(primary.sysIdDynamic(direction), secondary.sysIdDynamic(direction));
  }

  public void clearGoalOverride() {
    primary.clearGoalOverride();
    secondary.clearGoalOverride();
  }

  public void setGoalPosition(double position) {
    primary.setGoalPosition(position);
    secondary.setGoalPosition(position);
  }

  public void setOpenLoop(double percent) {
    primary.setOpenLoop(percent);
    secondary.setOpenLoop(percent);
  }

  public void stopOpenLoop() {
    primary.stopOpenLoop();
    secondary.stopOpenLoop();
  }

  public boolean isAtGoal() {
    return primary.isAtGoal() && secondary.isAtGoal();
  }

  public double getPosition() {
    return primary.getPosition();
  }

  public void setBrakeMode(boolean enabled) {
    primary.setBrakeMode(enabled);
    secondary.setBrakeMode(enabled);
  }

  private static final class IntakePivotArm extends GenericPositionArmSystem<IntakePivotGoal> {
    private final Supplier<IntakePivotGoal> goalSupplier;

    private IntakePivotArm(
        String name, GenericArmSystemIO io, Supplier<IntakePivotGoal> goalSupplier) {
      super(
          name,
          io,
          new ArmConfig(
              IntakePivotConstants.GAINS.kP(),
              IntakePivotConstants.GAINS.kI(),
              IntakePivotConstants.GAINS.kD(),
              IntakePivotConstants.POSITION_TOLERANCE,
              IntakePivotConstants.SOFT_LIMITS_ENABLED,
              IntakePivotConstants.SOFT_LIMIT_MIN,
              IntakePivotConstants.SOFT_LIMIT_MAX,
              IntakePivotConstants.MAX_VOLTAGE));
      this.goalSupplier = goalSupplier;
    }

    @Override
    public IntakePivotGoal getGoal() {
      return goalSupplier.get();
    }
  }
}
