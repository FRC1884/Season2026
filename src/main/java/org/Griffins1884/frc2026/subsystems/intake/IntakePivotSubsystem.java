package org.Griffins1884.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.generic.arms.GenericArmSystemIO;
import org.Griffins1884.frc2026.generic.arms.GenericPositionArmSystem;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

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
  private IntakePivotGoal previousGoal = IntakePivotGoal.IDLING;
  private boolean hardStopLatched = false;
  private String hardStopAction = "NONE";
  private int hardStopSpikeSamples = 0;
  private static final double LOOP_PERIOD_SEC = 0.02;

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
    clearGoalOverrideInternal();
  }

  public void setGoalPosition(double position) {
    setGoalPositionInternal(position);
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

  @Override
  public void periodic() {
    IntakePivotGoal activeGoal = goal;
    if (activeGoal != previousGoal) {
      resetHardStopState();
      previousGoal = activeGoal;
    }

    if (!isHardStopGoal(activeGoal)) {
      logHardStop(activeGoal, false, 0.0, false);
      return;
    }

    double seekPosition = getSeekPosition(activeGoal);
    boolean spikeDetected = false;
    if (!hardStopLatched) {
      setGoalPositionInternal(seekPosition);
      spikeDetected = isHardStopSpike(activeGoal);
      hardStopSpikeSamples = spikeDetected ? hardStopSpikeSamples + 1 : 0;

      int requiredSamples =
          Math.max(
              1,
              (int)
                  Math.ceil(
                      IntakePivotConstants.HARDSTOP_SPIKE_DEBOUNCE_SEC.get() / LOOP_PERIOD_SEC));
      if (hardStopSpikeSamples >= requiredSamples) {
        hardStopLatched = true;
        if (activeGoal == IntakePivotGoal.IDLING) {
          zeroPositionInternal();
          stopOpenLoopInternal();
          clearGoalOverrideInternal();
          hardStopAction = "STOW_ZERO";
        } else {
          setOpenLoopInternal(0.0);
          hardStopAction = "PICKUP_STOP";
        }
      }
    }

    if (hardStopLatched) {
      if (activeGoal == IntakePivotGoal.PICKUP) {
        setOpenLoopInternal(0.0);
      } else {
        stopOpenLoopInternal();
        clearGoalOverrideInternal();
      }
    }
    logHardStop(activeGoal, true, seekPosition, spikeDetected);
  }

  private void resetHardStopState() {
    hardStopLatched = false;
    hardStopAction = "NONE";
    hardStopSpikeSamples = 0;
    stopOpenLoopInternal();
    clearGoalOverrideInternal();
  }

  private void clearGoalOverrideInternal() {
    primary.clearGoalOverride();
    secondary.clearGoalOverride();
  }

  private void setGoalPositionInternal(double position) {
    primary.setGoalPosition(position);
    secondary.setGoalPosition(position);
  }

  private void setOpenLoopInternal(double percent) {
    primary.setOpenLoop(percent);
    secondary.setOpenLoop(percent);
  }

  private void stopOpenLoopInternal() {
    primary.stopOpenLoop();
    secondary.stopOpenLoop();
  }

  private void zeroPositionInternal() {
    primary.zeroPosition();
    secondary.zeroPosition();
  }

  private static boolean isHardStopGoal(IntakePivotGoal activeGoal) {
    return activeGoal == IntakePivotGoal.IDLING || activeGoal == IntakePivotGoal.PICKUP;
  }

  private static double getSeekPosition(IntakePivotGoal activeGoal) {
    return activeGoal == IntakePivotGoal.IDLING
        ? IntakePivotConstants.HARDSTOP_STOW_SEEK_POSITION.get()
        : IntakePivotConstants.HARDSTOP_PICKUP_SEEK_POSITION.get();
  }

  private boolean isHardStopSpike(IntakePivotGoal activeGoal) {
    double maxCurrentAmps =
        Math.max(
            Math.abs(primary.getSupplyCurrentAmps()), Math.abs(secondary.getSupplyCurrentAmps()));
    double maxVelocityRadPerSec =
        Math.max(Math.abs(primary.getVelocity()), Math.abs(secondary.getVelocity()));
    double currentThresholdAmps =
        activeGoal == IntakePivotGoal.IDLING
            ? IntakePivotConstants.HARDSTOP_STOW_CURRENT_AMPS.get()
            : IntakePivotConstants.HARDSTOP_PICKUP_CURRENT_AMPS.get();

    Logger.recordOutput("IntakePivot/HardStop/CurrentAmps", maxCurrentAmps);
    Logger.recordOutput("IntakePivot/HardStop/VelocityRadPerSec", maxVelocityRadPerSec);
    Logger.recordOutput("IntakePivot/HardStop/CurrentThresholdAmps", currentThresholdAmps);
    return maxCurrentAmps >= currentThresholdAmps
        && maxVelocityRadPerSec <= IntakePivotConstants.HARDSTOP_MAX_VELOCITY_RAD_PER_SEC.get();
  }

  private void logHardStop(
      IntakePivotGoal activeGoal, boolean enabled, double seekPosition, boolean spikeDetected) {
    Logger.recordOutput("IntakePivot/HardStop/Enabled", enabled);
    Logger.recordOutput("IntakePivot/HardStop/Goal", activeGoal.toString());
    Logger.recordOutput("IntakePivot/HardStop/SeekPosition", seekPosition);
    Logger.recordOutput("IntakePivot/HardStop/Latched", hardStopLatched);
    Logger.recordOutput("IntakePivot/HardStop/Action", hardStopAction);
    Logger.recordOutput("IntakePivot/HardStop/SpikeSamples", hardStopSpikeSamples);
    Logger.recordOutput("IntakePivot/HardStop/SpikeDetected", spikeDetected);
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
              IntakePivotConstants.GAINS.kS(),
              IntakePivotConstants.GAINS.kG(),
              IntakePivotConstants.GAINS.kV(),
              IntakePivotConstants.GAINS.kA(),
              IntakePivotConstants.MOTION_MAGIC_CRUISE_VEL,
              IntakePivotConstants.MOTION_MAGIC_ACCEL,
              IntakePivotConstants.MOTION_MAGIC_JERK,
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
