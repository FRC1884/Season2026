package org.Griffins1884.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
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
import org.Griffins1884.frc2026.generic.arms.GenericPositionArmSystem.ControlMode;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakePivotSubsystem extends SubsystemBase {
  @RequiredArgsConstructor
  @Getter
  public enum IntakePivotGoal implements GenericPositionArmSystem.PivotGoal {
    IDLING(IntakePivotConstants.IDLE_ANGLE_RAD),
    PICKUP(IntakePivotConstants.PICKUP_RAD),
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
  private final DigitalInput primaryZeroLimitSwitch;
  private final DigitalInput secondaryZeroLimitSwitch;
  private boolean zeroingRequested = false;
  private boolean zeroingLatched = false;
  private String zeroingAction = "IDLE";
  private int zeroingDetectSamples = 0;
  private static final double LOOP_PERIOD_SEC = 0.02;

  public IntakePivotSubsystem(String name, IntakePivotIO primaryIO, IntakePivotIO secondaryIO) {
    primary = new IntakePivotArm(name, primaryIO, () -> goal);
    secondary = new IntakePivotArm(name + "Follower", secondaryIO, () -> goal);
    primaryZeroLimitSwitch =
        createLimitSwitch(IntakePivotConstants.PRIMARY_ZERO_LIMIT_SWITCH_DIO_CHANNEL);
    secondaryZeroLimitSwitch =
        createLimitSwitch(IntakePivotConstants.SECONDARY_ZERO_LIMIT_SWITCH_DIO_CHANNEL);
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

  public void requestZeroCalibration() {
    zeroingRequested = true;
    zeroingLatched = false;
    zeroingAction = "REQUESTED";
    zeroingDetectSamples = 0;
    stopOpenLoopInternal();
    clearGoalOverrideInternal();
  }

  public void cancelZeroCalibration() {
    zeroingRequested = false;
    zeroingLatched = false;
    zeroingAction = "CANCELLED";
    zeroingDetectSamples = 0;
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
    return (primary.getPosition() + secondary.getPosition()) * 0.5;
  }

  public void setBrakeMode(boolean enabled) {
    primary.setBrakeMode(enabled);
    secondary.setBrakeMode(enabled);
  }

  @Override
  public void periodic() {
    IntakePivotGoal activeGoal = goal;
    boolean openLoopActive = isOpenLoopControlActive();
    if (!openLoopActive) {
      setGoalPositionInternal(activeGoal.getAngle().getAsDouble());
    }

    double seekPosition = IntakePivotConstants.HARDSTOP_STOW_SEEK_POSITION.get();
    boolean zeroConditionDetected = false;
    if (zeroingRequested) {
      setGoalPositionInternal(seekPosition);
      zeroConditionDetected = isZeroingConditionMet();
      zeroingDetectSamples = zeroConditionDetected ? zeroingDetectSamples + 1 : 0;

      int requiredSamples =
          Math.max(
              1,
              (int)
                  Math.ceil(
                      IntakePivotConstants.HARDSTOP_SPIKE_DEBOUNCE_SEC.get() / LOOP_PERIOD_SEC));
      if (zeroingDetectSamples >= requiredSamples) {
        zeroingLatched = true;
        zeroingRequested = false;
        zeroPositionInternal();
        stopOpenLoopInternal();
        clearGoalOverrideInternal();
        zeroingAction = "ZEROED";
      } else {
        zeroingAction = "SEEKING";
      }
    }

    if (!zeroingRequested && !zeroingLatched && !"CANCELLED".equals(zeroingAction)) {
      zeroingAction = "IDLE";
    }
    logZeroingStatus(activeGoal, seekPosition, zeroConditionDetected);
  }

  private static DigitalInput createLimitSwitch(int channel) {
    if (channel < 0) {
      return null;
    }
    return new DigitalInput(channel);
  }

  private void clearGoalOverrideInternal() {
    primary.clearGoalOverride();
    secondary.clearGoalOverride();
  }

  private void setGoalPositionInternal(double position) {
    double primaryPosition = primary.getPosition();
    double secondaryPosition = secondary.getPosition();
    double averagePosition = (primaryPosition + secondaryPosition) * 0.5;
    double safePosition = Double.isFinite(position) ? position : averagePosition;
    double syncError = primaryPosition - secondaryPosition;
    if (!Double.isFinite(syncError)) {
      syncError = 0.0;
    }

    double deadband = Math.max(0.0, IntakePivotConstants.SYNC_DEADBAND_RAD.get());
    if (Math.abs(syncError) < deadband) {
      syncError = 0.0;
    }

    double maxTrim = Math.max(0.0, IntakePivotConstants.SYNC_MAX_TRIM_RAD.get());
    double correction =
        clamp(syncError * IntakePivotConstants.SYNC_CORRECTION_KP.get(), -maxTrim, maxTrim);

    double primaryGoal = safePosition - (correction * 0.5);
    double secondaryGoal = safePosition + (correction * 0.5);

    primary.setGoalPosition(primaryGoal);
    secondary.setGoalPosition(secondaryGoal);

    Logger.recordOutput("IntakePivot/Sync/PrimaryPositionRad", primaryPosition);
    Logger.recordOutput("IntakePivot/Sync/SecondaryPositionRad", secondaryPosition);
    Logger.recordOutput("IntakePivot/Sync/ErrorRad", syncError);
    Logger.recordOutput("IntakePivot/Sync/CorrectionRad", correction);
    Logger.recordOutput("IntakePivot/Sync/PrimaryGoalRad", primaryGoal);
    Logger.recordOutput("IntakePivot/Sync/SecondaryGoalRad", secondaryGoal);
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

  private boolean isZeroingConditionMet() {
    Logger.recordOutput(
        "IntakePivot/Zeroing/DetectionMode", IntakePivotConstants.ZEROING_DETECTION_MODE.name());
    return switch (IntakePivotConstants.ZEROING_DETECTION_MODE) {
      case LIMIT_SWITCH -> isLimitSwitchZeroDetected();
      case CURRENT_SPIKE -> isCurrentSpikeZeroDetected();
    };
  }

  private boolean isLimitSwitchZeroDetected() {
    boolean primaryConfigured = primaryZeroLimitSwitch != null;
    boolean secondaryConfigured = secondaryZeroLimitSwitch != null;
    boolean primaryPressed = readLimitSwitch(primaryZeroLimitSwitch);
    boolean secondaryPressed = readLimitSwitch(secondaryZeroLimitSwitch);

    boolean detected;
    if (!primaryConfigured && !secondaryConfigured) {
      detected = false;
    } else if (IntakePivotConstants.ZERO_LIMIT_SWITCH_REQUIRE_BOTH
        && primaryConfigured
        && secondaryConfigured) {
      detected = primaryPressed && secondaryPressed;
    } else {
      detected = primaryPressed || secondaryPressed;
    }

    Logger.recordOutput("IntakePivot/Zeroing/LimitSwitchPrimaryConfigured", primaryConfigured);
    Logger.recordOutput("IntakePivot/Zeroing/LimitSwitchSecondaryConfigured", secondaryConfigured);
    Logger.recordOutput("IntakePivot/Zeroing/LimitSwitchPrimaryPressed", primaryPressed);
    Logger.recordOutput("IntakePivot/Zeroing/LimitSwitchSecondaryPressed", secondaryPressed);
    Logger.recordOutput("IntakePivot/Zeroing/LimitSwitchDetected", detected);
    return detected;
  }

  private boolean readLimitSwitch(DigitalInput limitSwitch) {
    if (limitSwitch == null) {
      return false;
    }
    boolean rawState = limitSwitch.get();
    return IntakePivotConstants.ZERO_LIMIT_SWITCH_ACTIVE_LOW ? !rawState : rawState;
  }

  private boolean isOpenLoopControlActive() {
    return primary.getControlMode() == ControlMode.OPEN_LOOP
        || secondary.getControlMode() == ControlMode.OPEN_LOOP;
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private boolean isCurrentSpikeZeroDetected() {
    double maxCurrentAmps =
        Math.max(
            Math.abs(primary.getSupplyCurrentAmps()), Math.abs(secondary.getSupplyCurrentAmps()));
    double maxVelocityRadPerSec =
        Math.max(Math.abs(primary.getVelocity()), Math.abs(secondary.getVelocity()));
    double currentThresholdAmps = IntakePivotConstants.HARDSTOP_STOW_CURRENT_AMPS.get();

    Logger.recordOutput("IntakePivot/HardStop/CurrentAmps", maxCurrentAmps);
    Logger.recordOutput("IntakePivot/HardStop/VelocityRadPerSec", maxVelocityRadPerSec);
    Logger.recordOutput("IntakePivot/HardStop/CurrentThresholdAmps", currentThresholdAmps);
    return maxCurrentAmps >= currentThresholdAmps
        && maxVelocityRadPerSec <= IntakePivotConstants.HARDSTOP_MAX_VELOCITY_RAD_PER_SEC.get();
  }

  private void logZeroingStatus(
      IntakePivotGoal activeGoal, double seekPosition, boolean zeroConditionDetected) {
    Logger.recordOutput("IntakePivot/Zeroing/Requested", zeroingRequested);
    Logger.recordOutput("IntakePivot/Zeroing/Goal", activeGoal.toString());
    Logger.recordOutput("IntakePivot/Zeroing/SeekPosition", seekPosition);
    Logger.recordOutput("IntakePivot/Zeroing/Latched", zeroingLatched);
    Logger.recordOutput("IntakePivot/Zeroing/Action", zeroingAction);
    Logger.recordOutput("IntakePivot/Zeroing/DetectSamples", zeroingDetectSamples);
    Logger.recordOutput("IntakePivot/Zeroing/ConditionDetected", zeroConditionDetected);
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
