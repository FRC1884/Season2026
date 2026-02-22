package org.Griffins1884.frc2026.generic.rollers;

import static org.Griffins1884.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class GenericRollerSystemIOKraken implements GenericRollerSystemIO {
  private static final double DEFAULT_CLOSED_LOOP_RAMP_SECONDS = 0.02;

  public enum VelocityControlRequest {
    VELOCITY_VOLTAGE,
    VELOCITY_TORQUE_CURRENT_FOC
  }

  private final TalonFX[] motors;
  private final TalonFX leader;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withSlot(0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(0).withUpdateFreqHz(0);
  private final VelocityControlRequest velocityControlRequest;
  private int velocityControlSlot = 0;
  private final double reduction;

  private final StatusSignal<?> positionSignal;
  private final StatusSignal<?> velocitySignal;
  private final StatusSignal<?> appliedVoltageSignal;
  private final StatusSignal<?> supplyCurrentSignal;
  private final StatusSignal<?> torqueCurrentSignal;
  private final StatusSignal<?> tempSignal;

  public GenericRollerSystemIOKraken(
      int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this(
        new int[] {id},
        currentLimitAmps,
        new boolean[] {invert},
        brake,
        reduction,
        new CANBus("rio"),
        DEFAULT_CLOSED_LOOP_RAMP_SECONDS,
        VelocityControlRequest.VELOCITY_VOLTAGE);
  }

  public GenericRollerSystemIOKraken(
      int[] ids, int currentLimitAmps, boolean[] inverted, boolean brake, double reduction) {
    this(
        ids,
        currentLimitAmps,
        inverted,
        brake,
        reduction,
        new CANBus("rio"),
        DEFAULT_CLOSED_LOOP_RAMP_SECONDS,
        VelocityControlRequest.VELOCITY_VOLTAGE);
  }

  public GenericRollerSystemIOKraken(
      int id,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction,
      CANBus canBus) {
    this(
        new int[] {id},
        currentLimitAmps,
        new boolean[] {invert},
        brake,
        reduction,
        canBus,
        DEFAULT_CLOSED_LOOP_RAMP_SECONDS,
        VelocityControlRequest.VELOCITY_VOLTAGE);
  }

  public GenericRollerSystemIOKraken(
      int[] ids,
      int currentLimitAmps,
      boolean[] inverted,
      boolean brake,
      double reduction,
      CANBus canBus) {
    this(
        ids,
        currentLimitAmps,
        inverted,
        brake,
        reduction,
        canBus,
        DEFAULT_CLOSED_LOOP_RAMP_SECONDS,
        VelocityControlRequest.VELOCITY_VOLTAGE);
  }

  public GenericRollerSystemIOKraken(
      int[] ids,
      int currentLimitAmps,
      boolean[] inverted,
      boolean brake,
      double reduction,
      CANBus canBus,
      double closedLoopRampPeriodSeconds) {
    this(
        ids,
        currentLimitAmps,
        inverted,
        brake,
        reduction,
        canBus,
        closedLoopRampPeriodSeconds,
        VelocityControlRequest.VELOCITY_VOLTAGE);
  }

  public GenericRollerSystemIOKraken(
      int[] ids,
      int currentLimitAmps,
      boolean[] inverted,
      boolean brake,
      double reduction,
      CANBus canBus,
      double closedLoopRampPeriodSeconds,
      VelocityControlRequest velocityControlRequest) {
    this.reduction = reduction;
    this.velocityControlRequest = velocityControlRequest;

    motors = new TalonFX[ids.length];
    leader = motors[0] = new TalonFX(ids[0], canBus);
    applyConfig(
        leader, currentLimitAmps, brake, inverted[0], reduction, closedLoopRampPeriodSeconds);

    if (ids.length > 1) {
      for (int i = 1; i < ids.length; i++) {
        TalonFX follower = motors[i] = new TalonFX(ids[i], canBus);
        applyConfig(
            follower, currentLimitAmps, brake, inverted[i], reduction, closedLoopRampPeriodSeconds);
        follower.setControl(
            new Follower(
                leader.getDeviceID(),
                inverted[i] ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
      }
    }

    positionSignal = leader.getPosition();
    velocitySignal = leader.getVelocity();
    appliedVoltageSignal = leader.getMotorVoltage();
    supplyCurrentSignal = leader.getSupplyCurrent();
    torqueCurrentSignal = leader.getTorqueCurrent();
    tempSignal = leader.getDeviceTemp();
  }

  @Override
  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        appliedVoltageSignal,
        supplyCurrentSignal,
        torqueCurrentSignal,
        tempSignal);

    if (inputs.connected.length != motors.length) {
      inputs.connected = new boolean[motors.length];
    }
    boolean ok = positionSignal.getStatus().isOK();
    for (int i = 0; i < motors.length; i++) {
      inputs.connected[i] = ok;
    }

    double positionRotations = positionSignal.getValueAsDouble();
    double velocityRotationsPerSec = velocitySignal.getValueAsDouble();
    inputs.positionRads = Units.rotationsToRadians(positionRotations) / reduction;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocityRotationsPerSec) / reduction;
    inputs.velocity = velocityRotationsPerSec * 60.0;
    inputs.appliedVoltage = appliedVoltageSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentSignal.getValueAsDouble();
    inputs.tempCelsius = tempSignal.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void runVelocity(double velocityRpm, double feedforwardVolts) {
    double velocityRotationsPerSecond = velocityRpm / 60.0;
    if (velocityControlRequest == VelocityControlRequest.VELOCITY_TORQUE_CURRENT_FOC) {
      leader.setControl(
          velocityTorqueCurrentRequest
              .withSlot(velocityControlSlot)
              .withVelocity(velocityRotationsPerSecond)
              .withFeedForward(feedforwardVolts));
      return;
    }

    leader.setControl(
        velocityVoltageRequest
            .withSlot(velocityControlSlot)
            .withVelocity(velocityRotationsPerSecond)
            .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setVelocityPID(double kP, double kI, double kD) {
    setVelocityPID(0, kP, kI, kD);
  }

  @Override
  public void setVelocityPID(int slot, double kP, double kI, double kD) {
    TalonFXConfiguration updated = new TalonFXConfiguration();
    leader.getConfigurator().refresh(updated);
    int sanitizedSlot = sanitizeSlot(slot);
    if (sanitizedSlot == 0) {
      updated.Slot0.kP = kP;
      updated.Slot0.kI = kI;
      updated.Slot0.kD = kD;
    } else if (sanitizedSlot == 1) {
      updated.Slot1.kP = kP;
      updated.Slot1.kI = kI;
      updated.Slot1.kD = kD;
    } else {
      updated.Slot2.kP = kP;
      updated.Slot2.kI = kI;
      updated.Slot2.kD = kD;
    }
    tryUntilOk(5, () -> leader.getConfigurator().apply(updated, 0.25));
  }

  @Override
  public void setVelocityControlSlot(int slot) {
    velocityControlSlot = sanitizeSlot(slot);
  }

  @Override
  public boolean supportsVelocityControl() {
    return true;
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    for (TalonFX motor : motors) {
      TalonFXConfiguration updated = new TalonFXConfiguration();
      motor.getConfigurator().refresh(updated);
      updated.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      tryUntilOk(5, () -> motor.getConfigurator().apply(updated, 0.25));
    }
  }

  private static void applyConfig(
      TalonFX motor,
      int currentLimitAmps,
      boolean brake,
      boolean inverted,
      double reduction,
      double closedLoopRampPeriodSeconds) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRampPeriodSeconds;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRampPeriodSeconds;
    config.TorqueCurrent.PeakForwardTorqueCurrent = currentLimitAmps;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    if (reduction > 0.0) {
      config.Feedback.SensorToMechanismRatio = reduction;
    }
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }

  private static int sanitizeSlot(int slot) {
    if (slot <= 0) {
      return 0;
    }
    if (slot >= 2) {
      return 2;
    }
    return slot;
  }
}
