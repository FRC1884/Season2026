package org.Griffins1884.frc2026.generic.rollers;

import static org.Griffins1884.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class GenericRollerSystemIOKraken implements GenericRollerSystemIO {
  private final TalonFX[] motors;
  private final TalonFX leader;
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
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
        new CANBus("rio"));
  }

  public GenericRollerSystemIOKraken(
      int[] ids, int currentLimitAmps, boolean[] inverted, boolean brake, double reduction) {
    this(ids, currentLimitAmps, inverted, brake, reduction, new CANBus("rio"));
  }

  public GenericRollerSystemIOKraken(
      int id,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction,
      CANBus canBus) {
    this(new int[] {id}, currentLimitAmps, new boolean[] {invert}, brake, reduction, canBus);
  }

  public GenericRollerSystemIOKraken(
      int[] ids,
      int currentLimitAmps,
      boolean[] inverted,
      boolean brake,
      double reduction,
      CANBus canBus) {
    this.reduction = reduction;

    motors = new TalonFX[ids.length];
    leader = motors[0] = new TalonFX(ids[0], canBus);
    applyConfig(leader, currentLimitAmps, brake, inverted[0]);

    if (ids.length > 1) {
      for (int i = 1; i < ids.length; i++) {
        TalonFX follower = motors[i] = new TalonFX(ids[i], canBus);
        applyConfig(follower, currentLimitAmps, brake, inverted[i]);
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
      TalonFX motor, int currentLimitAmps, boolean brake, boolean inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
  }
}
