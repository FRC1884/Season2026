package org.Griffins1884.frc2026.generic.rollers;

import static org.Griffins1884.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class GenericRollerSystemIOKraken implements GenericRollerSystemIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
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
    this(id, currentLimitAmps, invert, brake, reduction, new CANBus("rio"));
  }

  public GenericRollerSystemIOKraken(
      int id,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction,
      CANBus canBus) {
    this.reduction = reduction;
    motor = new TalonFX(id, canBus);

    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    appliedVoltageSignal = motor.getMotorVoltage();
    supplyCurrentSignal = motor.getSupplyCurrent();
    torqueCurrentSignal = motor.getTorqueCurrent();
    tempSignal = motor.getDeviceTemp();
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

    if (inputs.connected.length != 1) {
      inputs.connected = new boolean[] {true};
    } else {
      inputs.connected[0] = positionSignal.getStatus().isOK();
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
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    TalonFXConfiguration updated = new TalonFXConfiguration();
    motor.getConfigurator().refresh(updated);
    updated.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    tryUntilOk(5, () -> motor.getConfigurator().apply(updated, 0.25));
  }
}
