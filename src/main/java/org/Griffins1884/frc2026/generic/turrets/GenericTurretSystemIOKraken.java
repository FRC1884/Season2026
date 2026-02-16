package org.Griffins1884.frc2026.generic.turrets;

import static org.Griffins1884.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class GenericTurretSystemIOKraken implements GenericTurretSystemIO {
  private static final double TICKS_PER_ROTATION = 2048.0;
  private final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final PositionTorqueCurrentFOC positionRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final double gearRatio;
  private double lastKP = Double.NaN;
  private double lastKI = Double.NaN;
  private double lastKD = Double.NaN;
  private double lastPositionSetpointRotations = Double.NaN;

  private final StatusSignal<?> positionSignal;
  private final StatusSignal<?> velocitySignal;
  private final StatusSignal<?> appliedVoltageSignal;
  private final StatusSignal<?> supplyCurrentSignal;
  private final StatusSignal<?> torqueCurrentSignal;
  private final StatusSignal<?> tempSignal;

  public GenericTurretSystemIOKraken(
      int id, int currentLimitAmps, boolean invert, boolean brake, double gearRatio) {
    this(id, currentLimitAmps, invert, brake, gearRatio, new CANBus("rio"));
  }

  public GenericTurretSystemIOKraken(
      int id,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double gearRatio,
      CANBus canBus) {
    this.gearRatio = gearRatio;
    motor = new TalonFX(id, canBus);

    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.TorqueCurrent.PeakForwardTorqueCurrent = currentLimitAmps;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -currentLimitAmps;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    config.Slot0.kP = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    appliedVoltageSignal = motor.getMotorVoltage();
    supplyCurrentSignal = motor.getSupplyCurrent();
    torqueCurrentSignal = motor.getTorqueCurrent();
    tempSignal = motor.getDeviceTemp();
  }

  @Override
  public void updateInputs(GenericTurretSystemIOInputs inputs) {
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
    inputs.positionRad = Units.rotationsToRadians(positionRotations) / gearRatio;
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotationsPerSec) / gearRatio;
    inputs.motorPositionRotations = positionRotations;
    inputs.motorPositionTicks = positionRotations * TICKS_PER_ROTATION;
    inputs.motorGoalRotations = lastPositionSetpointRotations;
    inputs.motorGoalTicks = lastPositionSetpointRotations * TICKS_PER_ROTATION;
    inputs.appliedVoltage = appliedVoltageSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentSignal.getValueAsDouble();
    inputs.tempCelsius = tempSignal.getValueAsDouble();
    inputs.absoluteConnected = false;
    inputs.absolutePositionRad = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public boolean usesInternalPositionControl() {
    return true;
  }

  @Override
  public void setPositionSetpoint(double positionRad, double kP, double kI, double kD) {
    if (pidChanged(kP, kI, kD)) {
      config.Slot0.kP = kP;
      config.Slot0.kI = kI;
      config.Slot0.kD = kD;
      tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
      lastKP = kP;
      lastKI = kI;
      lastKD = kD;
    }
    double wrappedRad = MathUtil.inputModulus(positionRad, 0.0, 2.0 * Math.PI);
    double positionRotations = (wrappedRad / (2.0 * Math.PI)) * gearRatio;
    lastPositionSetpointRotations = positionRotations;
    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    TalonFXConfiguration updated = new TalonFXConfiguration();
    motor.getConfigurator().refresh(updated);
    updated.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    tryUntilOk(5, () -> motor.getConfigurator().apply(updated, 0.25));
  }

  @Override
  public void setPosition(double positionRad) {
    motor.setPosition(Units.radiansToRotations(positionRad) * gearRatio);
  }

  private boolean pidChanged(double kP, double kI, double kD) {
    if (Double.isNaN(lastKP) || Double.isNaN(lastKI) || Double.isNaN(lastKD)) {
      return true;
    }
    return Double.compare(kP, lastKP) != 0
        || Double.compare(kI, lastKI) != 0
        || Double.compare(kD, lastKD) != 0;
  }
}
