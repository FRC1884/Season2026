package frc.robot.generic.turrets;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;

public class GenericTurretSystemIOKraken implements GenericTurretSystemIO {
  private final TalonFX motor;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final double gearRatio;
  private final double absoluteGearRatio;
  private final CANcoder absoluteEncoder;

  private final StatusSignal<?> positionSignal;
  private final StatusSignal<?> velocitySignal;
  private final StatusSignal<?> appliedVoltageSignal;
  private final StatusSignal<?> supplyCurrentSignal;
  private final StatusSignal<?> torqueCurrentSignal;
  private final StatusSignal<?> tempSignal;
  private final StatusSignal<?> absolutePositionSignal;

  public GenericTurretSystemIOKraken(
      int id, int currentLimitAmps, boolean invert, boolean brake, double gearRatio) {
    this(id, currentLimitAmps, invert, brake, gearRatio, -1, false, 1.0, "");
  }

  public GenericTurretSystemIOKraken(
      int id,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double gearRatio,
      int cancoderId,
      boolean cancoderInverted,
      double absoluteGearRatio) {
    this(
        id,
        currentLimitAmps,
        invert,
        brake,
        gearRatio,
        cancoderId,
        cancoderInverted,
        absoluteGearRatio,
        "");
  }

  public GenericTurretSystemIOKraken(
      int id,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double gearRatio,
      int cancoderId,
      boolean cancoderInverted,
      double absoluteGearRatio,
      String canBus) {
    this.gearRatio = gearRatio;
    this.absoluteGearRatio = absoluteGearRatio;
    motor = new TalonFX(id, canBus);

    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    if (cancoderId >= 0) {
      absoluteEncoder = new CANcoder(cancoderId, canBus);
      CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
      cancoderConfig.MagnetSensor.SensorDirection =
          cancoderInverted
              ? SensorDirectionValue.Clockwise_Positive
              : SensorDirectionValue.CounterClockwise_Positive;
      tryUntilOk(5, () -> absoluteEncoder.getConfigurator().apply(cancoderConfig, 0.25));
      absolutePositionSignal = absoluteEncoder.getAbsolutePosition();
    } else {
      absoluteEncoder = null;
      absolutePositionSignal = null;
    }

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    appliedVoltageSignal = motor.getMotorVoltage();
    supplyCurrentSignal = motor.getSupplyCurrent();
    torqueCurrentSignal = motor.getTorqueCurrent();
    tempSignal = motor.getDeviceTemp();
  }

  @Override
  public void updateInputs(GenericTurretSystemIOInputs inputs) {
    if (absolutePositionSignal != null) {
      BaseStatusSignal.refreshAll(
          positionSignal,
          velocitySignal,
          appliedVoltageSignal,
          supplyCurrentSignal,
          torqueCurrentSignal,
          tempSignal,
          absolutePositionSignal);
    } else {
      BaseStatusSignal.refreshAll(
          positionSignal,
          velocitySignal,
          appliedVoltageSignal,
          supplyCurrentSignal,
          torqueCurrentSignal,
          tempSignal);
    }

    if (inputs.connected.length != 1) {
      inputs.connected = new boolean[] {true};
    } else {
      inputs.connected[0] = positionSignal.getStatus().isOK();
    }

    double positionRotations = positionSignal.getValueAsDouble();
    double velocityRotationsPerSec = velocitySignal.getValueAsDouble();
    inputs.positionRad = Units.rotationsToRadians(positionRotations) / gearRatio;
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotationsPerSec) / gearRatio;
    inputs.appliedVoltage = appliedVoltageSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentSignal.getValueAsDouble();
    inputs.tempCelsius = tempSignal.getValueAsDouble();

    if (absolutePositionSignal != null) {
      inputs.absoluteConnected = true;
      inputs.absolutePositionRad =
          Units.rotationsToRadians(absolutePositionSignal.getValueAsDouble()) / absoluteGearRatio;
    } else {
      inputs.absoluteConnected = false;
      inputs.absolutePositionRad = 0.0;
    }
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
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
}
