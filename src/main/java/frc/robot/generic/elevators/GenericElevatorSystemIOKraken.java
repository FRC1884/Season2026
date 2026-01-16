package frc.robot.generic.elevators;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class GenericElevatorSystemIOKraken implements GenericElevatorSystemIO {
  private final TalonFX[] motors;
  private final TalonFX leader;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final double positionCoefficient;

  private final StatusSignal<?> positionSignal;
  private final StatusSignal<?> velocitySignal;
  private final StatusSignal<?> appliedVoltageSignal;
  private final StatusSignal<?> supplyCurrentSignal;
  private final StatusSignal<?> torqueCurrentSignal;
  private final StatusSignal<?> tempSignal;

  public GenericElevatorSystemIOKraken(
      int[] ids, int currentLimitAmps, boolean brake, double positionCoefficient) {
    this(ids, currentLimitAmps, brake, positionCoefficient, false, "");
  }

  public GenericElevatorSystemIOKraken(
      int[] ids,
      int currentLimitAmps,
      boolean brake,
      double positionCoefficient,
      boolean inverted,
      String canBus) {
    this.positionCoefficient = positionCoefficient;

    motors = new TalonFX[ids.length];
    leader = motors[0] = new TalonFX(ids[0], canBus);

    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));

    if (ids.length > 1) {
      for (int i = 1; i < ids.length; i++) {
        TalonFX follower = motors[i] = new TalonFX(ids[i], canBus);
        tryUntilOk(5, () -> follower.getConfigurator().apply(config, 0.25));
        follower.setControl(new Follower(leader.getDeviceID(), false));
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
  public void updateInputs(GenericElevatorSystemIOInputs inputs) {
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

    inputs.encoderPosition = positionSignal.getValueAsDouble() * positionCoefficient;
    inputs.velocity = velocitySignal.getValueAsDouble() * positionCoefficient;
    inputs.appliedVoltage = appliedVoltageSignal.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentSignal.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentSignal.getValueAsDouble();
    inputs.tempCelsius = tempSignal.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    TalonFXConfiguration updated = new TalonFXConfiguration();
    leader.getConfigurator().refresh(updated);
    updated.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    tryUntilOk(5, () -> leader.getConfigurator().apply(updated, 0.25));
  }

  @Override
  public void setPosition(double position) {
    leader.setPosition(position / positionCoefficient);
  }

  protected void invert(int index) {
    if (index < 0 || index >= motors.length) {
      return;
    }
    TalonFX motor = motors[index];
    TalonFXConfiguration invertedConfig = new TalonFXConfiguration();
    motor.getConfigurator().refresh(invertedConfig);
    invertedConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> motor.getConfigurator().apply(invertedConfig, 0.25));
  }
}
