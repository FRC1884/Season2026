package org.Griffins1884.frc2026.generic.arms;

import static org.Griffins1884.frc2026.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class GenericArmSystemIOKraken implements GenericArmSystemIO {
  private final TalonFX[] motors;
  private final TalonFX leader;
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final PositionTorqueCurrentFOC positionTorqueRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final double positionCoefficient;
  private double lastKP = Double.NaN;
  private double lastKI = Double.NaN;
  private double lastKD = Double.NaN;
  private double lastKG = Double.NaN;

  private final StatusSignal<?> positionSignal;
  private final StatusSignal<?> velocitySignal;
  private final StatusSignal<?> appliedVoltageSignal;
  private final StatusSignal<?> supplyCurrentSignal;
  private final StatusSignal<?> torqueCurrentSignal;
  private final StatusSignal<?> tempSignal;

  public GenericArmSystemIOKraken(
      int[] ids,
      int currentLimitAmps,
      boolean brake,
      double forwardSoftLimit,
      double reverseSoftLimit,
      double positionCoefficient,
      boolean[] inverted,
      CANBus canBus) {
    this.positionCoefficient = positionCoefficient;

    motors = new TalonFX[ids.length];
    leader = motors[0] = new TalonFX(ids[0], canBus);

    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    InvertedValue leaderInvertedValue =
        inverted[0] ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.Inverted = leaderInvertedValue;
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
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimit / positionCoefficient;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimit / positionCoefficient;

    tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));

    if (ids.length > 1) {
      for (int i = 1; i < ids.length; i++) {
        TalonFX follower = motors[i] = new TalonFX(ids[i], canBus);
        config.MotorOutput.Inverted =
            inverted[i]
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> follower.getConfigurator().apply(config, 0.25));
        config.MotorOutput.Inverted = leaderInvertedValue;
        follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));
        follower.setPosition(0.0);
      }
    }

    leader.setPosition(0.0);
    positionSignal = leader.getPosition();
    velocitySignal = leader.getVelocity();
    appliedVoltageSignal = leader.getMotorVoltage();
    supplyCurrentSignal = leader.getSupplyCurrent();
    torqueCurrentSignal = leader.getTorqueCurrent();
    tempSignal = leader.getDeviceTemp();
  }

  @Override
  public void updateInputs(GenericArmSystemIOInputs inputs) {
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
  public boolean usesInternalPositionControl() {
    return true;
  }

  @Override
  public void setPositionSetpoint(double position, double kP, double kI, double kD, double kG) {
    if (pidChanged(kP, kI, kD, kG)) {
      config.Slot0.kP = kP;
      config.Slot0.kI = kI;
      config.Slot0.kD = kD;
      config.Slot0.kG = kG;
      config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
      lastKP = kP;
      lastKI = kI;
      lastKD = kD;
      lastKG = kG;
    }
    leader.setControl(positionTorqueRequest.withPosition(position / positionCoefficient));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leader.getConfigurator().refresh(config);
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
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

  private boolean pidChanged(double kP, double kI, double kD, double kG) {
    if (Double.isNaN(lastKP)
        || Double.isNaN(lastKI)
        || Double.isNaN(lastKD)
        || Double.isNaN(lastKG)) {
      return true;
    }
    return Double.compare(kP, lastKP) != 0
        || Double.compare(kI, lastKI) != 0
        || Double.compare(kD, lastKD) != 0
        || Double.compare(kG, lastKG) != 0;
  }
}
