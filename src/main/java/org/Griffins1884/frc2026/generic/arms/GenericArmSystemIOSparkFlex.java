package org.Griffins1884.frc2026.generic.arms;

import static com.revrobotics.spark.config.SparkBaseConfig.IdleMode.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public class GenericArmSystemIOSparkFlex implements GenericArmSystemIO {
  private final SparkFlex[] motors;
  private final AbsoluteEncoder absoluteEncoder;
  private final RelativeEncoder relativeEncoder;
  private SparkBaseConfig config;
  private final SparkFlex leader;
  private final double positionCoefficient;

  public GenericArmSystemIOSparkFlex(
      int[] ids, int currentLimitAmps, boolean brake, double forwardLimit, double reverseLimit) {
    this(ids, currentLimitAmps, brake, forwardLimit, reverseLimit, 1.0);
  }

  public GenericArmSystemIOSparkFlex(
      int[] ids,
      int currentLimitAmps,
      boolean brake,
      double forwardLimit,
      double reverseLimit,
      double positionCoefficient) {
    this.positionCoefficient = positionCoefficient;

    motors = new SparkFlex[ids.length];
    config =
        new SparkFlexConfig().smartCurrentLimit(currentLimitAmps).idleMode(brake ? kBrake : kCoast);
    config
        .softLimit
        .forwardSoftLimit(forwardLimit / positionCoefficient)
        .reverseSoftLimit(reverseLimit / positionCoefficient);

    leader = motors[0] = new SparkFlex(ids[0], SparkLowLevel.MotorType.kBrushless);
    leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (ids.length > 1) {
      for (int i = 1; i < ids.length; i++) {
        motors[i] = new SparkFlex(ids[i], SparkLowLevel.MotorType.kBrushless);
        motors[i].configure(
            new SparkFlexConfig().apply(config).follow(leader),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
      }
    }
    absoluteEncoder = leader.getAbsoluteEncoder();
    relativeEncoder = leader.getEncoder();
  }

  public void updateInputs(GenericArmSystemIOInputs inputs) {
    if (motors != null) {
      if (inputs.connected.length != motors.length) {
        inputs.connected = new boolean[motors.length];
      }
      for (int i = 0; i < motors.length; i++) {
        inputs.connected[i] = true;
      }
      double positionRotations =
          (absoluteEncoder != null) ? absoluteEncoder.getPosition() : relativeEncoder.getPosition();
      double velocityRpm =
          (absoluteEncoder != null) ? absoluteEncoder.getVelocity() : relativeEncoder.getVelocity();
      inputs.encoderPosition = positionRotations * positionCoefficient;
      inputs.velocity = velocityRpm * positionCoefficient / 60.0;
      inputs.appliedVoltage = leader.getAppliedOutput() * leader.getBusVoltage();
      inputs.supplyCurrentAmps = leader.getOutputCurrent();
      inputs.torqueCurrentAmps = leader.getOutputCurrent();
      inputs.tempCelsius = leader.getMotorTemperature();
    }
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    config = config.idleMode(enabled ? kBrake : kCoast);
    leader.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setPosition(double position) {
    if (relativeEncoder != null) {
      relativeEncoder.setPosition(position / positionCoefficient);
    }
  }

  protected void invert(int id) {
    motors[id].configure(
        new SparkFlexConfig().inverted(true),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }
}
