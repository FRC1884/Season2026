package org.Griffins1884.frc2026.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class IndexerIOMax implements IndexerIO {
  private final SparkMax leader;
  private final SparkMax follower;
  private final RelativeEncoder encoder;
  private SparkBaseConfig config;

  public IndexerIOMax() {
    config =
        new SparkMaxConfig()
            .smartCurrentLimit(IndexerConstants.CURRENT_LIMIT_AMPS)
            .inverted(IndexerConstants.INVERTED)
            .idleMode(IndexerConstants.BRAKE_MODE ? IdleMode.kBrake : IdleMode.kCoast);

    leader = new SparkMax(IndexerConstants.LEADER_ID, MotorType.kBrushless);
    leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (IndexerConstants.FOLLOWER_ID >= 0) {
      follower = new SparkMax(IndexerConstants.FOLLOWER_ID, MotorType.kBrushless);
      follower.configure(
          new SparkMaxConfig().apply(config).follow(leader, IndexerConstants.FOLLOWER_INVERTED),
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    } else {
      follower = null;
    }

    encoder = leader.getEncoder();
  }

  @Override
  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    int motorCount = follower == null ? 1 : 2;
    if (inputs.connected.length != motorCount) {
      inputs.connected = new boolean[motorCount];
    }
    for (int i = 0; i < motorCount; i++) {
      inputs.connected[i] = true;
    }

    inputs.positionRads =
        Units.rotationsToRadians(encoder.getPosition()) / IndexerConstants.REDUCTION;
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity())
            / IndexerConstants.REDUCTION;
    inputs.velocity = encoder.getVelocity();
    inputs.appliedVoltage = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.supplyCurrentAmps = leader.getOutputCurrent();
    inputs.torqueCurrentAmps = inputs.supplyCurrentAmps;
    inputs.tempCelsius = leader.getMotorTemperature();
  }

  @Override
  public void runVolts(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    config = config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    leader.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    if (follower != null) {
      follower.configure(
          new SparkMaxConfig().apply(config).follow(leader, IndexerConstants.FOLLOWER_INVERTED),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
    }
  }
}
