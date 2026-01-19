package org.Griffins1884.frc2026.generic.rollers;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public abstract class GenericRollerSystemIOSparkMax implements GenericRollerSystemIO {
  private final SparkMax sparkMax;
  private final RelativeEncoder encoder;
  private SparkBaseConfig config;

  private final double reduction;

  public GenericRollerSystemIOSparkMax(
      int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    sparkMax = new SparkMax(id, MotorType.kBrushless);

    config =
        new SparkMaxConfig()
            .smartCurrentLimit(currentLimitAmps)
            .inverted(invert)
            .idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);

    sparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    encoder = sparkMax.getEncoder();
  }

  @Override
  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    if (inputs.connected.length != 1) {
      inputs.connected = new boolean[] {true};
    } else {
      inputs.connected[0] = true;
    }
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition()) / reduction;
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
    inputs.velocity = encoder.getVelocity();
    inputs.appliedVoltage = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
    inputs.supplyCurrentAmps = sparkMax.getOutputCurrent();
    inputs.torqueCurrentAmps = inputs.supplyCurrentAmps;
    inputs.tempCelsius = sparkMax.getMotorTemperature();
  }

  @Override
  public void runVolts(double volts) {
    sparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    sparkMax.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    config = config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    sparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
