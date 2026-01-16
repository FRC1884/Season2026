package frc.robot.generic.turrets;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;

public class GenericTurretSystemIOSparkFlex implements GenericTurretSystemIO {
  private final SparkFlex motor;
  private final RelativeEncoder relativeEncoder;
  private final AbsoluteEncoder absoluteEncoder;
  private final double gearRatio;
  private final double absoluteGearRatio;
  private SparkBaseConfig config;

  public GenericTurretSystemIOSparkFlex(
      int id, int currentLimitAmps, boolean invert, boolean brake, double gearRatio) {
    this(id, currentLimitAmps, invert, brake, gearRatio, gearRatio);
  }

  public GenericTurretSystemIOSparkFlex(
      int id,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double gearRatio,
      double absoluteGearRatio) {
    this.gearRatio = gearRatio;
    this.absoluteGearRatio = absoluteGearRatio;
    motor = new SparkFlex(id, MotorType.kBrushless);

    config =
        new SparkFlexConfig()
            .smartCurrentLimit(currentLimitAmps)
            .idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast)
            .inverted(invert);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    relativeEncoder = motor.getEncoder();
    absoluteEncoder = motor.getAbsoluteEncoder();
  }

  @Override
  public void updateInputs(GenericTurretSystemIOInputs inputs) {
    if (inputs.connected.length != 1) {
      inputs.connected = new boolean[] {true};
    } else {
      inputs.connected[0] = true;
    }

    inputs.positionRad = Units.rotationsToRadians(relativeEncoder.getPosition()) / gearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(relativeEncoder.getVelocity()) / gearRatio;
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.torqueCurrentAmps = inputs.supplyCurrentAmps;
    inputs.tempCelsius = motor.getMotorTemperature();

    if (absoluteEncoder != null) {
      inputs.absoluteConnected = true;
      inputs.absolutePositionRad =
          Units.rotationsToRadians(absoluteEncoder.getPosition()) / absoluteGearRatio;
    } else {
      inputs.absoluteConnected = false;
      inputs.absolutePositionRad = 0.0;
    }
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    config = config.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setPosition(double positionRad) {
    relativeEncoder.setPosition(Units.radiansToRotations(positionRad) * gearRatio);
  }
}
