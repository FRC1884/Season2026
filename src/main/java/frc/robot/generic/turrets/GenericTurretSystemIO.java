package frc.robot.generic.turrets;

import org.littletonrobotics.junction.AutoLog;

public interface GenericTurretSystemIO {
  @AutoLog
  class GenericTurretSystemIOInputs {
    public boolean[] connected = new boolean[] {};
    public boolean absoluteConnected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double absolutePositionRad = 0.0;
  }

  default void updateInputs(GenericTurretSystemIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void setBrakeMode(boolean enabled) {}

  default void setPosition(double positionRad) {}
}
