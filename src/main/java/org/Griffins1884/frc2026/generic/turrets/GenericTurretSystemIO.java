package org.Griffins1884.frc2026.generic.turrets;

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
    public double motorPositionRotations = Double.NaN;
    public double motorPositionTicks = Double.NaN;
    public double motorGoalRotations = Double.NaN;
    public double motorGoalTicks = Double.NaN;
  }

  default void updateInputs(GenericTurretSystemIOInputs inputs) {}

  default void setVoltage(double volts) {}

  /** Whether this IO supports internal position control. */
  default boolean usesInternalPositionControl() {
    return false;
  }

  /** Run turret to a position using internal controller. */
  default void setPositionSetpoint(double positionRad, double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}

  default void setPosition(double positionRad) {}
}
