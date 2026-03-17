package org.Griffins1884.frc2026.mechanisms;

import org.littletonrobotics.junction.AutoLog;

/** Common mechanism-capable IO contract for the new unified mechanism layer. */
public interface MechanismIO {
  @AutoLog
  class MechanismIOInputs {
    public boolean[] motorConnected = new boolean[] {};
    public boolean sensorConnected = true;
    public boolean forwardLimitSwitch = false;
    public boolean reverseLimitSwitch = false;
    public double position = 0.0;
    public double velocity = 0.0;
    public double appliedOutput = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetPosition = 0.0;
    public double targetVelocity = 0.0;
    public double closedLoopError = 0.0;
    public String controlMode = "DISABLED";
    public String[] warnings = new String[] {};
    public String[] faults = new String[] {};
  }

  default void configure(MechanismDefinition definition) {}

  default void updateInputs(MechanismIOInputs inputs) {}

  default void setOpenLoop(double percent) {}

  default void setVoltage(double volts) {}

  default void setTargetPosition(double position) {}

  default void setTargetVelocity(double velocity) {}

  default void setTargetCurrent(double currentAmps) {}

  default void setPosition(double position) {}

  default void setBrakeMode(boolean enabled) {}

  default void stop() {}
}
