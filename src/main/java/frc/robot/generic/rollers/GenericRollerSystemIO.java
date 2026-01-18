package frc.robot.generic.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerSystemIO {
  @AutoLog
  abstract class GenericRollerSystemIOInputs {
    public boolean[] connected = {true, true};
    public double positionRads = 0.0;
    public double velocity = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(GenericRollerSystemIOInputs inputs) {}

  /** Run roller system at volts */
  default void runVolts(double volts) {}

  /** Stop roller system */
  default void stop() {}

  /** Enable or disable brake mode. */
  default void setBrakeMode(boolean enabled) {}
}
