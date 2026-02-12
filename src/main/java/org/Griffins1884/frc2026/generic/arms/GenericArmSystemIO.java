package org.Griffins1884.frc2026.generic.arms;

import org.littletonrobotics.junction.AutoLog;

public interface GenericArmSystemIO {

  @AutoLog
  public static class GenericArmSystemIOInputs {
    public boolean[] connected = new boolean[] {};
    public double velocity = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double encoderPosition = 0.0;
    public double goal = 0.0;
  }

  default void updateInputs(GenericArmSystemIOInputs inputs) {}

  /** Run arm system to an angle */
  default void setVoltage(double volts) {}

  /** Whether this IO supports internal position control. */
  default boolean usesInternalPositionControl() {
    return false;
  }

  /** Run arm system to a position using internal controller. */
  default void setPositionSetpoint(double position, double kP, double kI, double kD) {}

  /** Enable or disable brake mode. */
  default void setBrakeMode(boolean enabled) {}

  /** Reset the encoder position to the provided value. */
  default void setPosition(double position) {}
}
