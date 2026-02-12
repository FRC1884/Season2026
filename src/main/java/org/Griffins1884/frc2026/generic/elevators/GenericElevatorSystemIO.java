package org.Griffins1884.frc2026.generic.elevators;

import org.littletonrobotics.junction.AutoLog;

public interface GenericElevatorSystemIO {

  @AutoLog
  public static class GenericElevatorSystemIOInputs {
    public boolean[] connected = new boolean[] {};
    public double velocity = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double encoderPosition = 0.0;
    public double goal = 0.0;
  }

  default void updateInputs(GenericElevatorSystemIOInputs inputs) {}

  /** Run elevator system to a height */
  default void setVoltage(double volts) {}

  /** Whether this IO supports internal position control. */
  default boolean usesInternalPositionControl() {
    return false;
  }

  /** Run elevator system to a position using internal controller. */
  default void setPositionSetpoint(double position, double kP, double kI, double kD) {}

  /** Enable or disable brake mode. */
  default void setBrakeMode(boolean enabled) {}

  /** Reset the encoder position to the provided value. */
  default void setPosition(double position) {}
}
