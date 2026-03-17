package org.Griffins1884.frc2026.mechanisms.arms;

import org.littletonrobotics.junction.AutoLog;

public interface MechanismArmIO {

  @AutoLog
  public static class MechanismArmIOInputs {
    public boolean[] connected = new boolean[] {};
    public double velocity = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double encoderPosition = 0.0;
    public double goal = 0.0;
  }

  default void updateInputs(MechanismArmIOInputs inputs) {}

  /** Run arm system to an angle */
  default void setVoltage(double volts) {}

  /** Whether this IO supports internal position control. */
  default boolean usesInternalPositionControl() {
    return false;
  }

  /** Run arm system to a position using internal controller. */
  default void setPositionSetpoint(double position, double kP, double kI, double kD, double kG) {}

  /** Update Motion Magic constraints if supported. */
  default void setMotionMagicParams(double cruiseVelocity, double acceleration, double jerk) {}

  /** Enable or disable brake mode. */
  default void setBrakeMode(boolean enabled) {}

  /** Reset the encoder position to the provided value. */
  default void setPosition(double position) {}
}
