package org.Griffins1884.frc2026.generic.rollers;

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

  /** Run roller system at velocity (RPM) with optional feedforward (volts). */
  default void runVelocity(double velocityRpm, double feedforwardVolts) {}

  /** Update the onboard velocity PID gains if supported. */
  default void setVelocityPID(double kP, double kI, double kD) {}

  /** Update the onboard velocity feedforward gains if supported. */
  default void setVelocityFF(double kS, double kV, double kA) {}

  /** Whether this IO uses onboard feedforward (kS/kV/kA) for velocity control. */
  default boolean usesVelocitySlotFeedforward() {
    return false;
  }

  /** Whether this IO supports onboard velocity control. */
  default boolean supportsVelocityControl() {
    return false;
  }

  /** Stop roller system */
  default void stop() {}

  /** Enable or disable brake mode. */
  default void setBrakeMode(boolean enabled) {}
}
