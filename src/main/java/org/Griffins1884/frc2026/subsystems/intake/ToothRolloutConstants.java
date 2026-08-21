package org.Griffins1884.frc2026.subsystems.intake;

import org.Griffins1884.frc2026.mechanisms.MechanismDefinition;

public final class ToothRolloutConstants {
  /** Schema-only identity for the deterministic simulation path; this is not a CAN allocation. */
  public static final int SIMULATION_DEVICE_ID = 0;

  public static final MechanismDefinition.MotorControllerType SIMULATION_CONTROLLER =
      MechanismDefinition.MotorControllerType.SIMULATION_ONLY;
  public static final double SIMULATION_REDUCTION = 1.0;
  public static final double SIMULATION_MAX_OUTPUT_VOLTS = 1.0;
  public static final double INTAKE_FEED_SIMULATION_VOLTS = 1.0;
  public static final double EJECT_REVERSE_SIMULATION_VOLTS = -1.0;

  private ToothRolloutConstants() {}
}
