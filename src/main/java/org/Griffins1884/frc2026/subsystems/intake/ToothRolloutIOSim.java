package org.Griffins1884.frc2026.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

/** Deterministic no-hardware ToothRollout IO for tests and SIMBOT simulation. */
public final class ToothRolloutIOSim implements ToothRolloutIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double SYNTHETIC_RADIANS_PER_SECOND_PER_VOLT = 1.0;

  private double appliedVoltage;
  private double positionRads;

  @Override
  public void updateInputs(MechanismRollerIOInputs inputs) {
    double velocityRadsPerSec = appliedVoltage * SYNTHETIC_RADIANS_PER_SECOND_PER_VOLT;
    positionRads += velocityRadsPerSec * LOOP_PERIOD_SECONDS;

    inputs.connected = new boolean[] {true};
    inputs.positionRads = positionRads;
    inputs.velocityRadsPerSec = velocityRadsPerSec;
    inputs.velocity = Units.radiansPerSecondToRotationsPerMinute(velocityRadsPerSec);
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = 0.0;
    inputs.torqueCurrentAmps = 0.0;
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage =
        Double.isFinite(volts)
            ? MathUtil.clamp(
                volts,
                -ToothRolloutConstants.SIMULATION_MAX_OUTPUT_VOLTS,
                ToothRolloutConstants.SIMULATION_MAX_OUTPUT_VOLTS)
            : 0.0;
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
  }
}
