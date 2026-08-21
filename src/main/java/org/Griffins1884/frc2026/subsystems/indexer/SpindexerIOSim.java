package org.Griffins1884.frc2026.subsystems.indexer;

import edu.wpi.first.wpilibj.DriverStation;

public final class SpindexerIOSim implements SpindexerIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;
  private static final double RPM_TO_RAD_PER_SEC = 2.0 * Math.PI / 60.0;
  private double commandedRpm;
  private double appliedVoltage;
  private double positionRads;

  @Override
  public void updateInputs(MechanismRollerIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }
    double velocityRadsPerSec = commandedRpm * RPM_TO_RAD_PER_SEC;
    positionRads += velocityRadsPerSec * LOOP_PERIOD_SECONDS;
    inputs.connected = new boolean[] {true};
    inputs.positionRads = positionRads;
    inputs.velocity = commandedRpm;
    inputs.velocityRadsPerSec = velocityRadsPerSec;
    inputs.appliedVoltage = appliedVoltage;
  }

  @Override
  public void runVolts(double volts) {
    if (!Double.isFinite(volts) || volts == 0.0) {
      stop();
      return;
    }
    runVelocity(Math.copySign(SpindexerConstants.INDEX_RPM.get(), volts), 0.0);
  }

  @Override
  public void runVelocity(double velocityRpm, double feedforwardVolts) {
    if (!Double.isFinite(velocityRpm) || !Double.isFinite(feedforwardVolts)) {
      stop();
      return;
    }
    commandedRpm = velocityRpm;
    appliedVoltage =
        velocityRpm == 0.0 ? 0.0 : Math.copySign(SpindexerConstants.MAX_VOLTAGE, velocityRpm);
  }

  @Override
  public boolean supportsVelocityControl() {
    return true;
  }

  @Override
  public void stop() {
    commandedRpm = 0.0;
    appliedVoltage = 0.0;
  }
}
