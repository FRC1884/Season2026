package org.Griffins1884.frc2026.mechanisms.rollers;

import static edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MechanismRollerIOSim implements MechanismRollerIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;

  public MechanismRollerIOSim(DCMotor motorModel, double reduction, double moi) {
    sim = new DCMotorSim(createDCMotorSystem(motorModel, reduction, moi), motorModel);
  }

  @Override
  public void updateInputs(MechanismRollerIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(appliedVoltage);
    }

    sim.update(0.02);
    if (inputs.connected.length != 1) {
      inputs.connected = new boolean[] {true};
    } else {
      inputs.connected[0] = true;
    }
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.velocity = sim.getAngularVelocityRPM();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = inputs.supplyCurrentAmps;
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
