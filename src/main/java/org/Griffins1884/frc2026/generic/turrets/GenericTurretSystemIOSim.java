package org.Griffins1884.frc2026.generic.turrets;

import static edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class GenericTurretSystemIOSim implements GenericTurretSystemIO {
  private final DCMotorSim sim;
  private final double invertSign;
  private double appliedVolts = 0.0;
  private double positionOffsetRad = 0.0;

  public GenericTurretSystemIOSim(DCMotor motorModel, double gearRatio, double moi) {
    this(motorModel, gearRatio, moi, false);
  }

  public GenericTurretSystemIOSim(
      DCMotor motorModel, double gearRatio, double moi, boolean inverted) {
    sim = new DCMotorSim(createDCMotorSystem(motorModel, gearRatio, moi), motorModel);
    invertSign = inverted ? -1.0 : 1.0;
  }

  @Override
  public void updateInputs(GenericTurretSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(appliedVolts);
    }
    sim.update(0.02);
    if (inputs.connected.length != 1) {
      inputs.connected = new boolean[] {true};
    } else {
      inputs.connected[0] = true;
    }
    double rawPositionRad = sim.getAngularPositionRad() * invertSign;
    inputs.positionRad = rawPositionRad + positionOffsetRad;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec() * invertSign;
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = inputs.supplyCurrentAmps;
    inputs.tempCelsius = 0.0;
    inputs.absoluteConnected = false;
    inputs.absolutePositionRad = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts * invertSign);
  }

  @Override
  public void setPosition(double positionRad) {
    positionOffsetRad = positionRad - sim.getAngularPositionRad() * invertSign;
  }
}
