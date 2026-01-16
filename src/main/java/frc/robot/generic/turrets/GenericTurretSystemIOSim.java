package frc.robot.generic.turrets;

import static edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class GenericTurretSystemIOSim implements GenericTurretSystemIO {
  private final DCMotorSim sim;
  private double appliedVolts = 0.0;
  private double positionOffsetRad = 0.0;

  public GenericTurretSystemIOSim(DCMotor motorModel, double gearRatio, double moi) {
    sim = new DCMotorSim(createDCMotorSystem(motorModel, gearRatio, moi), motorModel);
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
    inputs.positionRad = sim.getAngularPositionRad() + positionOffsetRad;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
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
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPosition(double positionRad) {
    positionOffsetRad = positionRad - sim.getAngularPositionRad();
  }
}
