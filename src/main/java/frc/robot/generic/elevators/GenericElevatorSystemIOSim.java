package frc.robot.generic.elevators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class GenericElevatorSystemIOSim implements GenericElevatorSystemIO {
  private ElevatorSim sim;

  private double appliedVolts = 0.0;
  private double positionOffset = 0.0;

  public GenericElevatorSystemIOSim(int numMotors, double startingAngle) {
    sim =
        new ElevatorSim(
            DCMotor.getNeoVortex(numMotors),
            (10 / Units.metersToInches(0.012) / 0.5),
            1,
            0.3126232,
            0,
            Units.degreesToRadians(110),
            true,
            startingAngle);
  }

  @Override
  public void updateInputs(GenericElevatorSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);
    inputs.encoderPosition = sim.getPositionMeters() + positionOffset;
    inputs.velocity = sim.getVelocityMetersPerSecond();
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = inputs.supplyCurrentAmps;
    if (inputs.connected.length != 1) {
      inputs.connected = new boolean[] {true};
    } else {
      inputs.connected[0] = true;
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPosition(double position) {
    positionOffset = position - sim.getPositionMeters();
  }
}
