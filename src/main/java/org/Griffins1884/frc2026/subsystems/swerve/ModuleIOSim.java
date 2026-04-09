package org.Griffins1884.frc2026.subsystems.swerve;

import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.DRIVE_MOTOR_GAINS;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.ROTATOR_GAINS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import org.Griffins1884.frc2026.simulation.physics.LocalSwervePhysicsSimulation;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

/** Local deterministic module simulation implementation. */
public class ModuleIOSim implements ModuleIO {
  private final LocalSwervePhysicsSimulation simulation;
  private final int moduleIndex;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController =
      new PIDController(DRIVE_MOTOR_GAINS.kP().get(), 0, DRIVE_MOTOR_GAINS.kD().get());
  private final PIDController turnController =
      new PIDController(ROTATOR_GAINS.kP().get(), 0, ROTATOR_GAINS.kD().get());
  private final int tuningId = System.identityHashCode(this);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(LocalSwervePhysicsSimulation simulation, int moduleIndex) {
    this.simulation = simulation;
    this.moduleIndex = moduleIndex;
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> driveController.setPID(values[0], values[1], values[2]),
        DRIVE_MOTOR_GAINS.kP(),
        DRIVE_MOTOR_GAINS.kI(),
        DRIVE_MOTOR_GAINS.kD());
    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> turnController.setPID(values[0], values[1], values[2]),
        ROTATOR_GAINS.kP(),
        ROTATOR_GAINS.kI(),
        ROTATOR_GAINS.kD());

    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(simulation.getDriveVelocityRadPerSec(moduleIndex));
      simulation.setDriveVelocity(moduleIndex, driveController.getSetpoint(), driveAppliedVolts);
    } else {
      driveController.reset();
      simulation.setDriveOpenLoop(moduleIndex, driveAppliedVolts);
    }
    if (turnClosedLoop) {
      turnAppliedVolts =
          turnController.calculate(simulation.getTurnPosition(moduleIndex).getRadians());
      simulation.setTurnPosition(moduleIndex, Rotation2d.fromRadians(turnController.getSetpoint()));
    } else {
      turnController.reset();
      simulation.setTurnOpenLoop(moduleIndex, turnAppliedVolts);
    }

    inputs.driveConnected = true;
    inputs.drivePositionRad = simulation.getDrivePositionRad(moduleIndex);
    inputs.driveVelocityRadPerSec = simulation.getDriveVelocityRadPerSec(moduleIndex);
    inputs.driveAppliedVolts = simulation.getDriveAppliedVolts(moduleIndex);
    inputs.driveCurrentAmps = simulation.getDriveCurrentAmps(moduleIndex);

    inputs.turnConnected = true;
    inputs.turnPosition = simulation.getTurnPosition(moduleIndex);
    inputs.turnAbsolutePosition = inputs.turnPosition;
    inputs.turnPositionRotations = simulation.getTurnPositionRotations(moduleIndex);
    inputs.turnAbsolutePositionRotations = inputs.turnPositionRotations;
    inputs.turnZeroTrimRotations = simulation.getZeroTrimRotations(moduleIndex);
    inputs.turnVelocityRadPerSec = simulation.getTurnVelocityRadPerSec(moduleIndex);
    inputs.turnAppliedVolts = simulation.getTurnAppliedVolts(moduleIndex);
    inputs.turnCurrentAmps = simulation.getTurnCurrentAmps(moduleIndex);

    inputs.odometryTimestamps = simulation.getCachedTimestamps();
    inputs.odometryDrivePositionsRad = simulation.getCachedDrivePositionsRad(moduleIndex);
    inputs.odometryTurnPositions = simulation.getCachedTurnPositions(moduleIndex);
    inputs.odometryTurnPositionsRotations = simulation.getCachedTurnPositionsRotations(moduleIndex);
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts =
        DRIVE_MOTOR_GAINS.kS().get() * Math.signum(velocityRadPerSec)
            + DRIVE_MOTOR_GAINS.kV().get() * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }

  @Override
  public void captureZeroTrim() {
    simulation.captureZeroTrim(moduleIndex);
  }

  @Override
  public void clearZeroTrim() {
    simulation.clearZeroTrim(moduleIndex);
  }
}
