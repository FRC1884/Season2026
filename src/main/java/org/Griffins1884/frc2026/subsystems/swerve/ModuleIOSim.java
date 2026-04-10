package org.Griffins1884.frc2026.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import org.Griffins1884.frc2026.simulation.drive.Season2026DriveSimulation;
import org.Griffins1884.frc2026.simulation.drive.Season2026SwerveCorner;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
import org.Griffins1884.frc2026.util.SparkUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation moduleSimulation;
  private final Season2026DriveSimulation terrainSimulation;
  private final Season2026SwerveCorner tractionCorner;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

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
  private double lastDriveAuthorityScale = 1.0;
  private double lastTurnAuthorityScale = 1.0;
  private double lastSupportedDriveAuthorityScale = 1.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this(null, null, moduleSimulation);
  }

  public ModuleIOSim(
      Season2026DriveSimulation terrainSimulation,
      Season2026SwerveCorner tractionCorner,
      SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.terrainSimulation = terrainSimulation;
    this.tractionCorner = tractionCorner;
    this.driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(DRIVE_MOTOR_CURRENT_LIMIT));
    this.turnMotor =
        moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(ROTATOR_MOTOR_CURRENT_LIMIT_AMPS));

    // Enable wrapping for turn PID
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
    // Run closed-loop control
    if (driveClosedLoop) {
      lastDriveAuthorityScale = tractionDriveScale();
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(
                  moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
      driveAppliedVolts *= lastDriveAuthorityScale;
    } else {
      driveController.reset();
      lastDriveAuthorityScale = 1.0;
    }
    if (turnClosedLoop) {
      lastTurnAuthorityScale = turnAuthorityScale();
      turnAppliedVolts =
          turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians())
              * lastTurnAuthorityScale;
    } else {
      turnController.reset();
      lastTurnAuthorityScale = 1.0;
    }

    // Update simulation state
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));
    inputs.terrainDriveAuthorityScale = lastDriveAuthorityScale;

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));
    inputs.terrainTurnAuthorityScale = lastTurnAuthorityScale;

    // Update odometry inputs
    inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
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

  private double tractionDriveScale() {
    if (DriverStation.isAutonomousEnabled()) {
      return 1.0;
    }
    if (terrainSimulation == null || tractionCorner == null) {
      return 1.0;
    }
    double driveScale = terrainSimulation.driveAuthorityScale(tractionCorner);
    var supportDiagnostics = terrainSimulation.supportSnapshot();
    if (supportDiagnostics.supportContactCount() > 0 && driveScale > 1e-6) {
      lastSupportedDriveAuthorityScale = driveScale;
      return driveScale;
    }
    if (supportDiagnostics.actualAirborne() || supportDiagnostics.supportContactCount() == 0) {
      return lastSupportedDriveAuthorityScale;
    }
    return driveScale;
  }

  private double turnAuthorityScale() {
    if (DriverStation.isAutonomousEnabled()) {
      return 1.0;
    }
    if (terrainSimulation == null || tractionCorner == null) {
      return 1.0;
    }
    return terrainSimulation.steerAuthorityScale(tractionCorner);
  }
}
