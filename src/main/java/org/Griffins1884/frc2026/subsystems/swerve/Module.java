// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.Griffins1884.frc2026.subsystems.swerve;

import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.List;
import lombok.Getter;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.GlobalConstants.RobotSwerveMotors;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber krakenDrivekS =
      new LoggedTunableNumber("Drive/Module/DrivekS");
  private static final LoggedTunableNumber krakenDrivekV =
      new LoggedTunableNumber("Drive/Module/DrivekV");
  private static final LoggedTunableNumber krakenDrivekT =
      new LoggedTunableNumber("Drive/Module/DrivekT");
  private static final LoggedTunableNumber krakenDrivekP =
      new LoggedTunableNumber("Drive/Module/DrivekP");
  private static final LoggedTunableNumber krakenDrivekD =
      new LoggedTunableNumber("Drive/Module/DrivekD");
  private static final LoggedTunableNumber krakenTurnkP =
      new LoggedTunableNumber("Drive/Module/TurnkP");
  private static final LoggedTunableNumber krakenTurnkD =
      new LoggedTunableNumber("Drive/Module/TurnkD");

  static {
    krakenDrivekS.initDefault(KRAKEN_DRIVE_TORQUE_GAINS.kS().get());
    krakenDrivekV.initDefault(KRAKEN_DRIVE_TORQUE_GAINS.kV().get());
    krakenDrivekT.initDefault(
        SwerveConstants.KRAKEN_DRIVE_GEAR_RATIO / DCMotor.getKrakenX60Foc(1).KtNMPerAmp);
    krakenDrivekP.initDefault(KRAKEN_DRIVE_TORQUE_GAINS.kP().get());
    krakenDrivekD.initDefault(KRAKEN_DRIVE_TORQUE_GAINS.kD().get());
    krakenTurnkP.initDefault(KRAKEN_TURN_TORQUE_GAINS.kP().get());
    krakenTurnkD.initDefault(KRAKEN_TURN_TORQUE_GAINS.kD().get());
  }

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private SimpleMotorFeedforward krakenFfModel =
      new SimpleMotorFeedforward(krakenDrivekS.get(), krakenDrivekV.get());

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;

  /** -- GETTER -- Returns the module positions received this cycle. */
  @Getter private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on module " + index + ".", AlertType.kError);
  }

  public void addOrchestraInstruments(List<TalonFX> instruments) {
    if (instruments == null) {
      return;
    }
    io.addOrchestraInstruments(instruments);
  }

  public void periodic() {
    if (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS) {
      if (krakenDrivekS.hasChanged(hashCode()) || krakenDrivekV.hasChanged(hashCode())) {
        krakenFfModel = new SimpleMotorFeedforward(krakenDrivekS.get(), krakenDrivekV.get());
      }
      if (krakenDrivekP.hasChanged(hashCode()) || krakenDrivekD.hasChanged(hashCode())) {
        io.setDrivePID(krakenDrivekP.get(), 0.0, krakenDrivekD.get());
      }
      if (krakenTurnkP.hasChanged(hashCode()) || krakenTurnkD.hasChanged(hashCode())) {
        io.setTurnPID(krakenTurnkP.get(), 0.0, krakenTurnkD.get());
      }
    }

    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + index, inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    if (GlobalConstants.robotSwerveMotors == RobotSwerveMotors.FULLKRACKENS) {
      // Mechanical Advantage-style control for full Kraken modules
      double speedRadPerSec = state.speedMetersPerSecond / WHEEL_RADIUS;
      io.setDriveVelocity(speedRadPerSec, krakenFfModel.calculate(speedRadPerSec));
      if (Math.abs(state.angle.minus(getAngle()).getDegrees()) < TURN_DEADBAND_DEGREES) {
        io.setTurnOpenLoop(0.0);
      } else {
        io.setTurnPosition(state.angle);
      }
      return;
    }

    // Default (Spark/HalfSpark) control path
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / WHEEL_RADIUS);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zeroRotation degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }

  /** Runs a steer-only SysId sweep while keeping the drive stage disabled. */
  public void runTurnCharacterization(double output) {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(output);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * WHEEL_RADIUS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  public double getVoltage() {
    return inputs.driveAppliedVolts;
  }

  public double getDriveVoltage() {
    return inputs.driveAppliedVolts;
  }

  public double getTurnVoltage() {
    return inputs.turnAppliedVolts;
  }

  public double getTurnPositionRad() {
    return inputs.turnPosition.getRadians();
  }

  public double getTurnVelocityRadPerSec() {
    return inputs.turnVelocityRadPerSec;
  }
}
