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

package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.SwerveConstants.*;
import frc.robot.util.PhoenixUtil;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

/** Module IO implementation for Kraken X60 drive motors with Neo 550 turn motors. */
public class ModuleIOFullKraken implements ModuleIO {
  private static final double TWO_PI = 2.0 * Math.PI;

  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder turnEncoder;
  private final Rotation2d zeroRotation;

  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
  private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

  // Control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0);

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrentAmps;
  private final StatusSignal<Current> driveTorqueCurrentAmps;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnSupplyCurrentAmps;
  private final StatusSignal<Current> turnTorqueCurrentAmps;

  // TimeStamp Queue
  private final Queue<Double> timestampQueue;

  public ModuleIOFullKraken(ModuleConstants moduleConstants) {
    zeroRotation = moduleConstants.zeroRotation();

    driveMotor = new TalonFX(moduleConstants.driveID());
    turnMotor = new TalonFX(moduleConstants.rotatorID());
    turnEncoder = new CANcoder(moduleConstants.cancoderID());
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = moduleConstants.zeroRotation().getRadians();
    cancoderConfig.MagnetSensor.SensorDirection =
        moduleConstants.encoderInverted()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnEncoder.getConfigurator().apply(cancoderConfig));

    // Configure drive motor (Kraken X60)
    driveConfig.MotorOutput.Inverted =
        DRIVE_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = DRIVE_MOTOR_CURRENT_LIMIT;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DRIVE_MOTOR_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_MOTOR_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    driveConfig.Slot0.kP = DRIVE_MOTOR_GAINS.kP();
    driveConfig.Slot0.kI = DRIVE_MOTOR_GAINS.kI();
    driveConfig.Slot0.kD = DRIVE_MOTOR_GAINS.kD();
    driveConfig.Slot0.kS = DRIVE_MOTOR_GAINS.kS();
    driveConfig.Slot0.kV = DRIVE_MOTOR_GAINS.kV();
    driveConfig.Slot0.kA = DRIVE_MOTOR_GAINS.kA();
    driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
    tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveMotor.setPosition(0.0));
    driveMotor.optimizeBusUtilization();

    // Configure drive motor (Kraken X44)
    turnConfig.MotorOutput.Inverted =
        moduleConstants.turnInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Feedback.FeedbackRemoteSensorID = moduleConstants.cancoderID();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.RotorToSensorRatio = ROTATOR_GEAR_RATIO;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = ROTATOR_MOTOR_CURRENT_LIMIT_AMPS;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -ROTATOR_MOTOR_CURRENT_LIMIT_AMPS;
    turnConfig.CurrentLimits.StatorCurrentLimit = ROTATOR_MOTOR_CURRENT_LIMIT_AMPS;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.Slot0.kP = ROTATOR_GAINS.kP();
    turnConfig.Slot0.kI = ROTATOR_GAINS.kI();
    turnConfig.Slot0.kD = ROTATOR_GAINS.kD();
    turnConfig.Slot0.kS = ROTATOR_GAINS.kS();
    turnConfig.Slot0.kV = ROTATOR_GAINS.kV();
    turnConfig.Slot0.kA = ROTATOR_GAINS.kA();
    tryUntilOk(5, () -> turnMotor.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> turnMotor.setPosition(0.0));
    turnMotor.optimizeBusUtilization();
    // Create drive status signals
    drivePosition = driveMotor.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getPosition().clone());
    driveVelocity = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    driveSupplyCurrentAmps = driveMotor.getSupplyCurrent();
    driveTorqueCurrentAmps = driveMotor.getTorqueCurrent();

    // Create turn status signals
    turnAbsolutePosition = turnEncoder.getAbsolutePosition();
    turnPosition = turnMotor.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnMotor.getPosition().clone());
    turnVelocity = turnMotor.getVelocity();
    turnAppliedVolts = turnMotor.getMotorVoltage();
    turnSupplyCurrentAmps = turnMotor.getSupplyCurrent();
    turnTorqueCurrentAmps = turnMotor.getTorqueCurrent();

    // Create odometry queues
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        GlobalConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition, turnAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps);
    tryUntilOk(
        5, () -> ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor, turnEncoder));

    // Register signals for refresh
    PhoenixUtil.registerSignals(
        true,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        turnPosition,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update Drive inputs
    StatusCode driveStatus =
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrentAmps,
            driveTorqueCurrentAmps);

    inputs.driveConnected = driveStatus.equals(StatusCode.OK);
    inputs.drivePositionRad = drivePosition.getValueAsDouble() * TWO_PI;
    inputs.driveVelocityRadPerSec = driveVelocity.getValueAsDouble() * TWO_PI;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveSupplyCurrentAmps.getValueAsDouble();
    inputs.turnConnected = turnMotor.isConnected();

    // Update Turn inputs
    StatusCode turnStatus =
        BaseStatusSignal.refreshAll(
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnSupplyCurrentAmps,
            turnTorqueCurrentAmps);

    inputs.turnConnected = driveStatus.equals(StatusCode.OK);
    inputs.turnPosition = new Rotation2d(turnPosition.getValueAsDouble()).minus(zeroRotation);
    inputs.turnVelocityRadPerSec = turnVelocity.getValueAsDouble() * TWO_PI;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnSupplyCurrentAmps.getValueAsDouble();
    inputs.turnConnected = turnMotor.isConnected();

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

    if (inputs.odometryTimestamps.length == 0) {
      double timestamp = Timer.getFPGATimestamp();
      inputs.odometryTimestamps = new double[] {timestamp};
      inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
      inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
    }
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveMotor.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnMotor.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts =
        DRIVE_MOTOR_GAINS.kS() * Math.signum(velocityRadPerSec)
            + DRIVE_MOTOR_GAINS.kV() * velocityRadPerSec;
    driveMotor.setControl(
        velocityTorqueCurrentRequest
            .withVelocity(Units.radiansToRotations(velocityRadPerSec))
            .withFeedForward(ffVolts));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnMotor.setControl(positionTorqueCurrentRequest.withPosition(rotation.getRotations()));
  }
}
