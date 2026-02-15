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
import static org.Griffins1884.frc2026.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.ModuleConstants;
import org.Griffins1884.frc2026.util.PhoenixUtil;

/** Module IO implementation for Kraken X60 drive motors with Neo 550 turn motors. */
public class ModuleIOFullKraken implements ModuleIO {
  private static final double TWO_PI = 2.0 * Math.PI;

  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder turnEncoder;
  private final Rotation2d zeroRotation;
  private final Rotation2d encoderOffset;
  private final boolean hasCancoder;
  private static final double RADIANS_PER_ROTATION = TWO_PI;

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
    hasCancoder = moduleConstants.cancoderID() >= 0;
    encoderOffset = hasCancoder ? new Rotation2d() : zeroRotation;
    CANBus canBus = new CANBus("DriveTrain");

    driveMotor = new TalonFX(moduleConstants.driveID(), canBus);
    turnMotor = new TalonFX(moduleConstants.rotatorID(), canBus);
    turnEncoder = hasCancoder ? new CANcoder(moduleConstants.cancoderID()) : null;
    if (hasCancoder) {
      var cancoderConfig = new CANcoderConfiguration();
      // Match Mechanical Advantage: apply zeroRotation as the CANCoder magnet offset.
      cancoderConfig.MagnetSensor.MagnetOffset = moduleConstants.zeroRotation().getRotations();
      cancoderConfig.MagnetSensor.SensorDirection =
          moduleConstants.encoderInverted()
              ? SensorDirectionValue.Clockwise_Positive
              : SensorDirectionValue.CounterClockwise_Positive;
      tryUntilOk(5, () -> turnEncoder.getConfigurator().apply(cancoderConfig));
    }

    // Configure drive motor (Kraken X60)
    driveConfig.MotorOutput.Inverted =
        DRIVE_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = KRAKEN_DRIVE_CURRENT_LIMIT;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -KRAKEN_DRIVE_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimit = KRAKEN_DRIVE_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = KRAKEN_DRIVE_CURRENT_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    driveConfig.Slot0.kP = 0.0;
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = 0.0;
    driveConfig.Slot0.kS = 0.0;
    driveConfig.Slot0.kV = 0.0;
    driveConfig.Slot0.kA = 0.0;
    driveConfig.Feedback.SensorToMechanismRatio = KRAKEN_DRIVE_GEAR_RATIO;
    tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveMotor.setPosition(0.0));
    driveMotor.optimizeBusUtilization();

    // Configure turn motor (Kraken X44)
    turnConfig.MotorOutput.Inverted =
        moduleConstants.turnInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Feedback.RotorToSensorRatio = KRAKEN_ROTATOR_GEAR_RATIO;
    if (hasCancoder) {
      turnConfig.Feedback.FeedbackRemoteSensorID = moduleConstants.cancoderID();
      turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    } else {
      // Fall back to the integrated sensor if a CANCoder is not present
      turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    }
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = KRAKEN_ROTATOR_CURRENT_LIMIT_AMPS;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -KRAKEN_ROTATOR_CURRENT_LIMIT_AMPS;
    turnConfig.CurrentLimits.StatorCurrentLimit = KRAKEN_ROTATOR_CURRENT_LIMIT_AMPS;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.Slot0.kP = 0.0;
    turnConfig.Slot0.kI = 0.0;
    turnConfig.Slot0.kD = 0.0;
    turnConfig.Slot0.kS = 0.0;
    turnConfig.Slot0.kV = 0.0;
    turnConfig.Slot0.kA = 0.0;
    tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));
    if (!hasCancoder) {
      tryUntilOk(5, () -> turnMotor.setPosition(zeroRotation.getRotations()));
    }
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
    turnAbsolutePosition = hasCancoder ? turnEncoder.getAbsolutePosition() : null;
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
    if (turnAbsolutePosition != null) {
      BaseStatusSignal.setUpdateFrequencyForAll(
          GlobalConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition, turnAbsolutePosition);
    } else {
      BaseStatusSignal.setUpdateFrequencyForAll(
          GlobalConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    }
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
    if (hasCancoder) {
      tryUntilOk(
          5, () -> ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor, turnEncoder));
    } else {
      tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor));
    }

    // Register signals for refresh
    if (turnAbsolutePosition != null) {
      PhoenixUtil.registerSignals(
          true, // Default to the RIO bus; swap to true if you move these onto a CANivore
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
    } else {
      PhoenixUtil.registerSignals(
          true,
          drivePosition,
          driveVelocity,
          driveAppliedVolts,
          driveSupplyCurrentAmps,
          driveTorqueCurrentAmps,
          turnPosition,
          turnVelocity,
          turnAppliedVolts,
          turnSupplyCurrentAmps,
          turnTorqueCurrentAmps);
    }
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

    inputs.turnConnected = turnStatus.equals(StatusCode.OK);
    inputs.turnPosition =
        new Rotation2d(turnPosition.getValueAsDouble() * RADIANS_PER_ROTATION).minus(encoderOffset);
    inputs.turnVelocityRadPerSec = turnVelocity.getValueAsDouble() * TWO_PI;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnSupplyCurrentAmps.getValueAsDouble();
    inputs.turnConnected &= turnMotor.isConnected();

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value * TWO_PI).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map(
                (Double value) -> new Rotation2d(value * RADIANS_PER_ROTATION).minus(encoderOffset))
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
    setDriveVelocity(velocityRadPerSec, 0.0);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double feedforward) {
    double wheelRotationsPerSecond = velocityRadPerSec / TWO_PI;
    driveMotor.setControl(
        velocityTorqueCurrentRequest
            .withVelocity(wheelRotationsPerSecond)
            .withFeedForward(feedforward));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnMotor.setControl(
        positionTorqueCurrentRequest.withPosition(rotation.plus(encoderOffset).getRotations()));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveConfig.Slot0.kP = kP;
    driveConfig.Slot0.kI = kI;
    driveConfig.Slot0.kD = kD;
    tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnConfig.Slot0.kP = kP;
    turnConfig.Slot0.kI = kI;
    turnConfig.Slot0.kD = kD;
    tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    brakeModeExecutor.execute(
        () -> {
          synchronized (driveConfig) {
            driveConfig.MotorOutput.NeutralMode =
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
          }
        });
    brakeModeExecutor.execute(
        () -> {
          synchronized (turnConfig) {
            turnConfig.MotorOutput.NeutralMode =
                enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));
          }
        });
  }

  @Override
  public void addOrchestraInstruments(List<TalonFX> instruments) {
    if (instruments == null) {
      return;
    }
    instruments.add(driveMotor);
    instruments.add(turnMotor);
  }
}
