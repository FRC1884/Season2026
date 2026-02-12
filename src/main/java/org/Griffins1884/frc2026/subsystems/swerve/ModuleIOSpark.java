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

import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.DRIVE_ENCODER_POSITION_FACTOR;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.DRIVE_ENCODER_VELOCITY_FACTOR;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.DRIVE_INVERTED;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.DRIVE_MOTOR_GAINS;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.ROTATOR_ENCODER_INVERTED;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.ROTATOR_ENCODER_POSITION_FACTOR;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.ROTATOR_ENCODER_VELOCITY_FACTOR;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.ROTATOR_GAINS;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.ROTATOR_INVERTED;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.ROTATOR_MOTOR_CURRENT_LIMIT_AMPS;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.ROTATOR_PID_MAX_INPUT;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.ROTATOR_PID_MIN_INPUT;
import static org.Griffins1884.frc2026.util.SparkUtil.ifOk;
import static org.Griffins1884.frc2026.util.SparkUtil.sparkStickyFault;
import static org.Griffins1884.frc2026.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.ModuleConstants;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;
  private final SparkFlexConfig driveConfig = new SparkFlexConfig();
  private final SparkMaxConfig turnConfig = new SparkMaxConfig();
  private final int tuningId = System.identityHashCode(this);

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIOSpark(ModuleConstants moduleConstants) {
    zeroRotation = moduleConstants.zeroRotation();
    driveSpark = new SparkFlex(moduleConstants.driveID(), MotorType.kBrushless);
    turnSpark = new SparkMax(moduleConstants.rotatorID(), MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getAbsoluteEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    driveConfig
        .inverted(DRIVE_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            DRIVE_MOTOR_GAINS.kP().get(),
            DRIVE_MOTOR_GAINS.kI().get(),
            DRIVE_MOTOR_GAINS.kD().get());
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / GlobalConstants.ODOMETRY_FREQUENCY))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    turnConfig
        .inverted(ROTATOR_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ROTATOR_MOTOR_CURRENT_LIMIT_AMPS)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(ROTATOR_ENCODER_INVERTED)
        .positionConversionFactor(ROTATOR_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(ROTATOR_ENCODER_VELOCITY_FACTOR)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(ROTATOR_PID_MIN_INPUT, ROTATOR_PID_MAX_INPUT)
        .pid(ROTATOR_GAINS.kP().get(), ROTATOR_GAINS.kI().get(), ROTATOR_GAINS.kD().get());
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / GlobalConstants.ODOMETRY_FREQUENCY))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    updateTunableGains();
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
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
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts =
        DRIVE_MOTOR_GAINS.kS().get() * Math.signum(velocityRadPerSec)
            + DRIVE_MOTOR_GAINS.kV().get() * velocityRadPerSec;
    driveController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), ROTATOR_PID_MIN_INPUT, ROTATOR_PID_MAX_INPUT);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }

  private void updateTunableGains() {
    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> {
          driveConfig.closedLoop.pid(values[0], values[1], values[2]);
          tryUntilOk(
              driveSpark,
              5,
              () ->
                  driveSpark.configure(
                      driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        },
        DRIVE_MOTOR_GAINS.kP(),
        DRIVE_MOTOR_GAINS.kI(),
        DRIVE_MOTOR_GAINS.kD());
    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> {
          turnConfig.closedLoop.pid(values[0], values[1], values[2]);
          tryUntilOk(
              turnSpark,
              5,
              () ->
                  turnSpark.configure(
                      turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        },
        ROTATOR_GAINS.kP(),
        ROTATOR_GAINS.kI(),
        ROTATOR_GAINS.kD());
  }
}
