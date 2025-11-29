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

import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_GEAR_RATIO;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_INVERTED;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_MOTOR_GAINS;
import static frc.robot.subsystems.swerve.SwerveConstants.ROTATOR_ENCODER_INVERTED;
import static frc.robot.subsystems.swerve.SwerveConstants.ROTATOR_ENCODER_POSITION_FACTOR;
import static frc.robot.subsystems.swerve.SwerveConstants.ROTATOR_ENCODER_VELOCITY_FACTOR;
import static frc.robot.subsystems.swerve.SwerveConstants.ROTATOR_GAINS;
import static frc.robot.subsystems.swerve.SwerveConstants.ROTATOR_INVERTED;
import static frc.robot.subsystems.swerve.SwerveConstants.ROTATOR_MOTOR_CURRENT_LIMIT_AMPS;
import static frc.robot.subsystems.swerve.SwerveConstants.ROTATOR_PID_MAX_INPUT;
import static frc.robot.subsystems.swerve.SwerveConstants.ROTATOR_PID_MIN_INPUT;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/** Module IO implementation for Kraken X60 drive motors with Neo 550 turn motors. */
public class ModuleIOHalfSpark implements ModuleIO {
  private static final double TWO_PI = 2.0 * Math.PI;

  private final TalonFX driveMotor;
  private final SparkMax turnSpark;
  private final AbsoluteEncoder turnEncoder;
  private final SparkClosedLoopController turnController;
  private final Rotation2d zeroRotation;

  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveVoltage;
  private final StatusSignal<Current> driveCurrent;

  private final VoltageOut driveVoltageOut = new VoltageOut(0.0);
  private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0).withSlot(0);

  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public ModuleIOHalfSpark(ModuleConstants moduleConstants) {
    zeroRotation = moduleConstants.zeroRotation();

    driveMotor = new TalonFX(moduleConstants.driveID());
    turnSpark = new SparkMax(moduleConstants.rotatorID(), MotorType.kBrushless);
    turnEncoder = turnSpark.getAbsoluteEncoder();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor (Kraken X60)
    driveConfig.MotorOutput.Inverted =
        DRIVE_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_MOTOR_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = DRIVE_MOTOR_CURRENT_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.Slot0.kP = DRIVE_MOTOR_GAINS.kP();
    driveConfig.Slot0.kI = DRIVE_MOTOR_GAINS.kI();
    driveConfig.Slot0.kD = DRIVE_MOTOR_GAINS.kD();
    driveConfig.Slot0.kS = DRIVE_MOTOR_GAINS.kS();
    driveConfig.Slot0.kV = DRIVE_MOTOR_GAINS.kV();
    driveConfig.Slot0.kA = DRIVE_MOTOR_GAINS.kA();
    driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.setPosition(0.0);
    driveMotor.optimizeBusUtilization();

    // Configure status signals for drive motor
    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveVoltage = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(GlobalConstants.ODOMETRY_FREQUENCY, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveVoltage, driveCurrent);

    // Configure turn motor (Neo 550 on Spark Max)
    var turnConfig = new SparkMaxConfig();
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
        .pidf(ROTATOR_GAINS.kP(), 0.0, ROTATOR_GAINS.kD(), 0.0);
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
        SparkOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  BaseStatusSignal.refreshAll(drivePosition);
                  return drivePosition.getValueAsDouble() * TWO_PI;
                });
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    StatusCode driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveVoltage, driveCurrent);

    inputs.driveConnected = driveStatus.equals(StatusCode.OK);
    inputs.drivePositionRad = drivePosition.getValueAsDouble() * TWO_PI;
    inputs.driveVelocityRadPerSec = driveVelocity.getValueAsDouble() * TWO_PI;
    inputs.driveAppliedVolts = driveVoltage.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

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

    boolean sparkConnected = turnConnectedDebounce.calculate(!sparkStickyFault);
    inputs.turnConnected = sparkConnected;

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
    driveMotor.setControl(driveVoltageOut.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts =
        DRIVE_MOTOR_GAINS.kS() * Math.signum(velocityRadPerSec)
            + DRIVE_MOTOR_GAINS.kV() * velocityRadPerSec;
    double wheelRotationsPerSecond = velocityRadPerSec / TWO_PI;
    driveMotor.setControl(
        driveVelocityRequest.withVelocity(wheelRotationsPerSecond).withFeedForward(ffVolts));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpointRadians =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), ROTATOR_PID_MIN_INPUT, ROTATOR_PID_MAX_INPUT);
    turnController.setReference(setpointRadians, ControlType.kPosition);
  }
}
