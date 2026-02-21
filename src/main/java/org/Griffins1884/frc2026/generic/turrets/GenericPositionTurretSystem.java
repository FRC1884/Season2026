package org.Griffins1884.frc2026.generic.turrets;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class GenericPositionTurretSystem extends SubsystemBase {
  public enum ControlMode {
    CLOSED_LOOP,
    OPEN_LOOP
  }

  public record TurretConfig(
      LoggedTunableNumber kP,
      LoggedTunableNumber kI,
      LoggedTunableNumber kD,
      double positionToleranceRad,
      double maxVelocityRadPerSec,
      double maxAccelRadPerSec2,
      boolean softLimitsEnabled,
      double softLimitMinRad,
      double softLimitMaxRad,
      boolean useAbsoluteEncoder,
      double absoluteEncoderOffsetRad,
      int absoluteEncoderPort,
      LoggedTunableNumber absoluteSyncThresholdRad,
      double maxVoltage) {}

  private final String name;
  private final GenericTurretSystemIO io;
  protected final GenericTurretSystemIOInputsAutoLogged inputs =
      new GenericTurretSystemIOInputsAutoLogged();
  private final Alert disconnected;
  private final ProfiledPIDController controller;
  private final TurretConfig config;
  private final SysIdRoutine sysIdRoutine;
  private final int tuningId = System.identityHashCode(this);

  @Getter private ControlMode controlMode = ControlMode.CLOSED_LOOP;
  @Getter private double goalRad = 0.0;
  private double openLoopPercent = 0.0;
  private boolean initialized = false;
  private double zeroOffsetRad = 0.0;
  private boolean absoluteSynced = false;

  private DigitalInput input;
  private DutyCycleEncoder absEncoder;

  public GenericPositionTurretSystem(String name, GenericTurretSystemIO io, TurretConfig config) {
    this.name = name;
    this.io = io;
    this.config = config;
    controller =
        new ProfiledPIDController(
            config.kP().get(),
            config.kI().get(),
            config.kD().get(),
            new TrapezoidProfile.Constraints(
                config.maxVelocityRadPerSec(), config.maxAccelRadPerSec2()));
    controller.setTolerance(config.positionToleranceRad());
    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Seconds.of(4),
                state -> Logger.recordOutput(name + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> io.setVoltage(voltage.in(Volts)),
                (log) ->
                    log.motor(name)
                        .voltage(Volts.of(inputs.appliedVoltage))
                        .angularVelocity(RadiansPerSecond.of(inputs.velocityRadPerSec))
                        .angularPosition(Radian.of(inputs.positionRad)),
                this));
    if (config.useAbsoluteEncoder()) {
      input = new DigitalInput(config.absoluteEncoderPort());
      absEncoder = new DutyCycleEncoder(config.absoluteEncoderPort());
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    if (config.useAbsoluteEncoder() && absEncoder != null) {
      double absoluteRad = getAbsoluteEncoderRad();
      inputs.absoluteConnected = Double.isFinite(absoluteRad);
      inputs.absolutePositionRad = Double.isFinite(absoluteRad) ? absoluteRad : 0.0;
      maybeSyncAbsoluteEncoder(absoluteRad);
    }

    boolean anyDisconnected = false;
    for (boolean isConnected : inputs.connected) {
      if (!isConnected) {
        anyDisconnected = true;
        break;
      }
    }
    disconnected.set(anyDisconnected);

    double positionRad = getPositionRad();
    if (!initialized) {
      goalRad = positionRad;
      controller.reset(positionRad, getVelocityRadPerSec());
      initialized = true;
    }

    logOutputs(positionRad);

    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
      return;
    }

    if (controlMode == ControlMode.OPEN_LOOP) {
      double clampedPercent = clampOpenLoop(openLoopPercent, positionRad);
      io.setVoltage(clampedPercent * config.maxVoltage());
      return;
    }
    double kP = config.kP().get();
    double kI = config.kI().get();
    double kD = config.kD().get();
    if (io.usesInternalPositionControl()) {
      io.setPositionSetpoint(goalRad, kP, kI, kD);
      return;
    }
    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> controller.setPID(values[0], values[1], values[2]),
        config.kP(),
        config.kI(),
        config.kD());
    double pidOutput = controller.calculate(positionRad, goalRad);
    double outputVolts = MathUtil.clamp(pidOutput, -config.maxVoltage(), config.maxVoltage());
    io.setVoltage(outputVolts);
  }

  public void setGoalRad(double goalRad) {
    double clampedGoal = clampGoal(goalRad);
    if (controlMode != ControlMode.CLOSED_LOOP) {
      controller.reset(getPositionRad(), getVelocityRadPerSec());
    }
    controlMode = ControlMode.CLOSED_LOOP;
    this.goalRad = clampedGoal;
  }

  public void setOpenLoop(double percent) {
    openLoopPercent = MathUtil.clamp(percent, -1.0, 1.0);
    controlMode = ControlMode.OPEN_LOOP;
  }

  public void stopOpenLoop() {
    openLoopPercent = 0.0;
    controlMode = ControlMode.CLOSED_LOOP;
  }

  public boolean isAtGoal() {
    return Math.abs(goalRad - getPositionRad()) <= config.positionToleranceRad();
  }

  public double getPositionRad() {
    return getSensorPositionRad() + zeroOffsetRad;
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  public double getAppliedVolts() {
    return inputs.appliedVoltage;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getTorqueCurrentAmps() {
    return inputs.torqueCurrentAmps;
  }

  public double getAbsolutePositionRad() {
    return inputs.absolutePositionRad;
  }

  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  protected void enableContinuousInput(double minRad, double maxRad) {
    controller.enableContinuousInput(minRad, maxRad);
  }

  private double getSensorPositionRad() {
    if (config.useAbsoluteEncoder() && absEncoder != null) {
      double absoluteRad = getAbsoluteEncoderRad();
      if (Double.isFinite(absoluteRad)) {
        return absoluteRad;
      }
    }
    return inputs.positionRad;
  }

  private double getAbsoluteEncoderRad() {
    if (absEncoder == null) {
      return Double.NaN;
    }
    double rotations = absEncoder.get();
    if (!Double.isFinite(rotations)) {
      return Double.NaN;
    }
    double rad = (rotations * 2.0 * Math.PI) + config.absoluteEncoderOffsetRad();
    return MathUtil.inputModulus(rad, 0.0, 2.0 * Math.PI);
  }

  private void maybeSyncAbsoluteEncoder(double absoluteRad) {
    if (!Double.isFinite(absoluteRad)) {
      return;
    }
    double sensorRad = inputs.positionRad;
    double delta = MathUtil.inputModulus(absoluteRad - sensorRad, -Math.PI, Math.PI);
    double threshold =
        config.absoluteSyncThresholdRad() != null ? config.absoluteSyncThresholdRad().get() : 0.0;
    if (!absoluteSynced || Math.abs(delta) > threshold) {
      io.setPosition(absoluteRad);
      absoluteSynced = true;
      Logger.recordOutput(name + "/AbsoluteSyncDeltaRad", delta);
    }
  }

  private double clampGoal(double goalRad) {
    if (!config.softLimitsEnabled()) {
      return goalRad;
    }
    return MathUtil.clamp(goalRad, config.softLimitMinRad(), config.softLimitMaxRad());
  }

  private double clampOpenLoop(double percent, double positionRad) {
    double output = MathUtil.clamp(percent, -1.0, 1.0);
    if (!config.softLimitsEnabled()) {
      return output;
    }
    if (output > 0.0 && positionRad >= config.softLimitMaxRad()) {
      return 0.0;
    }
    if (output < 0.0 && positionRad <= config.softLimitMinRad()) {
      return 0.0;
    }
    return output;
  }

  private void logOutputs(double positionRad) {
    Logger.recordOutput(name + "/PositionRad", positionRad);
    Logger.recordOutput(name + "/GoalRad", goalRad);
    Logger.recordOutput(name + "/PositionRotations", positionRad / (2.0 * Math.PI));
    Logger.recordOutput(name + "/GoalRotations", goalRad / (2.0 * Math.PI));
    Logger.recordOutput(name + "/ErrorRad", goalRad - positionRad);
    Logger.recordOutput(name + "/AtGoal", isAtGoal());
    Logger.recordOutput(name + "/ControlMode", controlMode.toString());
    Logger.recordOutput(name + "/OpenLoopPercent", openLoopPercent);
    Logger.recordOutput(name + "/SetpointVelocityRadPerSec", controller.getSetpoint().velocity);
    Logger.recordOutput(name + "/MotorPositionRotations", inputs.motorPositionRotations);
    Logger.recordOutput(name + "/MotorPositionTicks", inputs.motorPositionTicks);
    Logger.recordOutput(name + "/MotorGoalRotations", inputs.motorGoalRotations);
    Logger.recordOutput(name + "/MotorGoalTicks", inputs.motorGoalTicks);
  }
}
