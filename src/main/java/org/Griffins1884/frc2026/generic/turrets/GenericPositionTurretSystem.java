package org.Griffins1884.frc2026.generic.turrets;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class GenericPositionTurretSystem extends SubsystemBase {
  public enum ControlMode {
    CLOSED_LOOP,
    OPEN_LOOP
  }

  public record TurretConfig(
      GlobalConstants.Gains gains,
      double positionToleranceRad,
      double maxVelocityRadPerSec,
      double maxAccelRadPerSec2,
      boolean softLimitsEnabled,
      double softLimitMinRad,
      double softLimitMaxRad,
      boolean useAbsoluteEncoder,
      double absoluteEncoderOffsetRad,
      double maxVoltage) {}

  private final String name;
  private final GenericTurretSystemIO io;
  protected final GenericTurretSystemIOInputsAutoLogged inputs =
      new GenericTurretSystemIOInputsAutoLogged();
  private final Alert disconnected;
  private final ProfiledPIDController controller;
  private SimpleMotorFeedforward feedforward;
  private final TurretConfig config;
  private final SysIdRoutine sysIdRoutine;
  private final int tuningId = System.identityHashCode(this);

  private ControlMode controlMode = ControlMode.CLOSED_LOOP;
  private double goalRad = 0.0;
  private double openLoopPercent = 0.0;
  private boolean initialized = false;
  private double zeroOffsetRad = 0.0;

  public GenericPositionTurretSystem(String name, GenericTurretSystemIO io, TurretConfig config) {
    this.name = name;
    this.io = io;
    this.config = config;
    controller =
        new ProfiledPIDController(
            config.gains().kP().get(),
            config.gains().kI().get(),
            config.gains().kD().get(),
            new TrapezoidProfile.Constraints(
                config.maxVelocityRadPerSec(), config.maxAccelRadPerSec2()));
    controller.setTolerance(config.positionToleranceRad());
    feedforward =
        new SimpleMotorFeedforward(
            config.gains().kS().get(), config.gains().kV().get(), config.gains().kA().get());
    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Seconds.of(4),
                state -> Logger.recordOutput(name + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(voltage -> io.setVoltage(voltage.in(Volts)), null, this));
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

    goalRad = clampGoal(goalRad);
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
    double kP = config.gains().kP().get();
    double kI = config.gains().kI().get();
    double kD = config.gains().kD().get();
    if (io.usesInternalPositionControl()) {
      io.setPositionSetpoint(goalRad, kP, kI, kD);
      return;
    }
    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> controller.setPID(values[0], values[1], values[2]),
        config.gains().kP(),
        config.gains().kI(),
        config.gains().kD());
    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> feedforward = new SimpleMotorFeedforward(values[0], values[1], values[2]),
        config.gains().kS(),
        config.gains().kV(),
        config.gains().kA());

    double pidOutput = controller.calculate(positionRad, goalRad);
    double ffOutput = feedforward.calculate(controller.getSetpoint().velocity);
    double outputVolts =
        MathUtil.clamp(pidOutput + ffOutput, -config.maxVoltage(), config.maxVoltage());
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

  public double getGoalRad() {
    return goalRad;
  }

  public void setOpenLoop(double percent) {
    openLoopPercent = MathUtil.clamp(percent, -1.0, 1.0);
    controlMode = ControlMode.OPEN_LOOP;
  }

  public void stopOpenLoop() {
    openLoopPercent = 0.0;
    setGoalRad(getPositionRad());
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

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  protected void enableContinuousInput(double minRad, double maxRad) {
    controller.enableContinuousInput(minRad, maxRad);
  }

  public void zeroPosition() {
    double sensorPosition = getSensorPositionRad();
    zeroOffsetRad = -sensorPosition;
    openLoopPercent = 0.0;
    controlMode = ControlMode.CLOSED_LOOP;
    goalRad = 0.0;
    controller.reset(0.0, 0.0);
    initialized = true;
    if (!(config.useAbsoluteEncoder() && inputs.absoluteConnected)) {
      io.setPosition(0.0);
    }
  }

  private double getSensorPositionRad() {
    if (config.useAbsoluteEncoder() && inputs.absoluteConnected) {
      return inputs.absolutePositionRad + config.absoluteEncoderOffsetRad();
    }
    return inputs.positionRad;
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
    Logger.recordOutput(name + "/ErrorRad", goalRad - positionRad);
    Logger.recordOutput(name + "/AtGoal", isAtGoal());
    Logger.recordOutput(name + "/ControlMode", controlMode.toString());
    Logger.recordOutput(name + "/OpenLoopPercent", openLoopPercent);
    Logger.recordOutput(name + "/SetpointVelocityRadPerSec", controller.getSetpoint().velocity);
  }
}
