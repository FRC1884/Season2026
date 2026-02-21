package org.Griffins1884.frc2026.generic.elevators;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public abstract class GenericPositionElevatorSystem<
        G extends GenericPositionElevatorSystem.ExtensionGoal>
    extends SubsystemBase {
  public enum ControlMode {
    CLOSED_LOOP,
    OPEN_LOOP
  }

  public record ElevatorConfig(
      GlobalConstants.Gains gains,
      double positionTolerance,
      boolean softLimitsEnabled,
      double softLimitMin,
      double softLimitMax,
      double maxVoltage) {}

  // Goals
  public interface ExtensionGoal {
    DoubleSupplier getHeight();
  }

  public abstract G getGoal();

  private final String name;
  public final GenericElevatorSystemIO io;
  public final GenericElevatorSystemIOInputsAutoLogged inputs =
      new GenericElevatorSystemIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();
  private G lastGoal;

  private final PIDController pidController;
  private ElevatorFeedforward feedforward;
  private final SysIdRoutine sysIdRoutine;
  private final ElevatorConfig config;
  private final int tuningId = System.identityHashCode(this);

  @Getter private double goalPosition = 0.0;

  private ControlMode controlMode = ControlMode.CLOSED_LOOP;
  private double openLoopPercent = 0.0;
  private boolean initialized = false;
  private double zeroOffset = 0.0;
  private boolean manualGoalActive = false;
  private double manualGoalPosition = 0.0;

  public GenericPositionElevatorSystem(
      String name, GenericElevatorSystemIO io, GlobalConstants.Gains gains) {
    this(name, io, new ElevatorConfig(gains, 0.0, false, 0.0, 0.0, 12.0));
  }

  public GenericPositionElevatorSystem(
      String name, GenericElevatorSystemIO io, ElevatorConfig config) {
    this.name = name;
    this.io = io;
    this.config = config;

    pidController =
        new PIDController(
            config.gains().kP().get(), config.gains().kI().get(), config.gains().kD().get());
    pidController.setTolerance(config.positionTolerance());
    feedforward =
        new ElevatorFeedforward(
            config.gains().kS().get(),
            config.gains().kG().get(),
            config.gains().kV().get(),
            config.gains().kA().get());

    Consumer<SysIdRoutineLog> sysIdLog =
        (log) ->
            log.motor(name)
                .voltage(Volts.of(inputs.appliedVoltage))
                .linearVelocity(MetersPerSecond.of(inputs.velocity))
                .linearPosition(Meters.of(inputs.encoderPosition));
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Seconds.of(4),
                state ->
                    Logger.recordOutput("Elevators/" + name + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> io.setVoltage(voltage.in(Volts)), sysIdLog, this));

    disconnected =
        new Alert("Motor(s) disconnected on elevator: " + name + "!", Alert.AlertType.kError);
    stateTimer.start();
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

    if (getGoal() != lastGoal) {
      stateTimer.reset();
      lastGoal = getGoal();
    }

    double position = getPosition();
    if (!initialized) {
      goalPosition = position;
      pidController.reset();
      initialized = true;
    }

    double requestedGoal =
        manualGoalActive ? manualGoalPosition : getGoal().getHeight().getAsDouble();
    goalPosition = clampGoal(requestedGoal);
    inputs.goal = goalPosition;

    logOutputs(position);

    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
      return;
    }

    if (controlMode == ControlMode.OPEN_LOOP) {
      double clampedPercent = clampOpenLoop(openLoopPercent, position);
      io.setVoltage(clampedPercent * config.maxVoltage());
      return;
    }
    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> pidController.setPID(values[0], values[1], values[2]),
        config.gains().kP(),
        config.gains().kI(),
        config.gains().kD());
    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> feedforward = new ElevatorFeedforward(values[0], values[1], values[2], values[3]),
        config.gains().kS(),
        config.gains().kG(),
        config.gains().kV(),
        config.gains().kA());

    double pidOutput = pidController.calculate(position, goalPosition);
    double feedforwardOutput = feedforward.calculate(inputs.velocity);
    double outputVoltage =
        MathUtil.clamp(pidOutput + feedforwardOutput, -config.maxVoltage(), config.maxVoltage());

    io.setVoltage(outputVoltage);

    Logger.recordOutput("Elevators/" + name + "/Feedforward", feedforwardOutput);
    Logger.recordOutput("Elevators/" + name + "Goal", getGoal().toString());
  }

  public void setGoalPosition(double position) {
    manualGoalPosition = position;
    manualGoalActive = true;
    if (controlMode != ControlMode.CLOSED_LOOP) {
      pidController.reset();
    }
    controlMode = ControlMode.CLOSED_LOOP;
  }

  public void clearGoalOverride() {
    manualGoalActive = false;
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
    return Math.abs(goalPosition - getPosition()) <= config.positionTolerance();
  }

  public double getPosition() {
    return inputs.encoderPosition + zeroOffset;
  }

  public double getVelocity() {
    return inputs.velocity;
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

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  public void zeroPosition() {
    zeroOffset = -inputs.encoderPosition;
    openLoopPercent = 0.0;
    controlMode = ControlMode.CLOSED_LOOP;
    manualGoalActive = false;
    goalPosition = 0.0;
    pidController.reset();
    initialized = true;
    io.setPosition(0.0);
  }

  private double clampGoal(double goal) {
    if (!config.softLimitsEnabled()) {
      return goal;
    }
    return MathUtil.clamp(goal, config.softLimitMin(), config.softLimitMax());
  }

  private double clampOpenLoop(double percent, double position) {
    double output = MathUtil.clamp(percent, -1.0, 1.0);
    if (!config.softLimitsEnabled()) {
      return output;
    }
    if (output > 0.0 && position >= config.softLimitMax()) {
      return 0.0;
    }
    if (output < 0.0 && position <= config.softLimitMin()) {
      return 0.0;
    }
    return output;
  }

  private void logOutputs(double position) {
    Logger.recordOutput("Elevators/" + name + "/Position", position);
    Logger.recordOutput("Elevators/" + name + "/GoalPosition", goalPosition);
    Logger.recordOutput("Elevators/" + name + "/Error", goalPosition - position);
    Logger.recordOutput("Elevators/" + name + "/AtGoal", isAtGoal());
    Logger.recordOutput("Elevators/" + name + "/ControlMode", controlMode.toString());
    Logger.recordOutput("Elevators/" + name + "/OpenLoopPercent", openLoopPercent);
  }
}
