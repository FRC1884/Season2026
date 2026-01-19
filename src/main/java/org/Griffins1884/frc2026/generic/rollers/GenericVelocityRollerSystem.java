package org.Griffins1884.frc2026.generic.rollers;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.Griffins1884.frc2026.GlobalConstants;
import org.littletonrobotics.junction.Logger;

/**
 * (Note: we'll probably need to find a better name for this package. Rollers can constitute
 * anything from actual intake rollers to shooter flywheels.) <br>
 * <br>
 * This subsystem class and the IO implementations in this package can be conveniently subclassed to
 * provide a foundation for any such system.
 */
public abstract class GenericVelocityRollerSystem<
        G extends GenericVelocityRollerSystem.VelocityGoal>
    extends SubsystemBase {
  public enum ControlMode {
    CLOSED_LOOP,
    OPEN_LOOP
  }

  public record VelocityRollerConfig(
      GlobalConstants.Gains gains, double velocityTolerance, double maxVoltage) {}

  public interface VelocityGoal {
    DoubleSupplier getVelocitySupplier();
  }

  public abstract G getGoal();

  private final String name;
  private final GenericRollerSystemIO io;
  protected final GenericRollerSystemIOInputsAutoLogged inputs =
      new GenericRollerSystemIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();
  private G lastGoal;

  private final SysIdRoutine sysIdRoutine;
  private final PIDController pidController;
  private final SimpleMotorFeedforward feedforward;
  private final VelocityRollerConfig config;

  private ControlMode controlMode = ControlMode.CLOSED_LOOP;
  private double goalVelocity = 0.0;
  private double openLoopPercent = 0.0;
  private boolean manualGoalActive = false;
  private double manualGoalVelocity = 0.0;

  public GenericVelocityRollerSystem(
      String name, GenericRollerSystemIO io, GlobalConstants.Gains gains) {
    this(name, io, new VelocityRollerConfig(gains, 0.0, 12.0));
  }

  public GenericVelocityRollerSystem(
      String name, GenericRollerSystemIO io, VelocityRollerConfig config) {
    this.name = name;
    this.io = io;
    this.config = config;

    pidController =
        new PIDController(config.gains().kP(), config.gains().kI(), config.gains().kD());
    pidController.setTolerance(config.velocityTolerance());
    feedforward =
        new SimpleMotorFeedforward(config.gains().kS(), config.gains().kV(), config.gains().kA());

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Seconds.of(4),
                state -> Logger.recordOutput("Rollers/" + name + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(voltage -> io.runVolts(voltage.in(Volts)), null, this));

    disconnected = new Alert(name + " motor disconnected!", AlertType.kWarning);
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

    double measuredVelocity = inputs.velocity;
    double requestedVelocity =
        manualGoalActive ? manualGoalVelocity : getGoal().getVelocitySupplier().getAsDouble();
    goalVelocity = requestedVelocity;

    logOutputs(measuredVelocity);

    if (DriverStation.isDisabled()) {
      io.runVolts(0.0);
      return;
    }

    if (controlMode == ControlMode.OPEN_LOOP) {
      double clampedPercent = MathUtil.clamp(openLoopPercent, -1.0, 1.0);
      io.runVolts(clampedPercent * config.maxVoltage());
      return;
    }

    double pidOutput = pidController.calculate(measuredVelocity, goalVelocity);
    double feedforwardOutput = feedforward.calculate(goalVelocity);
    double outputVoltage =
        MathUtil.clamp(pidOutput + feedforwardOutput, -config.maxVoltage(), config.maxVoltage());

    io.runVolts(outputVoltage);

    Logger.recordOutput("Rollers/" + name + "/Feedforward", feedforwardOutput);
    Logger.recordOutput("Rollers/" + name + "Goal", getGoal().toString());
  }

  public void setGoalVelocity(double velocity) {
    manualGoalVelocity = velocity;
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
    return Math.abs(goalVelocity - inputs.velocity) <= config.velocityTolerance();
  }

  public double getVelocityRpm() {
    return inputs.velocity;
  }

  public double getVelocityRadPerSec() {
    return inputs.velocityRadsPerSec;
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

  private void logOutputs(double measuredVelocity) {
    Logger.recordOutput("Rollers/" + name + "/VelocityRpm", measuredVelocity);
    Logger.recordOutput("Rollers/" + name + "/GoalVelocity", goalVelocity);
    Logger.recordOutput("Rollers/" + name + "/Error", goalVelocity - measuredVelocity);
    Logger.recordOutput("Rollers/" + name + "/AtGoal", isAtGoal());
    Logger.recordOutput("Rollers/" + name + "/ControlMode", controlMode.toString());
    Logger.recordOutput("Rollers/" + name + "/OpenLoopPercent", openLoopPercent);
  }
}
