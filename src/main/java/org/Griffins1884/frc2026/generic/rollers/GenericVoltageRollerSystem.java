package org.Griffins1884.frc2026.generic.rollers;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * (Note: we'll probably need to find a better name for this package. Rollers can constitute
 * anything from actual intake rollers to shooter flywheels.) <br>
 * <br>
 * This subsystem class and the IO implementations in this package can be conveniently subclassed to
 * provide a foundation for any such system.
 */
public abstract class GenericVoltageRollerSystem<G extends GenericVoltageRollerSystem.VoltageGoal>
    extends SubsystemBase {
  public enum ControlMode {
    GOAL,
    MANUAL
  }

  public record VoltageRollerConfig(double maxVoltage) {}

  public interface VoltageGoal {
    DoubleSupplier getVoltageSupplier();
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
  private final VoltageRollerConfig config;

  private ControlMode controlMode = ControlMode.GOAL;
  private double goalVoltage = 0.0;
  private double manualVoltage = 0.0;

  public GenericVoltageRollerSystem(String name, GenericRollerSystemIO io) {
    this(name, io, new VoltageRollerConfig(12.0));
  }

  public GenericVoltageRollerSystem(
      String name, GenericRollerSystemIO io, VoltageRollerConfig config) {
    this.name = name;
    this.io = io;
    this.config = config;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Seconds.of(4),
                state -> Logger.recordOutput("Rollers/" + name + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> io.runVolts(voltage.in(Volts)),
                (log) ->
                    log.motor(name)
                        .voltage(Volts.of(inputs.appliedVoltage))
                        .angularVelocity(RadiansPerSecond.of(inputs.velocityRadsPerSec))
                        .angularPosition(Radian.of(inputs.positionRads)),
                this));

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

    double requestedVoltage =
        controlMode == ControlMode.MANUAL
            ? manualVoltage
            : getGoal().getVoltageSupplier().getAsDouble();
    goalVoltage = MathUtil.clamp(requestedVoltage, -config.maxVoltage(), config.maxVoltage());

    logOutputs();

    if (DriverStation.isDisabled()) {
      io.runVolts(0.0);
      return;
    }

    io.runVolts(goalVoltage);
    Logger.recordOutput("Rollers/" + name + "Goal", getGoal().toString());
  }

  public void setManualVoltage(double volts) {
    manualVoltage = volts;
    controlMode = ControlMode.MANUAL;
  }

  public void clearGoalOverride() {
    controlMode = ControlMode.GOAL;
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

  private void logOutputs() {
    Logger.recordOutput("Rollers/" + name + "/GoalVolts", goalVoltage);
    Logger.recordOutput("Rollers/" + name + "/ControlMode", controlMode.toString());
  }
}
