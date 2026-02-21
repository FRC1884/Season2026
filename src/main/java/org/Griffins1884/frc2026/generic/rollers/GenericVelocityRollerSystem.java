package org.Griffins1884.frc2026.generic.rollers;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
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
  private SimpleMotorFeedforward feedforward;
  private double feedforwardKa = 0.0;
  private final VelocityRollerConfig config;
  private final int tuningId = System.identityHashCode(this);

  private static final double RPM_TO_RAD_PER_SEC = 2.0 * Math.PI / 60.0;

  private double goalVelocity = 0.0;
  private boolean manualGoalActive = false;
  private double manualGoalVelocity = 0.0;
  private double lastGoalVelocityRadPerSec = 0.0;
  private double lastTimestampSec = 0.0;
  private GlobalConstants.Gains lastActiveGains = null;

  public GenericVelocityRollerSystem(
      String name, GenericRollerSystemIO io, GlobalConstants.Gains gains) {
    this(name, io, new VelocityRollerConfig(gains, 0.0, 12.0));
  }

  public GenericVelocityRollerSystem(
      String name, GenericRollerSystemIO io, VelocityRollerConfig config) {
    this.name = name;
    this.io = io;
    this.config = config;

    pidController = new PIDController(config.gains().kP().get(), config.gains().kI().get(), 0.0);
    pidController.setTolerance(config.velocityTolerance());
    feedforward = new SimpleMotorFeedforward(config.gains().kS().get(), config.gains().kV().get());
    feedforwardKa = config.gains().kA().get();
    lastTimestampSec = Timer.getFPGATimestamp();
    Consumer<SysIdRoutineLog> sysIdLog =
        (log) ->
            log.motor(name)
                .voltage(Volts.of(inputs.appliedVoltage))
                .angularVelocity(RadiansPerSecond.of(inputs.velocityRadsPerSec))
                .angularPosition(Radian.of(inputs.positionRads));
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Seconds.of(4),
                state -> {
                  if (GlobalConstants.isDebugMode()) {
                    Logger.recordOutput("Rollers/" + name + "/SysIdState", state.toString());
                  }
                }),
            new SysIdRoutine.Mechanism(voltage -> io.runVolts(voltage.in(Volts)), sysIdLog, this));

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
    if (GlobalConstants.isDebugMode()) {
      Logger.processInputs(name, inputs);
    }

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

    GlobalConstants.Gains activeGains = getActiveGains(requestedVelocity);
    if (activeGains == null) {
      activeGains = config.gains();
    }
    String gainsLabel = getActiveGainsLabel(requestedVelocity);

    logOutputs(measuredVelocity, activeGains, gainsLabel);

    if (DriverStation.isDisabled()) {
      io.runVolts(0.0);
      return;
    }

    if (activeGains != lastActiveGains) {
      pidController.setPID(activeGains.kP().get(), activeGains.kI().get(), 0.0);
      feedforward = new SimpleMotorFeedforward(activeGains.kS().get(), activeGains.kV().get());
      feedforwardKa = activeGains.kA().get();
      lastActiveGains = activeGains;
    }

    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> pidController.setPID(values[0], values[1], 0.0),
        activeGains.kP(),
        activeGains.kI());
    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> feedforward = new SimpleMotorFeedforward(values[0], values[1]),
        activeGains.kS(),
        activeGains.kV());
    LoggedTunableNumber.ifChanged(tuningId, values -> feedforwardKa = values[0], activeGains.kA());

    double nowSec = Timer.getFPGATimestamp();
    double dtSec = nowSec - lastTimestampSec;
    if (dtSec <= 0.0) {
      dtSec = 0.02;
    }
    lastTimestampSec = nowSec;

    double goalVelocityRadPerSec = goalVelocity * RPM_TO_RAD_PER_SEC;
    double goalAccelRadPerSec2 = (goalVelocityRadPerSec - lastGoalVelocityRadPerSec) / dtSec;
    lastGoalVelocityRadPerSec = goalVelocityRadPerSec;

    double feedforwardVolts =
        feedforward.calculate(goalVelocityRadPerSec) + feedforwardKa * goalAccelRadPerSec2;
    double pidOutput = pidController.calculate(measuredVelocity, goalVelocity);
    double outputVoltage =
        MathUtil.clamp(pidOutput + feedforwardVolts, -config.maxVoltage(), config.maxVoltage());

    io.runVolts(outputVoltage);

    if (GlobalConstants.isDebugMode()) {
      Logger.recordOutput("Rollers/" + name + "/Feedforward", feedforwardVolts);
    }
    Logger.recordOutput("Rollers/" + name + "Goal", getGoal().toString());
  }

  public void setGoalVelocity(double velocity) {
    manualGoalVelocity = velocity;
    manualGoalActive = true;
  }

  public void clearGoalOverride() {
    manualGoalActive = false;
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

  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  protected GlobalConstants.Gains getActiveGains(double requestedVelocityRpm) {
    return config.gains();
  }

  protected String getActiveGainsLabel(double requestedVelocityRpm) {
    return "DEFAULT";
  }

  private void logOutputs(
      double measuredVelocity, GlobalConstants.Gains activeGains, String gainsLabel) {
    Logger.recordOutput("Rollers/" + name + "/VelocityRpm", measuredVelocity);
    Logger.recordOutput("Rollers/" + name + "/GoalVelocity", goalVelocity);
    Logger.recordOutput("Rollers/" + name + "/Error", goalVelocity - measuredVelocity);
    Logger.recordOutput("Rollers/" + name + "/AtGoal", isAtGoal());
    Logger.recordOutput("Rollers/" + name + "/ControlMode", "EXTERNAL_PID");
    if (GlobalConstants.isDebugMode()) {
      Logger.recordOutput("Rollers/" + name + "/VelocityCommandRpm", goalVelocity);
      Logger.recordOutput("Rollers/" + name + "/VelocityMeasuredRpm", measuredVelocity);
      Logger.recordOutput(
          "Rollers/" + name + "/ClosedLoopErrorRpm", goalVelocity - measuredVelocity);
      Logger.recordOutput(
          "Rollers/" + name + "/VelocityCommandRadPerSec", goalVelocity * RPM_TO_RAD_PER_SEC);
      Logger.recordOutput("Rollers/" + name + "/Gains/Profile", gainsLabel);
      Logger.recordOutput("Rollers/" + name + "/Gains/kP", activeGains.kP().get());
      Logger.recordOutput("Rollers/" + name + "/Gains/kI", activeGains.kI().get());
      Logger.recordOutput("Rollers/" + name + "/Gains/kD", activeGains.kD().get());
      Logger.recordOutput("Rollers/" + name + "/Gains/kS", activeGains.kS().get());
      Logger.recordOutput("Rollers/" + name + "/Gains/kV", activeGains.kV().get());
      Logger.recordOutput("Rollers/" + name + "/FeedforwardDisabled", false);
    }
  }
}
