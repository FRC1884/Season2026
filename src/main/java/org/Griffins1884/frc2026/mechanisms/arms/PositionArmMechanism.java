package org.Griffins1884.frc2026.mechanisms.arms;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.mechanisms.MechanismDefinition;
import org.Griffins1884.frc2026.mechanisms.MechanismHealth;
import org.Griffins1884.frc2026.mechanisms.MechanismTelemetry;
import org.Griffins1884.frc2026.runtime.RuntimeModeManager;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Mechanism-backed replacement for the old generic position arm base. */
public abstract class PositionArmMechanism<G extends PositionArmMechanism.PivotGoal>
    extends SubsystemBase {
  public enum ControlMode {
    CLOSED_LOOP,
    OPEN_LOOP
  }

  public record ArmConfig(
      LoggedTunableNumber kP,
      LoggedTunableNumber kI,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kG,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA,
      LoggedTunableNumber motionMagicCruiseVelocity,
      LoggedTunableNumber motionMagicAcceleration,
      LoggedTunableNumber motionMagicJerk,
      double positionTolerance,
      boolean softLimitsEnabled,
      double softLimitMin,
      double softLimitMax,
      double maxVoltage) {}

  public interface PivotGoal {
    DoubleSupplier getAngle();
  }

  public abstract G getGoal();

  private final String name;
  private final MechanismDefinition definition;
  private final String mechanismKey;
  public final MechanismArmIO io;
  protected final MechanismArmIOInputsAutoLogged inputs = new MechanismArmIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();
  private G lastGoal;

  private final PIDController pidController;
  private ArmFeedforward feedforward;
  private final SysIdRoutine sysIdRoutine;
  private final ArmConfig config;
  private final int tuningId = System.identityHashCode(this);

  @Getter private double goalPosition = 0.0;

  private ControlMode controlMode = ControlMode.CLOSED_LOOP;
  private double openLoopPercent = 0.0;
  private boolean initialized = false;
  private double zeroOffset = 0.0;
  private boolean manualGoalActive = false;
  private double manualGoalPosition = 0.0;
  private boolean connected = false;
  private MechanismHealth health = MechanismHealth.OFFLINE;

  protected PositionArmMechanism(
      String name, MechanismDefinition definition, MechanismArmIO io, GlobalConstants.Gains gains) {
    this(
        name,
        definition,
        io,
        new ArmConfig(
            gains.kP(),
            gains.kI(),
            gains.kD(),
            gains.kS(),
            gains.kG(),
            gains.kV(),
            gains.kA(),
            null,
            null,
            null,
            0.0,
            false,
            0.0,
            0.0,
            12.0));
  }

  protected PositionArmMechanism(
      String name, MechanismDefinition definition, MechanismArmIO io, ArmConfig config) {
    this.name = Objects.requireNonNull(name, "name");
    this.definition = Objects.requireNonNull(definition, "definition");
    this.mechanismKey = definition.key();
    this.io = Objects.requireNonNull(io, "io");
    this.config = Objects.requireNonNull(config, "config");

    pidController = new PIDController(config.kP().get(), config.kI().get(), config.kD().get());
    pidController.setTolerance(config.positionTolerance());
    feedforward =
        new ArmFeedforward(
            config.kS().get(), config.kG().get(), config.kV().get(), config.kA().get());

    Consumer<SysIdRoutineLog> sysIdLog =
        log ->
            log.motor(name)
                .voltage(Volts.of(inputs.appliedVoltage))
                .angularVelocity(RadiansPerSecond.of(inputs.velocity))
                .angularPosition(Radian.of(inputs.encoderPosition));
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Seconds.of(4),
                state -> {
                  if (RuntimeModeManager.isDebugEnabled(mechanismKey)) {
                    Logger.recordOutput("Arms/" + name + "/SysIdState", state.toString());
                  }
                }),
            new SysIdRoutine.Mechanism(
                voltage -> io.setVoltage(voltage.in(Volts)), sysIdLog, this));

    disconnected = new Alert("Motor(s) disconnected on arm: " + name + "!", Alert.AlertType.kError);
    stateTimer.start();
    recordConfigSnapshot();
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
    if (RuntimeModeManager.isDebugEnabled(mechanismKey)) {
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
    connected = !anyDisconnected;
    health = connected ? MechanismHealth.NOMINAL : MechanismHealth.OFFLINE;

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
        manualGoalActive ? manualGoalPosition : getGoal().getAngle().getAsDouble();
    goalPosition = clampGoal(requestedGoal);
    inputs.goal = goalPosition;

    logOutputs(anyDisconnected, position);

    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
      return;
    }

    if (controlMode == ControlMode.OPEN_LOOP) {
      double clampedPercent = clampOpenLoop(openLoopPercent, position);
      io.setVoltage(clampedPercent * config.maxVoltage());
      return;
    }
    double kP = config.kP().get();
    double kI = config.kI().get();
    double kD = config.kD().get();

    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> pidController.setPID(values[0], values[1], values[2]),
        config.kP(),
        config.kI(),
        config.kD());
    if (config.motionMagicCruiseVelocity() != null
        && config.motionMagicAcceleration() != null
        && config.motionMagicJerk() != null) {
      LoggedTunableNumber.ifChanged(
          tuningId,
          values -> io.setMotionMagicParams(values[0], values[1], values[2]),
          config.motionMagicCruiseVelocity(),
          config.motionMagicAcceleration(),
          config.motionMagicJerk());
    }

    if (io.usesInternalPositionControl()) {
      double kG = config.kG() != null ? config.kG().get() : 0.0;
      io.setPositionSetpoint(goalPosition, kP, kI, kD, kG);
      if (RuntimeModeManager.isDebugEnabled(mechanismKey)) {
        Logger.recordOutput("Arms/" + name + "/Feedforward", 0.0);
        Logger.recordOutput("Arms/" + name + "/Goal", getGoal().toString());
      }
      return;
    }
    LoggedTunableNumber.ifChanged(
        tuningId,
        values -> feedforward = new ArmFeedforward(values[0], values[1], values[2], values[3]),
        config.kS(),
        config.kG(),
        config.kV(),
        config.kA());

    double pidOutput = pidController.calculate(position, goalPosition);
    double feedforwardOutput = feedforward.calculate(position, inputs.velocity);
    double outputVoltage =
        MathUtil.clamp(pidOutput + feedforwardOutput, -config.maxVoltage(), config.maxVoltage());

    io.setVoltage(outputVoltage);

    if (RuntimeModeManager.isDebugEnabled(mechanismKey)) {
      Logger.recordOutput("Arms/" + name + "/Feedforward", feedforwardOutput);
      Logger.recordOutput("Arms/" + name + "/Goal", getGoal().toString());
    }
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

  public MechanismDefinition getDefinition() {
    return definition;
  }

  public boolean isConnected() {
    return connected;
  }

  public MechanismHealth getHealth() {
    return health;
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

  private void logOutputs(boolean anyDisconnected, double position) {
    String mechanismRoot = "Mechanisms/" + mechanismKey;
    if (RuntimeModeManager.shouldLog(MechanismTelemetry.Signal.CONNECTION, mechanismKey)) {
      Logger.recordOutput(mechanismRoot + "/Connected", !anyDisconnected);
    }
    if (RuntimeModeManager.shouldLog(MechanismTelemetry.Signal.HEALTH, mechanismKey)) {
      Logger.recordOutput(
          mechanismRoot + "/Health",
          anyDisconnected ? MechanismHealth.OFFLINE.name() : MechanismHealth.NOMINAL.name());
    }
    if (RuntimeModeManager.shouldLog(MechanismTelemetry.Signal.POSITION, mechanismKey)) {
      Logger.recordOutput(mechanismRoot + "/Position", position);
    }
    if (RuntimeModeManager.shouldLog(MechanismTelemetry.Signal.TARGET, mechanismKey)) {
      Logger.recordOutput(mechanismRoot + "/TargetPosition", goalPosition);
    }
    if (RuntimeModeManager.shouldLog(MechanismTelemetry.Signal.CURRENT, mechanismKey)) {
      Logger.recordOutput(mechanismRoot + "/SupplyCurrentAmps", inputs.supplyCurrentAmps);
      Logger.recordOutput(mechanismRoot + "/TorqueCurrentAmps", inputs.torqueCurrentAmps);
    }
    if (RuntimeModeManager.shouldLog(MechanismTelemetry.Signal.VOLTAGE, mechanismKey)) {
      Logger.recordOutput(mechanismRoot + "/AppliedVoltage", inputs.appliedVoltage);
    }
    if (RuntimeModeManager.isDebugEnabled(mechanismKey)) {
      Logger.recordOutput("Arms/" + name + "/Position", position);
      Logger.recordOutput("Arms/" + name + "/GoalPosition", goalPosition);
      Logger.recordOutput("Arms/" + name + "/Error", goalPosition - position);
      Logger.recordOutput("Arms/" + name + "/AtGoal", isAtGoal());
      Logger.recordOutput("Arms/" + name + "/ControlMode", controlMode.toString());
      Logger.recordOutput("Arms/" + name + "/OpenLoopPercent", openLoopPercent);
    }
  }

  private void recordConfigSnapshot() {
    if (RuntimeModeManager.shouldLog(MechanismTelemetry.Signal.CONFIG_SNAPSHOT, mechanismKey)) {
      Logger.recordOutput("Mechanisms/" + mechanismKey + "/Definition", definition.toString());
    }
  }
}
