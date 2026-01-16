package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  public enum ControlMode {
    GOAL,
    MANUAL
  }

  public record ClimberConfig(double maxVoltage) {}

  @RequiredArgsConstructor
  @Getter
  public enum ClimberGoal {
    IDLING(() -> 0.0), // Intake is off
    TESTING(new LoggedTunableNumber("Climber/Test", 0.0));

    private final DoubleSupplier voltageSupplier;
  }

  @Getter @Setter private ClimberGoal goal = ClimberGoal.IDLING;
  @Getter private ClimberGoal lastGoal = ClimberGoal.IDLING;
  private final String name;
  public final ClimberIO io;
  protected final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();
  private final SysIdRoutine sysIdRoutine;
  private final ClimberConfig config;

  private ControlMode controlMode = ControlMode.GOAL;
  private double manualVoltage = 0.0;
  private double goalVoltage = 0.0;

  public ClimberSubsystem(String name, ClimberIO io) {
    this(name, io, new ClimberConfig(12.0));
  }

  public ClimberSubsystem(String name, ClimberIO io, ClimberConfig config) {
    this.name = name;
    this.io = io;
    this.config = config;

    Consumer<SysIdRoutineLog> sysIdLogCallbackDrive =
        (log) -> {
          log.motor("Climber").voltage(Volts.of(inputs.appliedVoltage));
        };
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Seconds.of(4),
                state -> Logger.recordOutput("Climber/" + name + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.runVolts(voltage.in(Volts)), sysIdLogCallbackDrive, this));

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /**
   * NOT the same as {@link edu.wpi.first.wpilibj2.command.Subsystem#periodic()}. This method will
   * be called periodically in {@link ClimberSubsystem}, hence why this subsystem does not extend
   * {@link edu.wpi.first.wpilibj2.command.SubsystemBase}.
   */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);

    if (getGoal() != lastGoal) {
      stateTimer.reset();
      lastGoal = getGoal();
    }

    double requestedVoltage =
        controlMode == ControlMode.MANUAL
            ? manualVoltage
            : getGoal().getVoltageSupplier().getAsDouble();
    goalVoltage = MathUtil.clamp(requestedVoltage, -config.maxVoltage(), config.maxVoltage());
    inputs.goal = goalVoltage;

    logOutputs();

    if (DriverStation.isDisabled()) {
      io.runVolts(0.0);
      return;
    }

    io.runVolts(goalVoltage);
    Logger.recordOutput("Climber/" + name + "/Goal", getGoal().toString());
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

  private void logOutputs() {
    Logger.recordOutput("Climber/" + name + "/GoalVolts", goalVoltage);
    Logger.recordOutput("Climber/" + name + "/ControlMode", controlMode.toString());
  }
}
