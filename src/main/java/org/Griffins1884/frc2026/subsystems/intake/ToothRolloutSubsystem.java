package org.Griffins1884.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.Griffins1884.frc2026.mechanisms.RobotMechanismDefinitions;
import org.Griffins1884.frc2026.mechanisms.rollers.VoltageRollerMechanism;

@Getter
public class ToothRolloutSubsystem
    extends VoltageRollerMechanism<ToothRolloutSubsystem.ToothRolloutGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ToothRolloutGoal implements VoltageGoal {
    STOPPED(() -> 0.0),
    INTAKE_FEED(() -> ToothRolloutConstants.INTAKE_FEED_SIMULATION_VOLTS),
    EJECT_REVERSE(() -> ToothRolloutConstants.EJECT_REVERSE_SIMULATION_VOLTS);

    private final DoubleSupplier voltageSupplier;

    @Override
    public DoubleSupplier getVoltageSupplier() {
      return voltageSupplier;
    }
  }

  private final ToothRolloutIO io;
  private ToothRolloutGoal goal = ToothRolloutGoal.STOPPED;

  public ToothRolloutSubsystem(String name, ToothRolloutIO io) {
    super(
        name,
        RobotMechanismDefinitions.TOOTH_ROLLOUT,
        io,
        new VoltageRollerConfig(ToothRolloutConstants.SIMULATION_MAX_OUTPUT_VOLTS));
    this.io = io;
  }

  public void setGoal(ToothRolloutGoal requestedGoal) {
    goal = requestedGoal != null ? requestedGoal : ToothRolloutGoal.STOPPED;
    clearGoalOverride();
  }

  public void stop() {
    setManualVoltage(0.0);
    setGoal(ToothRolloutGoal.STOPPED);
    io.stop();
  }

  public Command intakeFeedCommand() {
    return heldGoalCommand(ToothRolloutGoal.INTAKE_FEED, "ToothRolloutIntakeFeed");
  }

  public Command ejectReverseCommand() {
    return heldGoalCommand(ToothRolloutGoal.EJECT_REVERSE, "ToothRolloutEjectReverse");
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop, this).withName("ToothRolloutStop");
  }

  private Command heldGoalCommand(ToothRolloutGoal requestedGoal, String name) {
    return Commands.startEnd(() -> setGoal(requestedGoal), this::stop, this).withName(name);
  }
}
