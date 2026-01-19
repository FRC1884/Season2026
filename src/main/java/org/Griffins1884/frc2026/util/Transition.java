package org.Griffins1884.frc2026.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lombok.Getter;

public class Transition<E extends Enum<E>> {
  @Getter protected final E startState;
  @Getter protected final E endState;
  @Getter private final Command command;

  public Transition(E startState, E endState, Command command) {
    this.startState = startState;
    this.endState = endState;
    this.command = command;
  }

  /**
   * Tell if this transition already has the to and from states of `other`
   *
   * @param other the transition to compare
   * @return whether the transition is a duplicate or not
   */
  public boolean isValid(Transition<E> other) {
    return !(other.startState == startState && other.endState == endState);
  }

  /**
   * A string representation of the transition to be easily printed (generally, print the start,
   * end, and interruption states, as well as whatever you're running)
   */
  public String toString() {
    return "Start state: "
        + startState.name()
        + ", End state: "
        + endState.name()
        + ", Command: "
        + command.toString();
  }

  /**
   * Run (or start) the transition, then return a BooleanSupplier indicating whether it is finished
   *
   * @return whether the transition is finished
   */
  public void execute() {
    CommandScheduler.getInstance().schedule(command);
  }

  /** Cancel the transition (if it's currently running) */
  public void cancel() {
    command.cancel();
  }

  public boolean isFinished() {
    return command.isFinished();
  }
}
