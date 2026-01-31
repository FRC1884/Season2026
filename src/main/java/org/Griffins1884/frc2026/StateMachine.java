package org.Griffins1884.frc2026;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import java.util.*;
import org.Griffins1884.frc2026.util.StateGraph;
import org.Griffins1884.frc2026.util.Transition;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public abstract class StateMachine<E extends Enum<E>> extends SubsystemBase {
  private final StateGraph<E, Transition<E>> transitionGraph;
  private Transition<E> currentTransition;
  private Transition<E> queuedTransition;
  private final HashMap<E, Command> stateCommands;
  private final Timer transitionTimer;
  private final Set<E> currentFlags;
  private final double transitionTimeOut = 2;
  private final E undeterminedState;
  private E currentState;

  private boolean enabled;

  private final Class<E> enumType;
  private final List<StateMachine<?>> subsystems;

  private final LoggedDashboardChooser<E> stateChooser;
  private E lastChooserRequest;

  /**
   * Instantiate a new State Machine
   *
   * @param name name of the state machine to send over network tables
   * @param undeterminedState the undetermined state of the subsystem
   * @param enumType the class of enums to use for the state
   */
  public StateMachine(String name, E undeterminedState, Class<E> enumType) {
    this.enumType = enumType;

    this.undeterminedState = undeterminedState;
    currentState = undeterminedState;
    currentTransition = null;
    transitionTimer = new Timer();
    currentFlags = new HashSet<>();
    stateCommands = new HashMap<>();
    subsystems = new ArrayList<>();
    stateChooser = new LoggedDashboardChooser<>(name + "State Chooser");
    lastChooserRequest = undeterminedState;

    initStateChooser();

    setName(name);

    transitionGraph = new StateGraph<>(enumType);
    enabled = false;
  }

  private void initStateChooser() {
    stateChooser.addDefaultOption(undeterminedState.name(), undeterminedState);

    for (E state : enumType.getEnumConstants()) {
      if (state != undeterminedState) {
        stateChooser.addOption(state.name(), state);
      }
    }
  }

  /**
   * @return the child state-machines
   */
  public final List<StateMachine<?>> getChildSubsystems() {
    return subsystems;
  }

  /**
   * adds the specified system to the set of child subsystems owned by the current state machine
   *
   * @param machine the subsystem to add as a child
   */
  protected final void addChildSubsystem(StateMachine<?> machine) {
    subsystems.add(machine);
  }

  /**
   * @return the current state of the machine
   */
  public final E getState() {
    return currentState;
  }

  /**
   * Enable the state machine to start running. Transitions and state commands will only run while
   * the machine is enabled
   */
  public final void enable() {
    determineState();
    enabled = true;

    onEnable();
  }

  /**
   * Whether the state machine is enabled
   *
   * @return the current enabled status of the state machine
   */
  public final boolean isEnabled() {
    return enabled;
  }

  /** User-implemented method run immediately on the machine being enabled */
  protected void onEnable() {}

  /** User-implemented method ran immediately when the teleop period is started */
  protected void onTeleopStart() {}

  /** User-implemented method ran immediately when the autonomous period is started */
  protected void onAutonomousStart() {}

  /** User-implemented method ran immediately when the test period is started */
  protected void onTestStart() {}

  /**
   * Stop the state machine from running. No transitions or state commands will run while the
   * machine is disabled
   */
  public final void disable() {
    enabled = false;
    if (currentTransition != null) currentTransition.cancel();
    currentTransition = null;
    queuedTransition = null;
    setState(undeterminedState);
    if (getCurrentCommand() != null) getCurrentCommand().cancel();

    onDisable();
  }

  /** User-implemented method run immediately upon the machine being disabled */
  protected void onDisable() {}

  /**
   * @return whether the machine is in a determined state
   */
  public final boolean isDetermined() {
    return currentState != undeterminedState;
  }

  /**
   * Set a state command to run upon reaching that state
   *
   * @param state the state during which the command should run
   * @param command command to run
   */
  public final void registerStateCommand(E state, Command command) {
    stateCommands.put(state, command);
  }

  /**
   * Add an instant state command to run upon reaching that state
   *
   * @param state the state during which the command should run
   * @param toRun the command to run
   */
  protected final void registerStateCommand(E state, Runnable toRun) {
    registerStateCommand(state, new InstantCommand(toRun));
  }

  /**
   * Create a new transition in the machine
   *
   * @param start the start state of the transition
   * @param end the end state of the transition
   * @param command the command to run
   */
  protected final void addTransition(E start, E end, Command command) {
    transitionGraph.addEdge(new Transition<>(start, end, command));
  }

  protected final void addTransition(E start, E end) {
    transitionGraph.addEdge(new Transition<>(start, end, new InstantCommand()));
  }

  protected final void addTransition(E start, E end, Runnable toRun) {
    transitionGraph.addEdge(new Transition<>(start, end, new InstantCommand(toRun)));
  }

  protected final void removeTransition(E start, E end) {
    transitionGraph.removeEdge(start, end);
  }

  protected final void removeAllTransitionsFromState(E start) {
    for (E s : enumType.getEnumConstants()) {
      transitionGraph.removeEdge(start, s);
    }
  }

  /**
   * Adds a transition from every state to the given state
   *
   * @param state the state to go to
   * @param run transition command to run
   */
  public final void addOmniTransition(E state, Command run) {
    for (E s : enumType.getEnumConstants()) {
      if (s != state) {
        addTransition(s, state, run);
      }
    }
  }

  /**
   * Adds a transition from every state to the given state
   *
   * @param state the state to go to
   * @param run the transition runnable to run as an instant command
   */
  public final void addOmniTransition(E state, Runnable run) {
    addOmniTransition(state, new InstantCommand(run));
  }

  public final void addOmniTransition(E state) {
    addOmniTransition(state, () -> {});
  }

  public final void addOmniTransitions(E... states) {
    for (E state : states) {
      addOmniTransition(state);
    }
  }

  /**
   * Add a transition both ways between two states
   *
   * @param start beginning state
   * @param end ending state
   * @param run the command to run between them
   */
  public final void addCommutativeTransition(E start, E end, Command run) {
    transitionGraph.addEdge(new Transition<>(start, end, run));
    transitionGraph.addEdge(new Transition<>(end, start, run));
  }

  /**
   * Add a transition both ways between two states
   *
   * @param start beginning state
   * @param end ending state
   * @param run the runnable to run between them
   */
  public final void addCommutativeTransition(E start, E end, Runnable toRun) {
    transitionGraph.addEdge(new Transition<>(start, end, new InstantCommand(toRun)));
    transitionGraph.addEdge(new Transition<>(end, start, new InstantCommand(toRun)));
  }

  /**
   * Add a transition both ways between two states
   *
   * @param start beginning state
   * @param end ending state
   */
  public final void addCommutativeTransition(E start, E end) {
    transitionGraph.addEdge(new Transition<>(start, end, new InstantCommand()));
    transitionGraph.addEdge(new Transition<>(end, start, new InstantCommand()));
  }

  /**
   * Add a transition both ways between two states
   *
   * @param start beginning state
   * @param end ending state
   * @param run1 the command to run between start and end
   * @param run2 the command to run between end and start
   */
  public final void addCommutativeTransition(E start, E end, Command run1, Command run2) {
    transitionGraph.addEdge(new Transition<>(start, end, run1));
    transitionGraph.addEdge(new Transition<>(end, start, run2));
  }

  /**
   * Add a transition both ways between two states
   *
   * @param start beginning state
   * @param end ending state
   * @param run1 the runnable to run between start and end
   * @param run2 the runnable to run between end and start
   */
  public final void addCommutativeTransition(E start, E end, Runnable run1, Runnable run2) {
    transitionGraph.addEdge(new Transition<>(start, end, new InstantCommand(run1)));
    transitionGraph.addEdge(new Transition<>(end, start, new InstantCommand(run2)));
  }

  /**
   * @return true if the machine is actively transitioning between states
   */
  public final boolean isTransitioning() {
    return currentTransition != null;
  }

  /**
   * @return the object that returns the current transition information. Will be null if there is no
   *     transition
   */
  public final Transition<E> getCurrentTransition() {
    return currentTransition;
  }

  /**
   * Request a transition to a state
   *
   * @param state state to transition to
   */
  public final void requestTransition(E state) {
    Transition<E> transition = transitionGraph.getEdge(currentState, state);
    // Stop transitions to the same state from happening
    if (!isTransitioning() && transition != null && state != currentState) {
      currentTransition = transition;
      cancelStateCommand();
      transition.execute();
      transitionTimer.start();

      /*updateTransitioning();*/
    } else if (state != currentState) {
      queuedTransition = transition;
    }
  }

  private void cancelStateCommand() {
    if (stateCommands.containsKey(getState())) {
      Command prevCommand = stateCommands.get(getState());
      if (prevCommand.isScheduled()) prevCommand.cancel();
    }
  }

  /**
   * Request an instance-based state on the state machine
   *
   * @param state the state to go to
   * @param command the command to run upon reaching the state
   */
  public final void requestTransition(E state, Command command) {
    stateCommands.put(state, command);
    requestTransition(state);
  }

  /**
   * A command that will keep running until it gets to the target state
   *
   * @param state the state to go to
   * @return the command to run
   */
  public final Command transitionCommand(E state) {
    return new FunctionalCommand(
        () -> requestTransition(state), () -> {}, (interrupted) -> {}, () -> getState() == state);
  }

  /**
   * A command that will keep running until it gets to the target state, but it supports
   * instance-based states
   *
   * @param state the state to go to
   * @param command the command to run upon reaching that state
   * @return the command to run
   */
  public final Command transitionCommand(E state, Command command) {
    return new FunctionalCommand(
        () -> requestTransition(state, command),
        () -> {},
        (interrupted) -> {},
        () -> getState() == state);
  }

  /**
   * A command that will end immediately independent of if it reaches the target state or not
   *
   * @param state the state to go to
   * @param command the command to run upon reaching that state
   * @return the command to run
   */
  public final Command transitionCommand(E state, Command command, boolean waitForState) {
    if (waitForState) {
      return transitionCommand(state, command);
    } else {
      return new InstantCommand(() -> requestTransition(state, command));
    }
  }

  /**
   * A command that will end immediately independent of if it reaches the target state or not
   *
   * @param state the state to go to
   * @return the command to run
   */
  public final Command transitionCommand(E state, boolean waitForState) {
    if (waitForState) {
      return transitionCommand(state);
    } else {
      return new InstantCommand(() -> requestTransition(state));
    }
  }

  /**
   * @param state the state to wait for
   * @return the command to run
   */
  public final Command waitForState(E state) {
    return new WaitUntilCommand(() -> getState() == state);
  }

  /**
   * Wait for the state machine to indicate that a flag has been found
   *
   * @param flag the flag state to wait for
   * @return the command to run
   */
  public final Command waitForFlag(E flag) {
    return new WaitUntilCommand(() -> isFlag(flag));
  }

  /**
   * Get the current flags of a state
   *
   * @return the current flags
   */
  public final Set<E> getCurrentFlags() {
    return currentFlags;
  }

  public final String[] getCurrentFlagsAsArray() {
    int n = getCurrentFlags().size();
    String arr[] = new String[n];

    int i = 0;
    for (E flag : getCurrentFlags()) {
      arr[i] = flag.toString();
      i++;
    }

    return arr;
  }

  /**
   * Determine whether a state is currently an active flag state
   *
   * @param state the state to evaluate
   * @return whether the state is currently a flag or not
   */
  public final boolean isFlag(E state) {
    return getCurrentFlags().contains(state);
  }

  /**
   * Add a flag state to the current state
   *
   * @param flag the flag state
   */
  public final void setFlag(E flag) {
    currentFlags.add(flag);
  }

  /**
   * A command to set a flag on the state machine
   *
   * @param flag the flag to set on the state machine
   * @return the command to run
   */
  public final Command setFlagCommand(E flag) {
    return new InstantCommand(() -> setFlag(flag));
  }

  /**
   * Remove a specific flag state from the list of flags states
   *
   * @param flag the flag state to clear
   */
  public final void clearFlag(E flag) {
    currentFlags.remove(flag);
  }

  public final Command clearFlagCommand(E flag) {
    return new InstantCommand(() -> clearFlag(flag));
  }

  /** Clear all states that are currently flags */
  public final void clearFlags() {
    currentFlags.clear();
  }

  @Override
  public final void periodic() {
    E chooserRequest = stateChooser.get();

    if (enabled) {
      updateTransitioning();

      if (lastChooserRequest != chooserRequest) {
        requestTransition(chooserRequest);
      }
    }

    recordLogs();
    update();

    lastChooserRequest = chooserRequest;
  }

  private void recordLogs() {
    Logger.recordOutput(
        getName() + "/desired",
        isTransitioning() ? getCurrentTransition().getEndState().name() : getState().name());

    Logger.recordOutput(getName() + "/state", getState().toString());

    Logger.recordOutput(getName() + "/transitioning", isTransitioning());
    Logger.recordOutput(getName() + "/flags", getCurrentFlagsAsArray());
    Logger.recordOutput(getName(), getCurrentFlagsAsArray());

    Logger.recordOutput(getName() + "/enabled", enabled);

    logAdditionalOutputs();
  }

  protected final void setState(E state) {
    cancelStateCommand();

    currentState = state;
    clearFlags();
    if (stateCommands.containsKey(state)) {
      CommandScheduler.getInstance().schedule(stateCommands.get(state));
    }
  }

  private void updateTransitioning() {
    if (isTransitioning() && currentTransition.isFinished()) {
      setState(currentTransition.getEndState());
      currentTransition = null;
      transitionTimer.stop();
      transitionTimer.reset();
    }

    if (queuedTransition != null
        && (!isTransitioning() || transitionTimer.hasElapsed(transitionTimeOut))) {
      forceChangeTransition();
    }
  }

  private void forceChangeTransition() {

    if (currentTransition != null) currentTransition.cancel();
    currentTransition = queuedTransition;
    currentTransition.execute();
    queuedTransition = null;
    transitionTimer.reset();
    transitionTimer.start();
    clearFlags();
  }

  public final String toString() {
    return "Stated Subsystem Machine - "
        + getName()
        + "; In State: "
        + getState().name()
        + "; In Transition: "
        + isTransitioning();
  }

  public final void determineState() {
    if (!isDetermined()) determineSelf();
  }

  protected void update() {}

  /**
   * User-implemented method to determine the state of the machine. THIS METHOD IS RESPONSIBLE FOR
   * CALLING the setState() method
   */
  protected abstract void determineSelf();

  // Override this to any extra logged values to the logger in this method
  protected void logAdditionalOutputs() {}

  public Map<String, Sendable> additionalSendables() {
    return new HashMap<>();
  }
}
