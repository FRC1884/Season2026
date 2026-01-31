package org.Griffins1884.frc2026;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import org.Griffins1884.frc2026.util.Transition;
import org.junit.jupiter.api.Test;

class StateMachineTest {
  private enum TestState {
    UNDETERMINED,
    A,
    B
  }

  private static class TestStateMachine extends StateMachine<TestState> {
    TestStateMachine() {
      super("Test", TestState.UNDETERMINED, TestState.class);
    }

    @Override
    protected void determineSelf() {
      setState(TestState.A);
    }
  }

  @Test
  void forceChangeTransitionStartsTimer() throws Exception {
    TestStateMachine machine = new TestStateMachine();
    Transition<TestState> transition =
        new Transition<>(TestState.A, TestState.B, new InstantCommand());

    Field queuedField = StateMachine.class.getDeclaredField("queuedTransition");
    queuedField.setAccessible(true);
    queuedField.set(machine, transition);

    Method forceMethod = StateMachine.class.getDeclaredMethod("forceChangeTransition");
    forceMethod.setAccessible(true);
    forceMethod.invoke(machine);

    Field timerField = StateMachine.class.getDeclaredField("transitionTimer");
    timerField.setAccessible(true);
    Timer timer = (Timer) timerField.get(machine);

    assertTrue(timer.isRunning());
  }
}
