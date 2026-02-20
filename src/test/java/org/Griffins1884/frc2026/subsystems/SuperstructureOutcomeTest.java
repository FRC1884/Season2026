// package org.Griffins1884.frc2026.subsystems;

// import static org.junit.jupiter.api.Assertions.assertEquals;

// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.wpilibj.simulation.SimHooks;
// import edu.wpi.first.wpilibj2.command.Command;
// import org.Griffins1884.frc2026.GlobalConstants;
// import org.Griffins1884.frc2026.subsystems.Superstructure.SuperState;
// import org.junit.jupiter.api.BeforeAll;
// import org.junit.jupiter.api.Test;

// public class SuperstructureOutcomeTest {
//   private static final double LOOP_PERIOD_SECONDS = 0.02;
//   private static Superstructure superstructure;

//   @BeforeAll
//   static void setup() {
//     GlobalConstants.TUNING_MODE = false;
//     HAL.initialize(500, 0);
//     SimHooks.pauseTiming();
//     superstructure = new Superstructure(null);
//   }

//   @Test
//   void printAllSuperstructureStateOutcomes() {
//     for (SuperState state : SuperState.values()) {
//       forceState(superstructure, state);
//       superstructure.periodic();
//       Superstructure.SuperstructureOutcome outcome = superstructure.getOutcomeSnapshot();
//       System.out.println("State " + state + ": " + outcome);
//       assertEquals(state, outcome.state());
//     }
//   }

//   @Test
//   void printSuperstructureRoutineOverTime() {
//     RoutineStep[] routine =
//         new RoutineStep[] {
//           new RoutineStep(SuperState.IDLING, 2),
//           new RoutineStep(SuperState.INTAKING, 3),
//           new RoutineStep(SuperState.SHOOTING, 3),
//           new RoutineStep(SuperState.FERRYING, 3),
//           new RoutineStep(SuperState.ENDGAME_CLIMB, 25),
//           new RoutineStep(SuperState.AUTO_CLIMB, 30),
//           new RoutineStep(SuperState.CLIMB_DETACH, 30),
//           new RoutineStep(SuperState.TESTING, 3),
//           new RoutineStep(SuperState.IDLING, 3)
//         };

//     double timeSeconds = 0.0;
//     for (RoutineStep step : routine) {
//       for (int tick = 0; tick < step.ticks; tick++) {
//         forceState(superstructure, step.state);
//         superstructure.periodic();
//         Superstructure.SuperstructureOutcome outcome = superstructure.getOutcomeSnapshot();
//         System.out.printf("t=%.2fs state=%s outcome=%s%n", timeSeconds, step.state, outcome);
//         timeSeconds += LOOP_PERIOD_SECONDS;
//         SimHooks.stepTiming(LOOP_PERIOD_SECONDS);
//       }
//     }
//   }

//   private static void forceState(Superstructure superstructure, SuperState state) {
//     Command command = superstructure.setSuperStateCmd(state);
//     command.initialize();
//   }

//   private record RoutineStep(SuperState state, int ticks) {}
// }
