package org.Griffins1884.frc2026.subsystems.indexer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.Griffins1884.frc2026.Config;
import org.Griffins1884.frc2026.GlobalConstants.RobotMode;
import org.Griffins1884.frc2026.mechanisms.rollers.MechanismRollerIO.MechanismRollerIOInputs;
import org.Griffins1884.frc2026.subsystems.indexer.SpindexerSubsystem.SpindexerGoal;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class SpindexerSubsystemTest {
  private static final double EPSILON = 1e-9;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void goalsAndModePolicyMatchTheSimulationOnlyContract() {
    assertEquals(0.0, SpindexerGoal.IDLING.getVelocitySupplier().getAsDouble(), EPSILON);
    assertTrue(SpindexerGoal.INDEXING.getVelocitySupplier().getAsDouble() > 0.0);
    assertTrue(SpindexerGoal.REVERSE.getVelocitySupplier().getAsDouble() < 0.0);
    assertTrue(Double.isFinite(SpindexerGoal.TESTING.getVelocitySupplier().getAsDouble()));
    assertTrue(Config.Subsystems.isSpindexerEnabled(RobotMode.SIM));
    assertFalse(Config.Subsystems.isSpindexerEnabled(RobotMode.REAL));
    assertFalse(Config.Subsystems.isSpindexerEnabled(RobotMode.REPLAY));
  }

  @Test
  void simulationIsDeterministicAndFailsNonFiniteOrDisabledCommandsToZero() {
    setEnabled(true);
    SpindexerIOSim first = new SpindexerIOSim();
    SpindexerIOSim second = new SpindexerIOSim();
    TestInputs firstInputs = new TestInputs();
    TestInputs secondInputs = new TestInputs();
    first.runVelocity(250.0, 0.0);
    second.runVelocity(250.0, 0.0);
    for (int cycle = 0; cycle < 5; cycle++) {
      first.updateInputs(firstInputs);
      second.updateInputs(secondInputs);
    }
    assertEquals(firstInputs.positionRads, secondInputs.positionRads, EPSILON);
    assertTrue(firstInputs.velocity > 0.0 && firstInputs.connected[0]);

    first.runVelocity(-180.0, 0.0);
    first.updateInputs(firstInputs);
    assertTrue(firstInputs.velocity < 0.0);
    first.runVelocity(Double.NaN, 0.0);
    first.updateInputs(firstInputs);
    assertStopped(firstInputs);
    first.runVolts(0.0);
    first.updateInputs(firstInputs);
    assertStopped(firstInputs);
    first.runVelocity(250.0, 0.0);
    setEnabled(false);
    first.updateInputs(firstInputs);
    assertStopped(firstInputs);
  }

  private static void setEnabled(boolean enabled) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  private static void assertStopped(TestInputs inputs) {
    assertEquals(0.0, inputs.velocity, EPSILON);
    assertEquals(0.0, inputs.appliedVoltage, EPSILON);
  }

  private static final class TestInputs extends MechanismRollerIOInputs {}
}
