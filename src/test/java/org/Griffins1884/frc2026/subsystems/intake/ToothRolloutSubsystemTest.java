package org.Griffins1884.frc2026.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.Griffins1884.frc2026.mechanisms.MechanismDefinition;
import org.Griffins1884.frc2026.mechanisms.RobotMechanismDefinitions;
import org.Griffins1884.frc2026.mechanisms.rollers.MechanismRollerIOInputsAutoLogged;
import org.Griffins1884.frc2026.subsystems.intake.ToothRolloutSubsystem.ToothRolloutGoal;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ToothRolloutSubsystemTest {
  private static final double EPSILON = 1e-9;

  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private RecordingIO io;
  private ToothRolloutSubsystem subsystem;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void setUp() {
    scheduler.cancelAll();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    io = new RecordingIO();
    subsystem = new ToothRolloutSubsystem("ToothRolloutTest", io);
  }

  @AfterEach
  void tearDown() {
    scheduler.cancelAll();
    scheduler.unregisterSubsystem(subsystem);
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  @Test
  void goalsMapToDeterministicOutputsAndNullStops() {
    assertEquals(ToothRolloutGoal.STOPPED, subsystem.getGoal());

    assertGoalOutput(ToothRolloutGoal.INTAKE_FEED, 1.0);
    assertGoalOutput(ToothRolloutGoal.EJECT_REVERSE, -1.0);
    assertGoalOutput(ToothRolloutGoal.STOPPED, 0.0);

    subsystem.setGoal(null);
    subsystem.periodic();
    assertEquals(ToothRolloutGoal.STOPPED, subsystem.getGoal());
    assertEquals(0.0, io.commandedVolts, EPSILON);
  }

  @Test
  void disabledRobotAndExplicitStopForceZeroOutput() {
    assertGoalOutput(ToothRolloutGoal.INTAKE_FEED, 1.0);

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    subsystem.periodic();
    assertEquals(0.0, io.commandedVolts, EPSILON);

    subsystem.setManualVoltage(1.0);
    subsystem.stop();
    assertEquals(ToothRolloutGoal.STOPPED, subsystem.getGoal());
    assertEquals(0.0, io.commandedVolts, EPSILON);
    assertEquals(
        org.Griffins1884.frc2026.mechanisms.rollers.VoltageRollerMechanism.ControlMode.GOAL,
        subsystem.getControlMode());

    assertTrue(subsystem.stopCommand().getRequirements().contains(subsystem));
  }

  @Test
  void heldCommandsRequireSubsystemInterruptEachOtherAndStopOnCancellation() {
    Command intakeFeed = subsystem.intakeFeedCommand();
    Command ejectReverse = subsystem.ejectReverseCommand();

    assertTrue(intakeFeed.getRequirements().contains(subsystem));
    assertTrue(ejectReverse.getRequirements().contains(subsystem));

    scheduler.schedule(intakeFeed);
    scheduler.run();
    assertTrue(intakeFeed.isScheduled());
    assertEquals(ToothRolloutGoal.INTAKE_FEED, subsystem.getGoal());

    scheduler.schedule(ejectReverse);
    scheduler.run();
    assertFalse(intakeFeed.isScheduled());
    assertTrue(ejectReverse.isScheduled());
    assertEquals(ToothRolloutGoal.EJECT_REVERSE, subsystem.getGoal());

    scheduler.cancel(ejectReverse);
    scheduler.run();
    assertFalse(ejectReverse.isScheduled());
    assertEquals(ToothRolloutGoal.STOPPED, subsystem.getGoal());
    assertEquals(0.0, io.commandedVolts, EPSILON);
  }

  @Test
  void simulationProvidesDeterministicTelemetryAndRejectsNonFiniteInput() {
    ToothRolloutIOSim sim = new ToothRolloutIOSim();
    MechanismRollerIOInputsAutoLogged inputs = new MechanismRollerIOInputsAutoLogged();

    sim.runVolts(1.0);
    sim.updateInputs(inputs);
    assertTrue(inputs.connected[0]);
    assertEquals(1.0, inputs.appliedVoltage, EPSILON);
    assertTrue(inputs.positionRads > 0.0);
    assertTrue(inputs.velocityRadsPerSec > 0.0);

    sim.runVolts(-1.0);
    sim.updateInputs(inputs);
    assertEquals(-1.0, inputs.appliedVoltage, EPSILON);
    assertTrue(inputs.velocityRadsPerSec < 0.0);

    sim.runVolts(Double.NaN);
    sim.updateInputs(inputs);
    assertEquals(0.0, inputs.appliedVoltage, EPSILON);
    sim.runVolts(Double.POSITIVE_INFINITY);
    sim.updateInputs(inputs);
    assertEquals(0.0, inputs.appliedVoltage, EPSILON);
  }

  @Test
  void definitionIsSimulationOnlyOpenLoopRoller() {
    MechanismDefinition definition = RobotMechanismDefinitions.TOOTH_ROLLOUT;

    assertEquals(MechanismDefinition.MechanismType.ROLLER, definition.mechanismType());
    assertEquals(
        MechanismDefinition.MotorControllerType.SIMULATION_ONLY,
        definition.config().motorGroup().controllerType());
    assertTrue(definition.config().supportsOpenLoop());
    assertFalse(definition.config().supportsClosedLoop());
    assertFalse(definition.config().supportsCharacterization());
    assertTrue(definition.simulation().enabled());
  }

  private void assertGoalOutput(ToothRolloutGoal goal, double expectedVolts) {
    subsystem.setGoal(goal);
    subsystem.periodic();
    assertEquals(expectedVolts, io.commandedVolts, EPSILON);
  }

  private static final class RecordingIO implements ToothRolloutIO {
    private double commandedVolts;

    @Override
    public void updateInputs(MechanismRollerIOInputs inputs) {
      inputs.connected = new boolean[] {true};
      inputs.appliedVoltage = commandedVolts;
    }

    @Override
    public void runVolts(double volts) {
      commandedVolts = volts;
    }

    @Override
    public void stop() {
      commandedVolts = 0.0;
    }
  }
}
