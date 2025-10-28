package frc.robot.subsystems.objectivetracker;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ReefControlsIOServerTest {
  private NetworkTableInstance instance;

  @BeforeEach
  void resetInstance() {
    instance = NetworkTableInstance.getDefault();
    instance.stopServer();
    instance.stopClient();
    instance.stopLocal();
    instance.startLocal();
  }

  @Test
  void serverPublishesOutputsAndReadsInputs() {
    ReefControlsIOServer server = new ReefControlsIOServer();

    var dashboardTable = instance.getTable("/ReefControls/ToDashboard");
    var selectedLevelSub = dashboardTable.getIntegerTopic("SelectedLevel").subscribe(0);
    var level1Sub = dashboardTable.getIntegerTopic("Level1").subscribe(0);
    var level2Sub = dashboardTable.getIntegerTopic("Level2").subscribe(0);
    var level3Sub = dashboardTable.getIntegerTopic("Level3").subscribe(0);
    var level4Sub = dashboardTable.getIntegerTopic("Level4").subscribe(0);
    var algaeSub = dashboardTable.getIntegerTopic("Algae").subscribe(0);
    var coopSub = dashboardTable.getBooleanTopic("Coop").subscribe(false);
    var isElimsSub = dashboardTable.getBooleanTopic("IsElims").subscribe(false);

    server.setSelectedLevel(2);
    server.setLevel1State(3);
    server.setLevel2State(4);
    server.setLevel3State(5);
    server.setLevel4State(6);
    server.setAlgaeState(7);
    server.setCoopState(true);
    server.setElims(true);

    instance.flushLocal();

    assertEquals(2, selectedLevelSub.get());
    assertEquals(3, level1Sub.get());
    assertEquals(4, level2Sub.get());
    assertEquals(5, level3Sub.get());
    assertEquals(6, level4Sub.get());
    assertEquals(7, algaeSub.get());
    assertTrue(coopSub.get());
    assertTrue(isElimsSub.get());

    var robotTable = instance.getTable("/ReefControls/ToRobot");
    var selectedLevelPub = robotTable.getIntegerTopic("SelectedLevel").publish();
    var level1Pub = robotTable.getIntegerTopic("Level1").publish();
    var level2Pub = robotTable.getIntegerTopic("Level2").publish();
    var level3Pub = robotTable.getIntegerTopic("Level3").publish();
    var level4Pub = robotTable.getIntegerTopic("Level4").publish();
    var algaePub = robotTable.getIntegerTopic("Algae").publish();
    var coopPub = robotTable.getBooleanTopic("Coop").publish();

    selectedLevelPub.set(1);
    level1Pub.set(11);
    level2Pub.set(12);
    level3Pub.set(13);
    level4Pub.set(14);
    algaePub.set(15);
    coopPub.set(true);

    instance.flushLocal();

    var inputs = new ReefControlsIO.ReefControlsIOInputs();
    server.updateInputs(inputs);

    assertArrayEquals(new int[] {1}, inputs.selectedLevel);
    assertArrayEquals(new int[] {11}, inputs.level1State);
    assertArrayEquals(new int[] {12}, inputs.level2State);
    assertArrayEquals(new int[] {13}, inputs.level3State);
    assertArrayEquals(new int[] {14}, inputs.level4State);
    assertArrayEquals(new int[] {15}, inputs.algaeState);
    assertArrayEquals(new boolean[] {true}, inputs.coopState);
  }
}
