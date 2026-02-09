package org.Griffins1884.frc2026.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import org.Griffins1884.frc2026.util.HubShiftTracker.HubStatus;
import org.Griffins1884.frc2026.util.HubShiftTracker.MatchTimeframe;
import org.junit.jupiter.api.Test;

public class HubShiftTrackerTest {
  @Test
  void teleopTimeframes_matchTableBoundaries() {
    assertEquals(
        MatchTimeframe.TRANSITION,
        HubShiftTracker.computeTimeframe(false, true, 135.0),
        "135s should be TRANSITION");
    assertEquals(
        MatchTimeframe.SHIFT_1,
        HubShiftTracker.computeTimeframe(false, true, 130.0),
        "130s should be SHIFT_1");
    assertEquals(
        MatchTimeframe.SHIFT_2,
        HubShiftTracker.computeTimeframe(false, true, 105.0),
        "105s should be SHIFT_2");
    assertEquals(
        MatchTimeframe.SHIFT_3,
        HubShiftTracker.computeTimeframe(false, true, 80.0),
        "80s should be SHIFT_3");
    assertEquals(
        MatchTimeframe.SHIFT_4,
        HubShiftTracker.computeTimeframe(false, true, 55.0),
        "55s should be SHIFT_4");
    assertEquals(
        MatchTimeframe.ENDGAME,
        HubShiftTracker.computeTimeframe(false, true, 30.0),
        "30s should be ENDGAME");
  }

  @Test
  void allianceShifts_alternateBasedOnAutoWinner_redWinner() {
    Optional<DriverStation.Alliance> ourAlliance = Optional.of(DriverStation.Alliance.Red);
    Optional<DriverStation.Alliance> winner = Optional.of(DriverStation.Alliance.Red);

    var s1 = HubShiftTracker.compute(false, true, 129.0, ourAlliance, winner, "R"); // SHIFT_1
    assertTrue(s1.hubStatusValid());
    assertEquals(HubStatus.INACTIVE, s1.redHubStatus());
    assertEquals(HubStatus.ACTIVE, s1.blueHubStatus());
    assertFalse(s1.ourHubActive());

    var s2 = HubShiftTracker.compute(false, true, 104.0, ourAlliance, winner, "R");
    assertTrue(s2.hubStatusValid());
    assertEquals(HubStatus.ACTIVE, s2.redHubStatus());
    assertEquals(HubStatus.INACTIVE, s2.blueHubStatus());
    assertTrue(s2.ourHubActive());

    var s3 = HubShiftTracker.compute(false, true, 79.0, ourAlliance, winner, "R");
    assertTrue(s3.hubStatusValid());
    assertEquals(HubStatus.INACTIVE, s3.redHubStatus());
    assertEquals(HubStatus.ACTIVE, s3.blueHubStatus());
    assertFalse(s3.ourHubActive());

    var s4 = HubShiftTracker.compute(false, true, 54.0, ourAlliance, winner, "R");
    assertTrue(s4.hubStatusValid());
    assertEquals(HubStatus.ACTIVE, s4.redHubStatus());
    assertEquals(HubStatus.INACTIVE, s4.blueHubStatus());
    assertTrue(s4.ourHubActive());
  }

  @Test
  void allianceShifts_unknownWinner_isInvalid() {
    Optional<DriverStation.Alliance> ourAlliance = Optional.of(DriverStation.Alliance.Blue);
    var snapshot = HubShiftTracker.compute(false, true, 129.0, ourAlliance, Optional.empty(), "");
    assertFalse(snapshot.hubStatusValid());
    assertEquals(HubStatus.UNKNOWN, snapshot.redHubStatus());
    assertEquals(HubStatus.UNKNOWN, snapshot.blueHubStatus());
    assertFalse(snapshot.ourHubActive());
  }
}
