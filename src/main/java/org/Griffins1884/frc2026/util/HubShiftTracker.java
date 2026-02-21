package org.Griffins1884.frc2026.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Locale;
import java.util.Optional;

/**
 * Implements 2026 REBUILT hub-active shifting rules.
 *
 * <p>Manual summary:
 *
 * <ul>
 *   <li>AUTO, TRANSITION SHIFT, and END GAME: both hubs active.
 *   <li>SHIFT 1-4: exactly one hub is active.
 *   <li>The alliance that scored more fuel in AUTO has their hub inactive for SHIFT 1.
 *   <li>Hub activeness alternates at the start of each subsequent alliance shift.
 * </ul>
 */
public final class HubShiftTracker {
  public enum MatchTimeframe {
    AUTO,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    ENDGAME,
    UNKNOWN
  }

  public enum HubStatus {
    ACTIVE,
    INACTIVE,
    UNKNOWN
  }

  public record Snapshot(
      MatchTimeframe timeframe,
      boolean hubStatusValid,
      Optional<DriverStation.Alliance> autoWinner,
      HubStatus redHubStatus,
      HubStatus blueHubStatus,
      Optional<DriverStation.Alliance> ourAlliance,
      HubStatus ourHubStatus,
      boolean ourHubActive,
      String gameDataRaw,
      String recommendation) {}

  private HubShiftTracker() {}

  public static Snapshot fromDriverStation() {
    String gameData = DriverStation.getGameSpecificMessage();
    Optional<DriverStation.Alliance> autoWinner = parseAutoWinner(gameData);
    return compute(
        DriverStation.isAutonomous(),
        DriverStation.isTeleop(),
        DriverStation.getMatchTime(),
        DriverStation.getAlliance(),
        autoWinner,
        gameData);
  }

  public static Snapshot compute(
      boolean isAutonomous,
      boolean isTeleop,
      double matchTimeSeconds,
      Optional<DriverStation.Alliance> ourAlliance,
      Optional<DriverStation.Alliance> autoWinner,
      String gameDataRaw) {
    MatchTimeframe timeframe = computeTimeframe(isAutonomous, isTeleop, matchTimeSeconds);

    HubStatus red = HubStatus.UNKNOWN;
    HubStatus blue = HubStatus.UNKNOWN;
    boolean valid = false;

    if (timeframe == MatchTimeframe.AUTO
        || timeframe == MatchTimeframe.TRANSITION
        || timeframe == MatchTimeframe.ENDGAME) {
      red = HubStatus.ACTIVE;
      blue = HubStatus.ACTIVE;
      valid = true;
    } else if (timeframe == MatchTimeframe.SHIFT_1
        || timeframe == MatchTimeframe.SHIFT_2
        || timeframe == MatchTimeframe.SHIFT_3
        || timeframe == MatchTimeframe.SHIFT_4) {
      if (autoWinner.isPresent()) {
        int shiftNumber =
            switch (timeframe) {
              case SHIFT_1 -> 1;
              case SHIFT_2 -> 2;
              case SHIFT_3 -> 3;
              case SHIFT_4 -> 4;
              default -> 0;
            };
        boolean winnerInactiveThisShift = (shiftNumber % 2) == 1; // winner inactive on SHIFT 1/3
        if (autoWinner.get() == DriverStation.Alliance.Red) {
          red = winnerInactiveThisShift ? HubStatus.INACTIVE : HubStatus.ACTIVE;
          blue = winnerInactiveThisShift ? HubStatus.ACTIVE : HubStatus.INACTIVE;
        } else {
          blue = winnerInactiveThisShift ? HubStatus.INACTIVE : HubStatus.ACTIVE;
          red = winnerInactiveThisShift ? HubStatus.ACTIVE : HubStatus.INACTIVE;
        }
        valid = true;
      }
    }

    HubStatus ourHub = HubStatus.UNKNOWN;
    if (ourAlliance.isPresent()) {
      ourHub = ourAlliance.get() == DriverStation.Alliance.Red ? red : blue;
    }
    boolean ourHubActive = ourHub == HubStatus.ACTIVE;

    String recommendation = computeRecommendation(timeframe, ourHub, valid);

    return new Snapshot(
        timeframe,
        valid,
        autoWinner,
        red,
        blue,
        ourAlliance,
        ourHub,
        ourHubActive,
        gameDataRaw == null ? "" : gameDataRaw,
        recommendation);
  }

  public static MatchTimeframe computeTimeframe(
      boolean isAutonomous, boolean isTeleop, double matchTimeSeconds) {
    if (isAutonomous) {
      return MatchTimeframe.AUTO;
    }
    if (!isTeleop || !Double.isFinite(matchTimeSeconds) || matchTimeSeconds < 0.0) {
      return MatchTimeframe.UNKNOWN;
    }

    // TELEOP timer values from the manual:
    // Transition: 2:20-2:10 (140-130)
    // Shift1: 2:10-1:45 (130-105)
    // Shift2: 1:45-1:20 (105-80)
    // Shift3: 1:20-0:55 (80-55)
    // Shift4: 0:55-0:30 (55-30)
    // Endgame: 0:30-0:00 (30-0)
    if (matchTimeSeconds > 130.0) return MatchTimeframe.TRANSITION;
    if (matchTimeSeconds > 105.0) return MatchTimeframe.SHIFT_1;
    if (matchTimeSeconds > 80.0) return MatchTimeframe.SHIFT_2;
    if (matchTimeSeconds > 55.0) return MatchTimeframe.SHIFT_3;
    if (matchTimeSeconds > 30.0) return MatchTimeframe.SHIFT_4;
    return MatchTimeframe.ENDGAME;
  }

  public static Optional<DriverStation.Alliance> parseAutoWinner(String gameSpecificMessage) {
    if (gameSpecificMessage == null) {
      return Optional.empty();
    }
    String raw = gameSpecificMessage.trim();
    if (raw.isEmpty()) {
      return Optional.empty();
    }
    String upper = raw.toUpperCase(Locale.ROOT);

    // Common cases are "R" or "B". Be conservative to avoid false positives.
    boolean mentionsRed =
        upper.equals("R")
            || upper.equals("RED")
            || upper.contains(" RED ")
            || upper.startsWith("RED")
            || upper.endsWith("RED")
            || upper.contains("RED:")
            || upper.contains("RED=");
    boolean mentionsBlue =
        upper.equals("B")
            || upper.equals("BLUE")
            || upper.contains(" BLUE ")
            || upper.startsWith("BLUE")
            || upper.endsWith("BLUE")
            || upper.contains("BLUE:")
            || upper.contains("BLUE=");

    if (mentionsRed == mentionsBlue) {
      return Optional.empty();
    }
    return Optional.of(mentionsRed ? DriverStation.Alliance.Red : DriverStation.Alliance.Blue);
  }

  private static String computeRecommendation(
      MatchTimeframe timeframe, HubStatus ourHub, boolean hubStatusValid) {
    if (timeframe == MatchTimeframe.UNKNOWN) {
      return "UNKNOWN";
    }
    if (timeframe == MatchTimeframe.ENDGAME) {
      return "SCORE";
    }
    if (!hubStatusValid) {
      return "CHECK FMS";
    }
    return ourHub == HubStatus.ACTIVE ? "SCORE" : "COLLECT/FERRY/DEFEND";
  }
}
