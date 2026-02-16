package org.Griffins1884.frc2026.subsystems;

import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public final class SuperstructureConstants {
  public static final LoggedTunableNumber BALL_PRESENT_CURRENT_AMPS =
      new LoggedTunableNumber("Superstructure/BallCurrentAmps", 15.0);
  public static final LoggedTunableNumber BALL_PRESENCE_DEBOUNCE_SEC =
      new LoggedTunableNumber("Superstructure/BallPresenceDebounceSec", 0.15);
  public static final boolean AUTO_STOP_ON_EMPTY = false;
  public static final LoggedTunableNumber CLIMB_EXTEND_HEIGHT_METERS =
      new LoggedTunableNumber("Superstructure/ClimbExtendHeightMeters", 1.0);
  public static final LoggedTunableNumber CLIMB_LATCH_HEIGHT_METERS =
      new LoggedTunableNumber("Superstructure/ClimbLatchHeightMeters", 0.8);
  public static final LoggedTunableNumber CLIMB_PULL_HEIGHT_METERS =
      new LoggedTunableNumber("Superstructure/ClimbPullHeightMeters", 0.9);
  public static final LoggedTunableNumber CLIMB_DYNAMIC_HEIGHT_METERS =
      new LoggedTunableNumber("Superstructure/ClimbDynamicHeightMeters", 1.0);
  public static final LoggedTunableNumber CLIMB_DETACH_EXTEND_HEIGHT_METERS =
      new LoggedTunableNumber("Superstructure/ClimbDetachExtendHeightMeters", 1.0);
  public static final LoggedTunableNumber CLIMB_DETACH_RETRACT_HEIGHT_METERS =
      new LoggedTunableNumber("Superstructure/ClimbDetachRetractHeightMeters", 0.0);
  public static final LoggedTunableNumber CLIMB_DETACH_DRIVE_SECONDS =
      new LoggedTunableNumber("Superstructure/ClimbDetachDriveSeconds", 1.0);

  public static final int ENDGAME_CLIMB_LEVELS = 1;
  public static final int AUTO_CLIMB_LEVELS = 1;

  public static final boolean SHOOTING_WHILE_MOVING = false;
  public static final double MANUAL_JOG_VOLTAGE = 0.5;
  public static final double AUTO_STATE_SHOOTING_X_MAX_METERS = 4.0;
  public static final double AUTO_STATE_IDLE_X_MAX_METERS = 5.4;
  public static final double AUTO_STATE_INTAKE_X_MAX_METERS = 11.0;

  private SuperstructureConstants() {}
}
