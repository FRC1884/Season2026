package org.Griffins1884.frc2026.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public final class SuperstructureConstants {
  public static final Translation2d HOPPER_TARGET_BLUE = new Translation2d(4.63, 4.03);
  public static final Translation2d HOPPER_TARGET_RED = new Translation2d(11.9, 4.03);
  public static final Translation2d FERRY_TARGET_BLUE = HOPPER_TARGET_BLUE;
  public static final Translation2d FERRY_TARGET_RED = HOPPER_TARGET_RED;

  public static final LoggedTunableNumber BALL_PRESENT_CURRENT_AMPS =
      new LoggedTunableNumber("Superstructure/BallCurrentAmps", 15.0);
  public static final LoggedTunableNumber BALL_PRESENCE_DEBOUNCE_SEC =
      new LoggedTunableNumber("Superstructure/BallPresenceDebounceSec", 0.15);
  public static final boolean AUTO_STOP_ON_EMPTY = true;

  public static final LoggedTunableNumber INTAKE_PIVOT_POSITION =
      new LoggedTunableNumber("Superstructure/IntakePivotPosition", 0.07);
  public static final LoggedTunableNumber SHOOTER_PIVOT_OFFSET =
      new LoggedTunableNumber("Superstructure/ShooterPivotOffset", 0.0);

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

  public static final int ENDGAME_CLIMB_LEVELS = 3;
  public static final int AUTO_CLIMB_LEVELS = 1;

  public static Translation2d getHopperTarget() {
    boolean isRed =
        (DriverStation.getAlliance().isPresent())
            ? DriverStation.Alliance.Red == DriverStation.getAlliance().get()
            : false;

    if (isRed) {
      return HOPPER_TARGET_RED;
    } else {
      return HOPPER_TARGET_BLUE;
    }
  }

  public static Translation2d getFerryTarget() {
    boolean isRed =
        (DriverStation.getAlliance().isPresent())
            ? DriverStation.Alliance.Red == DriverStation.getAlliance().get()
            : false;

    if (isRed) {
      return FERRY_TARGET_RED;
    } else {
      return FERRY_TARGET_BLUE;
    }
  }

  private SuperstructureConstants() {}
}
