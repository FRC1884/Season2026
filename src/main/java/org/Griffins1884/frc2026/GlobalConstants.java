// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.Griffins1884.frc2026;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.loadField;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;
import java.nio.file.Path;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class GlobalConstants {
  public static final RobotMode MODE = RobotMode.SIM;
  public static final RobotType ROBOT = RobotType.SIMBOT;
  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final RobotSwerveMotors robotSwerveMotors = RobotSwerveMotors.FULLKRACKENS;

  public static boolean TUNING_MODE = true;

  public static enum RobotMode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY,
  }

  public static enum RobotType {
    COMPBOT,
    DEVBOT,
    SIMBOT,
    CRESCENDO
  }

  public static enum RobotSwerveMotors {
    FULLSPARK,
    HALFSPARK,
    FULLKRACKENS
  }

  /**
   * Checks whether the correct robot is selected when deploying (the main method is only ever
   * called on deploy).
   */
  public static void main(String... args) {
    if (ROBOT == RobotType.SIMBOT) {
      new Alert("SIM robot loaded in REAL mode, gains likely breaking!", AlertType.kWarning)
          .set(true);
    }
  }

  public static final class Coordinates {
    public static final Translation2d BLUE_HUB = new Translation2d(4.6, 4);
    public static final Translation2d RED_HUB = new Translation2d(11.9, 4);
    public static final Translation2d RED_HOPPER_CENTER = new Translation2d(11.9, 4.03);
    public static final Translation2d BLUE_HOPPER_CENTER = new Translation2d(4.63, 4.03);
    public static final Translation2d BLUE_CLIMB_CENTER = new Translation2d(1, 3.7);
    public static final Translation2d RED_CLIMB_CENTER = new Translation2d(15.5, 4.3);
    public static final Translation2d BLUE_HUMAN_PLAYER_OUTPUT_CENTER = new Translation2d(0.1, 0.7);
    public static final Translation2d RED_HUMAN_PLAYER_OUTPUT_CENTER = new Translation2d(16.4, 7.4);
    public static final Translation2d BLUE_FERRY = new Translation2d(3.0, 7.0);
    public static final Translation2d RED_FERRY = new Translation2d(13.5, 7.0);

    // TODO: Add more accurate coords, I hand selected the centres for these coordinates.
    // TODO: check if x and y coordinates are swapped because of Choreo and odo inconsistencies

    public static Translation2d getHopperTarget() {
      boolean isRed =
          DriverStation.getAlliance().isPresent()
              && DriverStation.Alliance.Red == DriverStation.getAlliance().get();

      if (isRed) {
        return RED_HOPPER_CENTER;
      } else {
        return BLUE_HOPPER_CENTER;
      }
    }

    public static Translation2d getClimbTarget() {
      boolean isRed =
          DriverStation.getAlliance().isPresent()
              && DriverStation.Alliance.Red == DriverStation.getAlliance().get();

      if (isRed) {
        return RED_CLIMB_CENTER;
      } else {
        return BLUE_CLIMB_CENTER;
      }
    }

    public static Translation2d getHumanPlayerOutputCenter() {
      boolean isRed =
          DriverStation.getAlliance().isPresent()
              && DriverStation.Alliance.Red == DriverStation.getAlliance().get();

      if (isRed) {
        return RED_HUMAN_PLAYER_OUTPUT_CENTER;
      } else {
        return BLUE_HUMAN_PLAYER_OUTPUT_CENTER;
      }
    }

    public static Translation2d getHub() {
      boolean isRed =
          DriverStation.getAlliance().isPresent()
              && DriverStation.Alliance.Red == DriverStation.getAlliance().get();

      if (isRed) {
        return RED_HUB;
      } else {
        return BLUE_HUB;
      }
    }

    public static Translation2d getFerryTarget(Pose2d drive) {
      boolean isRed =
          DriverStation.getAlliance().isPresent()
              && DriverStation.Alliance.Red == DriverStation.getAlliance().get();
      boolean isTop = drive.getY() >= 4.0;

      if (isRed) {
        return isTop ? RED_FERRY : RED_FERRY.minus(new Translation2d(0.0, 4.0));
      } else {
        return isTop ? BLUE_FERRY : BLUE_FERRY.minus(new Translation2d(0.0, 4.0));
      }
    }
  }

  // Blue origin, so we use blue side coords and tags
  public static final class FieldMap {
    public static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;
    public static final boolean WELDED_FIELD = true;
    public static final boolean COMP_FIELD = true;

    static {
      try {
        // Path reads from the working directory, and splices at `/src/main/deploy/`
        APRIL_TAG_FIELD_LAYOUT =
            COMP_FIELD
                ? loadField(
                    WELDED_FIELD
                        ? AprilTagFields.k2026RebuiltWelded
                        : AprilTagFields.k2026RebuiltAndymark)
                : new AprilTagFieldLayout(Path.of("tagfields/home_testing_1.json"));
      } catch (IOException e) {
        new Alert("Custom tag map not found, using default layout!", AlertType.kWarning).set(true);
        APRIL_TAG_FIELD_LAYOUT =
            loadField(
                WELDED_FIELD
                    ? AprilTagFields.k2026RebuiltWelded
                    : AprilTagFields.k2026RebuiltAndymark);
      }
      // APRIL_TAG_FIELD_LAYOUT = new AprilTagFieldLayout(Path.of("tagfields/home_testing_1.json"));
    }

    public static final double FIELD_WIDTH_METERS = Units.feetToMeters(26 + (5.0 / 12));
    public static final double FIELD_LENGTH_METERS = Units.feetToMeters(57 + (6.875 / 12));
  }

  public static final class AlignOffsets {}

  /** PID + FF gains, with overloaded constructors for disabling each term. */
  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    public Gains(double kP, double kI, double kD) {
      this(kP, kI, kD, 0, 0, 0, 0);
    }
  }
}
