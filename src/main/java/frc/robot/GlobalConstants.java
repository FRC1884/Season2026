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

package frc.robot;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.loadField;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
