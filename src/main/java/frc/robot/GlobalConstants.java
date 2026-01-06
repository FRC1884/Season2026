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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class GlobalConstants {
  public static final RobotMode MODE = RobotMode.REAL;
  public static final RobotType ROBOT = RobotType.DEVBOT;
  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final RobotSwerveMotors robotSwerveMotors = RobotSwerveMotors.FULLKRACKENS;

  public static boolean TUNING_MODE = false;

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
    public static final boolean WELDED_FIELD = false;
    public static final boolean COMP_FIELD = true;

    static {
      try {
        // Path reads from the working directory, and splices at `/src/main/deploy/`
        APRIL_TAG_FIELD_LAYOUT =
            COMP_FIELD
                ? loadField(
                    WELDED_FIELD
                        ? AprilTagFields.k2025ReefscapeWelded
                        : AprilTagFields.k2025ReefscapeAndyMark)
                : new AprilTagFieldLayout(Path.of("tagfields/home_testing_1.json"));
      } catch (IOException e) {
        new Alert("Custom tag map not found, using default layout!", AlertType.kWarning).set(true);
        APRIL_TAG_FIELD_LAYOUT =
            loadField(
                WELDED_FIELD
                    ? AprilTagFields.k2025ReefscapeWelded
                    : AprilTagFields.k2025ReefscapeAndyMark);
      }
      // APRIL_TAG_FIELD_LAYOUT = new AprilTagFieldLayout(Path.of("tagfields/home_testing_1.json"));
    }

    // TODO set all alignment offsets from tags
    @Getter
    public static enum Coordinates {
      REEF_1(18, 7),
      REEF_2(17, 8),
      REEF_3(22, 9),
      REEF_4(21, 10),
      REEF_5(20, 11),
      REEF_6(19, 6),
      LEFT_CORAL_STATION(13, 1),
      RIGHT_CORAL_STATION(12, 2),
      PROCESSOR(16, 3),
      LEFT_BARGE(14, 4),
      RIGHT_BARGE(15, 5);

      private final Pose2d bluePose;
      private final Pose2d redPose;

      Coordinates(int blueTagId, int redTagId) {
        this.bluePose = APRIL_TAG_FIELD_LAYOUT.getTagPose(blueTagId).get().toPose2d();
        this.redPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(redTagId).get().toPose2d();
      }

      public Pose2d getPose() {
        boolean isRed =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        return isRed ? redPose : bluePose;
      }
    }

    public static final double FIELD_WIDTH_METERS = Units.feetToMeters(26 + (5.0 / 12));
    public static final double FIELD_LENGTH_METERS = Units.feetToMeters(57 + (6.875 / 12));
  }

  public static final class AlignOffsets {
    public static final double BUMPER_TO_CENTER_OFFSET =
        Units.inchesToMeters(ROBOT == RobotType.DEVBOT ? -(26.0 / 2 + 3) : -(28.0 / 2 + 3.5));

    public static final double REEF_TO_BUMPER_OFFSET = -32.0 / 100;
    public static final double REEF_TO_BRANCH_OFFSET = Units.inchesToMeters(13.0 / 2);
    public static final double SOURCE_TO_TAG_STANDOFF = Units.inchesToMeters(40.0);

    public static final double SIDE_TO_SIDE_OFFSET_AUTO = Units.feetToMeters(1);
  }

  /** PID + FF gains, with overloaded constructors for disabling each term. */
  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    public Gains(double kP, double kI, double kD) {
      this(kP, kI, kD, 0, 0, 0, 0);
    }
  }
}
