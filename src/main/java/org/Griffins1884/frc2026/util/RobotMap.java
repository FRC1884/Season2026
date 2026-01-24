package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotMap {
  public static final class Coordinates {
    public static final Pose2d BLUE_HUB_POSE2D = new Pose2d(4.6, 4, new Rotation2d());
    public static final Pose2d RED_HUB_POSE2D = new Pose2d(11.9, 4, new Rotation2d());
    public static final Pose2d RED_HOPPER_CENTER_POSE2D = new Pose2d(16.2, 2.1, new Rotation2d());
    public static final Pose2d BLUE_HOPPER_CENTER_POSE2D = new Pose2d(0.3, 5.9, new Rotation2d());
    public static final Pose2d BLUE_CLIMB_CENTER = new Pose2d(1, 3.7, new Rotation2d());
    public static final Pose2d RED_CLIMB_CENTER = new Pose2d(15.5, 4.3, new Rotation2d());
    public static final Pose2d BLUE_HUMAN_PLAYER_OUTPUT_CENTER_POSE2D =
        new Pose2d(0.1, 0.7, new Rotation2d());
    public static final Pose2d RED_HUMAN_PLAYER_OUTPUT_CENTER_POSE2D =
        new Pose2d(16.4, 7.4, new Rotation2d());

    // TODO: Add more accurate coords, I hand selected the centres for these coordinates.
    // TODO: check if x and y coordinates are swapped because of Choreo and odo inconsistencies

  }
}
