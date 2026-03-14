package org.Griffins1884.frc2026.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.Griffins1884.frc2026.GlobalConstants.FieldConstants;

/**
 * Utility functions for flipping from the blue (default) to red alliance for mirrored fields. All
 * credit goes to team 5712.
 */
public class AllianceFlipUtil {
  private static Optional<Alliance> cachedAlliance = Optional.empty();

  private AllianceFlipUtil() {}

  /** Returns the current DriverStation alliance, updating the cache when available. */
  public static Optional<Alliance> getAlliance() {
    Optional<Alliance> liveAlliance = DriverStation.getAlliance();
    liveAlliance.ifPresent(alliance -> cachedAlliance = Optional.of(alliance));
    return liveAlliance;
  }

  /** Returns DriverStation alliance or the last known alliance if DS is temporarily unknown. */
  public static Optional<Alliance> getAllianceWithFallback() {
    Optional<Alliance> liveAlliance = getAlliance();
    return liveAlliance.isPresent() ? liveAlliance : cachedAlliance;
  }

  /** Infers alliance from field pose when DS alliance is unavailable. */
  public static Optional<Alliance> inferAllianceFromPose(Pose2d referencePose) {
    if (referencePose == null || !Double.isFinite(referencePose.getX())) {
      return Optional.empty();
    }
    return Optional.of(
        referencePose.getX() <= FieldConstants.fieldLength * 0.5 ? Alliance.Blue : Alliance.Red);
  }

  /**
   * Resolves alliance from DS first, then from cache, then by pose hint.
   *
   * @param referencePose Pose hint used only if DS/cache are unavailable.
   */
  public static Optional<Alliance> resolveAlliance(Pose2d referencePose) {
    Optional<Alliance> alliance = getAllianceWithFallback();
    return alliance.isPresent() ? alliance : inferAllianceFromPose(referencePose);
  }

  /** Returns true when alliance is known from DS or cache. */
  public static boolean isAllianceKnown() {
    return getAllianceWithFallback().isPresent();
  }

  /** Clears cached alliance (useful in tests). */
  public static void clearCachedAlliance() {
    cachedAlliance = Optional.empty();
  }

  /**
   * Flips an x coordinate to the correct side of the field based on the current alliance color.
   *
   * @param xCoordinate The x coordinate to be flipped.
   * @return The flipped x coordinate.
   */
  public static double apply(double xCoordinate) {
    if (shouldFlip()) {
      return FieldConstants.fieldLength - xCoordinate;
    } else {
      return xCoordinate;
    }
  }

  /**
   * Flips a translation to the correct side of the field based on the current alliance color.
   *
   * @param translation The translation to be flipped.
   * @return The flipped translation.
   */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(apply(translation.getX()), translation.getY());
    } else {
      return translation;
    }
  }

  /**
   * Flips a 3D translation to the correct side of the field based on the current alliance color.
   *
   * @param translation The 3D translation to be flipped across the XY-plane.
   * @return The flipped 3D translation.
   */
  public static Translation3d apply(Translation3d translation) {
    if (shouldFlip()) {
      return new Translation3d(apply(translation.getX()), translation.getY(), translation.getZ());
    } else {
      return translation;
    }
  }

  /**
   * Flips a rotation based on the current alliance color.
   *
   * @param rotation The rotation to be flipped.
   * @return The flipped rotation.
   */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance color.
   *
   * @param pose The pose to be flipped.
   * @return The flipped pose.
   */
  public static Pose2d apply(Pose2d pose) {
    if (pose == null) {
      return null;
    }
    if (!shouldFlip(pose)) {
      return pose;
    }
    Translation2d flippedTranslation =
        new Translation2d(
            FieldConstants.fieldLength - pose.getX(), FieldConstants.fieldWidth - pose.getY());
    Rotation2d flippedRotation =
        new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin());
    return new Pose2d(flippedTranslation, flippedRotation);
  }

  /**
   * Flips a 3D pose to the correct side of the field based on the current alliance color.
   *
   * @param pose The 3D pose to be flipped across the XY-plane.
   * @return The flipped 3D pose.
   */
  public static Pose3d apply(Pose3d pose) {
    if (shouldFlip()) {
      return new Pose3d(apply(pose.getTranslation()), pose.getRotation());
    } else {
      return pose;
    }
  }

  /**
   * Checks if the alliance color should be flipped.
   *
   * @return True if the alliance color should be flipped, false otherwise.
   */
  public static boolean shouldFlip() {
    Optional<Alliance> alliance = getAllianceWithFallback();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  /** Uses a pose hint when DS alliance is unavailable. */
  public static boolean shouldFlip(Pose2d referencePose) {
    Optional<Alliance> alliance = resolveAlliance(referencePose);
    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }
}
