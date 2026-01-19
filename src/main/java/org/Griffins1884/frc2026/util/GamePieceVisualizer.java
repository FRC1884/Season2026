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

package org.Griffins1884.frc2026.util;

import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.run;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** have fun :) */
public class GamePieceVisualizer {
  private static Supplier<Pose3d> blueSpeaker = Pose3d::new;
  private static final Transform3d defaultLauncherTransform =
      new Transform3d(-0.008, 0, 0.19, new Rotation3d(0.0, Units.degreesToRadians(-25.0), 0.0));
  private static Supplier<Pose2d> robotPoseSupplier = Pose2d::new;

  public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    robotPoseSupplier = supplier;
  }

  public static void setTargetSupplier(Supplier<Pose3d> supplier) {
    blueSpeaker = supplier;
  }

  public static Command shoot(
      DoubleSupplier shotDuration, Supplier<Transform3d> launcherTransform) {
    return new ScheduleCommand( // Branch off and exit immediately
        defer(
                () -> {
                  final Pose3d startPose =
                      new Pose3d(robotPoseSupplier.get()).transformBy(launcherTransform.get());
                  final Pose3d endPose = blueSpeaker.get();

                  final Timer timer = new Timer();
                  timer.start();
                  return run(() ->
                          Logger.recordOutput(
                              "GamePieceVisualizer",
                              new Pose3d[] {
                                startPose.interpolate(
                                    endPose, timer.get() / shotDuration.getAsDouble())
                              }))
                      .until(() -> timer.hasElapsed(shotDuration.getAsDouble()))
                      .finallyDo(() -> Logger.recordOutput("GamePieceVisualizer", new Pose3d[] {}));
                },
                Set.of())
            .ignoringDisable(true));
  }

  public static Command shoot(DoubleSupplier shotDuration, DoubleSupplier launcherPitch) {
    return shoot(
        shotDuration,
        () ->
            new Transform3d(
                defaultLauncherTransform.getTranslation(),
                new Rotation3d(
                    defaultLauncherTransform.getRotation().getX(),
                    -launcherPitch.getAsDouble(),
                    defaultLauncherTransform.getRotation().getZ())));
  }

  public static Command shoot(DoubleSupplier shotDuration) {
    return shoot(
        shotDuration,
        () ->
            (-Math.atan2(
                blueSpeaker.get().getZ(),
                blueSpeaker
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(
                        robotPoseSupplier
                            .get()
                            .transformBy(
                                new Transform2d(
                                    defaultLauncherTransform
                                        .getTranslation()
                                        .toTranslation2d()
                                        .getX(),
                                    defaultLauncherTransform
                                        .getTranslation()
                                        .toTranslation2d()
                                        .getY(),
                                    defaultLauncherTransform
                                        .getTranslation()
                                        .toTranslation2d()
                                        .getAngle()))
                            .getTranslation()))));
  }
}
