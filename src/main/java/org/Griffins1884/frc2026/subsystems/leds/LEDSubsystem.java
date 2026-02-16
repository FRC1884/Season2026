// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.Griffins1884.frc2026.subsystems.leds;

import static edu.wpi.first.units.Units.Second;
import static org.Griffins1884.frc2026.subsystems.leds.LEDConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.subsystems.Superstructure.SuperState;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  private record LEDOutputValue(LEDPattern pattern, int index) {
    LEDOutputValue(LEDPattern pattern) {
      this(pattern, -1);
    }

    static LEDOutputValue[] all(LEDPattern pattern) {
      return new LEDOutputValue[] {new LEDOutputValue(pattern)};
    }
  }

  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(LEDIO io) {
    super();
    this.io = io;
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);
  }

  private void setPattern(LEDOutputValue[] values) {
    if (Arrays.stream(values).anyMatch(value -> value.index == -1)) {
      var opt = Arrays.stream(values).filter(value -> value.index == -1).findFirst();

      if (opt.isPresent()) {
        var value = opt.get();

        io.setAllPattern(value.pattern);
      }
      return;
    }

    for (LEDOutputValue value : values) {
      io.setPattern(value.index(), value.pattern());
    }
  }

  public Command ledCommand(
      BooleanSupplier isEnabled,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Optional<Pose2d>> autoStartPoseSupplier,
      Supplier<SuperState> currentStateSupplier,
      BooleanSupplier hasBallSupplier,
      Supplier<String> climbPhaseSupplier) {
    return this.run(
        () -> {
          if (!isEnabled.getAsBoolean()) {
            Pose2d realPose = robotPoseSupplier.get();
            Optional<Pose2d> targetPose = autoStartPoseSupplier.get();
            if (realPose != null && targetPose.isPresent()) {
              Logger.recordOutput("LED/Mode", "Disabled/Align");
              setPattern(getFieldAlignPattern(realPose, targetPose.get()));
            } else {
              Logger.recordOutput("LED/Mode", "Disabled/Idle");
              Logger.recordOutput("LED/SegmentMask", 0b1111);
              setPattern(LEDOutputValue.all(LEDPattern.solid(IDLE_COLOR)));
            }
            return;
          }
          SuperState state = currentStateSupplier.get();
          boolean hasBall = hasBallSupplier.getAsBoolean();
          String climbPhase = climbPhaseSupplier.get();
          Logger.recordOutput("LED/Mode", "Enabled");
          Logger.recordOutput("LED/SuperState", state == null ? "UNKNOWN" : state.name());
          Logger.recordOutput("LED/HasBall", hasBall);
          Logger.recordOutput("LED/ClimbPhase", climbPhase == null ? "" : climbPhase);
          setPattern(getTeleopPattern(state, hasBall, climbPhase));
        });
  }

  private LEDOutputValue[] getFieldAlignPattern(Pose2d realPose, Pose2d targetPose) {
    Pose2d realAlliance = toAllianceForwardPose(realPose);
    Pose2d targetAlliance = toAllianceForwardPose(targetPose);
    if (realAlliance == null || targetAlliance == null) {
      return LEDOutputValue.all(LEDPattern.solid(IDLE_COLOR));
    }

    Translation2d delta = targetAlliance.getTranslation().minus(realAlliance.getTranslation());
    double xError = delta.getX();
    double yError = delta.getY();

    if (Math.abs(xError) <= TRANSLATION_TOLERANCE && Math.abs(yError) <= TRANSLATION_TOLERANCE) {
      Logger.recordOutput("LED/Align/XError", xError);
      Logger.recordOutput("LED/Align/YError", yError);
      Logger.recordOutput("LED/Align/Aligned", true);
      Logger.recordOutput("LED/SegmentMask", 0b1111);
      Logger.recordOutput("LED/Pattern", "Align/Ok");
      return LEDOutputValue.all(LEDPattern.solid(ALIGN_OK_COLOR));
    }

    double dist = Math.min(delta.getNorm(), FLASHING_MAX);
    dist = Math.max(dist, 0.05);
    var blinkSpeed = Second.of(1 / (5 * dist));

    LEDPattern front =
        xError > TRANSLATION_TOLERANCE
            ? LEDPattern.solid(ALIGN_MORE_COLOR).blink(blinkSpeed)
            : LEDPattern.solid(Color.kBlack);
    LEDPattern back =
        xError < -TRANSLATION_TOLERANCE
            ? LEDPattern.solid(ALIGN_LESS_COLOR).blink(blinkSpeed)
            : LEDPattern.solid(Color.kBlack);
    LEDPattern left =
        yError > TRANSLATION_TOLERANCE
            ? LEDPattern.solid(ALIGN_MORE_COLOR).blink(blinkSpeed)
            : LEDPattern.solid(Color.kBlack);
    LEDPattern right =
        yError < -TRANSLATION_TOLERANCE
            ? LEDPattern.solid(ALIGN_LESS_COLOR).blink(blinkSpeed)
            : LEDPattern.solid(Color.kBlack);

    int segmentMask = 0;
    if (yError > TRANSLATION_TOLERANCE) {
      segmentMask |= 0b0001;
    }
    if (xError > TRANSLATION_TOLERANCE) {
      segmentMask |= 0b0010;
    }
    if (yError < -TRANSLATION_TOLERANCE) {
      segmentMask |= 0b0100;
    }
    if (xError < -TRANSLATION_TOLERANCE) {
      segmentMask |= 0b1000;
    }
    Logger.recordOutput("LED/Align/XError", xError);
    Logger.recordOutput("LED/Align/YError", yError);
    Logger.recordOutput("LED/Align/Aligned", false);
    Logger.recordOutput("LED/SegmentMask", segmentMask);
    Logger.recordOutput("LED/Pattern", "Align/Directional");

    return new LEDOutputValue[] {
      new LEDOutputValue(left, LEFT),
      new LEDOutputValue(front, FRONT),
      new LEDOutputValue(right, RIGHT),
      new LEDOutputValue(back, BACK)
    };
  }

  private LEDOutputValue[] getTeleopPattern(SuperState state, boolean hasBall, String climbPhase) {
    if (state == null) {
      Logger.recordOutput("LED/SegmentMask", 0b1111);
      Logger.recordOutput("LED/Pattern", "Enabled/Unknown");
      return LEDOutputValue.all(LEDPattern.solid(IDLE_COLOR));
    }

    if (state == SuperState.FERRYING) {
      Logger.recordOutput("LED/SegmentMask", 0b1111);
      Logger.recordOutput("LED/Pattern", "Enabled/Ferrying");
      return LEDOutputValue.all(LEDPattern.solid(FERRY_COLOR));
    }

    if (state == SuperState.ENDGAME_CLIMB
        || state == SuperState.AUTO_CLIMB
        || state == SuperState.CLIMB_DETACH) {
      Logger.recordOutput("LED/SegmentMask", 0b1111);
      Logger.recordOutput("LED/Pattern", "Enabled/Climb");
      return LEDOutputValue.all(getClimbPattern(climbPhase));
    }

    if (state == SuperState.IDLING) {
      Logger.recordOutput("LED/SegmentMask", 0b1111);
      Logger.recordOutput("LED/Pattern", "Enabled/Idle");
      return LEDOutputValue.all(LEDPattern.solid(IDLE_COLOR));
    }

    if (hasBall) {
      LEDPattern pattern = LEDPattern.solid(HAS_BALL_COLOR);
      if (!HAS_BALL_SOLID) {
        pattern = pattern.breathe(BREATHE_SPEED);
      }
      Logger.recordOutput("LED/SegmentMask", 0b1111);
      Logger.recordOutput("LED/Pattern", "Enabled/HasBall");
      return LEDOutputValue.all(pattern);
    }

    LEDPattern pattern = LEDPattern.solid(INTAKE_SHOOT_COLOR);
    if (NO_BALL_BREATHE) {
      pattern = pattern.breathe(BREATHE_SPEED);
    }
    Logger.recordOutput("LED/SegmentMask", 0b1111);
    Logger.recordOutput("LED/Pattern", "Enabled/NoBall");
    return LEDOutputValue.all(pattern);
  }

  private LEDPattern getClimbPattern(String climbPhase) {
    String phase = climbPhase == null ? "" : climbPhase.toUpperCase();
    if (phase.startsWith("DETACH")) {
      return LEDPattern.solid(CLIMB_DETACH_COLOR).blink(CLIMB_BLINK_SPEED);
    }
    if ("DONE".equals(phase)) {
      return LEDPattern.solid(CLIMB_DONE_COLOR);
    }
    return LEDPattern.solid(CLIMB_PULL_COLOR).blink(CLIMB_BLINK_SPEED);
  }

  private Pose2d toAllianceForwardPose(Pose2d pose) {
    if (pose == null) {
      return null;
    }
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.get() == Alliance.Blue) {
      return new Pose2d(pose.getTranslation(), new Rotation2d());
    }
    double x = GlobalConstants.FieldConstants.fieldLength - pose.getX();
    double y = GlobalConstants.FieldConstants.fieldWidth - pose.getY();
    return new Pose2d(x, y, new Rotation2d());
  }

  public void close() {
    io.close();
  }
}
