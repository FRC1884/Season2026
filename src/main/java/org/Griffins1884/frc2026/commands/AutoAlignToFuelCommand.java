package org.Griffins1884.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Auto-aligns the robot to a fuel cluster using Limelight game-piece detection.
 *
 * <p>Behavior: - Locks onto a target (prevents switching) - Drives forward constantly - Rotates
 * continuously to keep tx centered
 */
public class AutoAlignToFuelCommand extends Command {

  // ---------------- Tuning constants ----------------

  private static final double MAX_TX_JUMP_DEG = 6.0;
  private static final int LOST_FRAMES_TO_UNLOCK = 1;

  private static final double ROTATION_KP = 1.8;
  private static final double MAX_ROTATION_RAD_PER_SEC = 45.0;

  private static final double FORWARD_SPEED_MPS = 3.5;
  private static final double MAX_RUNTIME_SEC = 6.0;

  // ---------------- Internal state ----------------

  private final SwerveSubsystem drive;
  private final NetworkTable limelight;
  private final Timer timer = new Timer();

  private boolean locked = false;
  private double lockedTxDeg = 0.0;
  private double smoothedTxDeg = 0.0;
  private int lostFrames = 0;

  // ---------------- Constructor ----------------

  public AutoAlignToFuelCommand(SwerveSubsystem drive) {
    this.drive = drive;
    this.limelight = NetworkTableInstance.getDefault().getTable("limelight-right");
    addRequirements(drive);
  }

  public static Command alignToFuelCommand(SwerveSubsystem drive) {
    return new AutoAlignToFuelCommand(drive);
  }

  // ---------------- Command lifecycle ----------------

  @Override
  public void initialize() {
    locked = false;
    lockedTxDeg = 0.0;
    smoothedTxDeg = 0.0;
    lostFrames = 0;
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double tv = limelight.getEntry("tv").getDouble(0.0);
    double tx = limelight.getEntry("tx").getDouble(0.0);

    boolean seesTarget = tv >= 1.0;

    Logger.recordOutput("AutoAlignFuel/SeesTarget", seesTarget);
    Logger.recordOutput("AutoAlignFuel/RawTxDeg", tx);
    Logger.recordOutput("AutoAlignFuel/Locked", locked);

    // -------- Acquire lock --------
    if (!locked) {
      if (seesTarget) {
        locked = true;
        lockedTxDeg = tx;
        smoothedTxDeg = tx;
        lostFrames = 0;
      } else {
        drive.runVelocity(new ChassisSpeeds());
        return;
      }
    }

    // -------- Maintain / lose lock --------
    if (!seesTarget) {
      lostFrames++;
      if (lostFrames >= LOST_FRAMES_TO_UNLOCK) {
        locked = false;
        drive.runVelocity(new ChassisSpeeds());
      }
      return;
    } else {
      lostFrames = 0;
    }

    // -------- Reject sudden jumps (prevents switching piles) --------
    if (Math.abs(tx - lockedTxDeg) <= MAX_TX_JUMP_DEG) {
      lockedTxDeg = tx;
      smoothedTxDeg += 0.25 * (tx - smoothedTxDeg);
    }

    Logger.recordOutput("AutoAlignFuel/SmoothedTxDeg", smoothedTxDeg);

    // -------- Continuous drive + rotate --------
    double omega =
        MathUtil.clamp(
            ROTATION_KP * Math.toRadians(smoothedTxDeg),
            -MAX_ROTATION_RAD_PER_SEC,
            MAX_ROTATION_RAD_PER_SEC);

    drive.runVelocity(new ChassisSpeeds(FORWARD_SPEED_MPS, 0.0, omega));
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    drive.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(MAX_RUNTIME_SEC);
  }
}
