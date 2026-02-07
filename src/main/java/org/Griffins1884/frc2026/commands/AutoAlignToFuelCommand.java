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
 * <p>Behavior:
 * <ol>
 *   <li>Locks onto a fuel cluster (prevents switching targets)</li>
 *   <li>Rotates until the locked cluster is centered (tx ≈ 0)</li>
 *   <li>Drives forward while maintaining alignment</li>
 * </ol>
 */
public class AutoAlignToFuelCommand extends Command {

    // ---------------- Tuning constants ----------------

    private static final double MIN_AREA_TO_LOCK = 1.5;
    private static final double MIN_AREA_TO_KEEP = 0.8;

    private static final double MAX_TX_JUMP_DEG = 6.0;
    private static final int LOST_FRAMES_TO_UNLOCK = 8;

    private static final double TX_DEADBAND_DEG = 1.5;

    private static final double ROTATION_KP = 0.02;
    private static final double MAX_ROTATION_RAD_PER_SEC = 3.0;

    private static final double FORWARD_SPEED_MPS = 1.2;

    private static final double MAX_RUNTIME_SEC = 3.0;

    // ---------------- Internal state ----------------

    private final SwerveSubsystem drive;
    private final NetworkTable limelight;

    private boolean locked = false;
    private double lockedTxDeg = 0.0;
    private double smoothedTxDeg = 0.0;
    private int lostFrames = 0;

    private enum Phase {
        ROTATE,
        DRIVE
    }

    private Phase phase = Phase.ROTATE;
    private double startTimeSec = 0.0;

    // ---------------- Constructor ----------------

    public AutoAlignToFuelCommand(SwerveSubsystem drive) {
        this.drive = drive;
        this.limelight =
                NetworkTableInstance.getDefault().getTable("notecam"); // game-piece Limelight
        addRequirements(drive);
    }

    // ---------------- Command lifecycle ----------------

    @Override
    public void initialize() {
        locked = false;
        lockedTxDeg = 0.0;
        smoothedTxDeg = 0.0;
        lostFrames = 0;
        phase = Phase.ROTATE;
        startTimeSec = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double tv = limelight.getEntry("tv").getDouble(0.0);
        double tx = limelight.getEntry("tx").getDouble(0.0);
        double ta = limelight.getEntry("ta").getDouble(0.0);

        boolean seesTarget = tv >= 1.0;

        Logger.recordOutput("AutoAlignFuel/SeesTarget", seesTarget);
        Logger.recordOutput("AutoAlignFuel/RawTxDeg", tx);
        Logger.recordOutput("AutoAlignFuel/Area", ta);
        Logger.recordOutput("AutoAlignFuel/Locked", locked);
        Logger.recordOutput("AutoAlignFuel/Phase", phase.toString());

        // -------- Acquire lock --------
        if (!locked) {
            if (seesTarget && ta >= MIN_AREA_TO_LOCK) {
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
        if (!seesTarget || ta < MIN_AREA_TO_KEEP) {
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

        // -------- Control --------
        double omega =
                MathUtil.clamp(
                        ROTATION_KP * Math.toRadians(smoothedTxDeg),
                        -MAX_ROTATION_RAD_PER_SEC,
                        MAX_ROTATION_RAD_PER_SEC);

        if (phase == Phase.ROTATE) {
            drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));

            if (Math.abs(smoothedTxDeg) <= TX_DEADBAND_DEG) {
                phase = Phase.DRIVE;
            }
        } else {
            drive.runVelocity(new ChassisSpeeds(FORWARD_SPEED_MPS, 0.0, omega));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTimeSec) > MAX_RUNTIME_SEC;
    }
}

