package org.Griffins1884.frc2026.subsystems.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public Rotation2d turnPosition = new Rotation2d();
    public double turnPositionRotations = 0.0;
    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public double turnAbsolutePositionRotations = 0.0;
    public double turnZeroTrimRotations = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    public double[] odometryTurnPositionsRotations = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the drive motor at the specified velocity with a feedforward term. */
  public default void setDriveVelocity(double velocityRadPerSec, double feedforward) {
    setDriveVelocity(velocityRadPerSec);
  }

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}

  /** Set P, I, and D gains for closed loop control on drive motor. */
  public default void setDrivePID(double kP, double kI, double kD) {}

  /** Set P, I, and D gains for closed loop control on turn motor. */
  public default void setTurnPID(double kP, double kI, double kD) {}

  /** Set brake mode on drive motor. */
  public default void setBrakeMode(boolean enabled) {}

  /** Add Kraken instruments to an Orchestra list if supported. */
  public default void addOrchestraInstruments(List<TalonFX> instruments) {}

  /** Saves the current steering angle as the new zero trim. */
  public default void captureZeroTrim() {}

  /** Clears any saved steering zero trim. */
  public default void clearZeroTrim() {}
}
