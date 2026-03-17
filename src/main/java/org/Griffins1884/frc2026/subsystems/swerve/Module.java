package org.Griffins1884.frc2026.subsystems.swerve;

import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.List;
import lombok.Getter;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final double ANGLE_JUMP_THRESHOLD_RAD = Math.toRadians(35.0);
  private static final double ANGLE_JUMP_MAX_TURN_RATE_RAD_PER_SEC = 1.0;
  private static final double SPEED_RATIO_EPSILON_MPS = 0.15;
  private static final LoggedTunableNumber krakenDrivekS =
      new LoggedTunableNumber("Drive/Module/DrivekS");
  private static final LoggedTunableNumber krakenDrivekV =
      new LoggedTunableNumber("Drive/Module/DrivekV");
  private static final LoggedTunableNumber krakenDrivekT =
      new LoggedTunableNumber("Drive/Module/DrivekT");
  private static final LoggedTunableNumber krakenDrivekP =
      new LoggedTunableNumber("Drive/Module/DrivekP");
  private static final LoggedTunableNumber krakenDrivekD =
      new LoggedTunableNumber("Drive/Module/DrivekD");
  private static final LoggedTunableNumber krakenTurnkP =
      new LoggedTunableNumber("Drive/Module/TurnkP");
  private static final LoggedTunableNumber krakenTurnkD =
      new LoggedTunableNumber("Drive/Module/TurnkD");

  static {
    krakenDrivekS.initDefault(KRAKEN_DRIVE_TORQUE_GAINS.kS().get());
    krakenDrivekV.initDefault(KRAKEN_DRIVE_TORQUE_GAINS.kV().get());
    krakenDrivekT.initDefault(
        SwerveConstants.KRAKEN_DRIVE_GEAR_RATIO / DCMotor.getKrakenX60Foc(1).KtNMPerAmp);
    krakenDrivekP.initDefault(KRAKEN_DRIVE_TORQUE_GAINS.kP().get());
    krakenDrivekD.initDefault(KRAKEN_DRIVE_TORQUE_GAINS.kD().get());
    krakenTurnkP.initDefault(KRAKEN_TURN_TORQUE_GAINS.kP().get());
    krakenTurnkD.initDefault(KRAKEN_TURN_TORQUE_GAINS.kD().get());
  }

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private SimpleMotorFeedforward krakenFfModel =
      new SimpleMotorFeedforward(krakenDrivekS.get(), krakenDrivekV.get());

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private double desiredSpeedMetersPerSec = 0.0;
  private Rotation2d desiredAngle = new Rotation2d();
  private Rotation2d lastTurnPosition = new Rotation2d();
  private boolean angleJumpDetected = false;
  private int angleJumpCount = 0;
  private double lastAngleDeltaRad = 0.0;

  /** -- GETTER -- Returns the module positions received this cycle. */
  @Getter private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on module " + index + ".", AlertType.kError);
  }

  public void addOrchestraInstruments(List<TalonFX> instruments) {
    if (instruments == null) {
      return;
    }
    io.addOrchestraInstruments(instruments);
  }

  public void periodic() {
    if (krakenDrivekS.hasChanged(hashCode()) || krakenDrivekV.hasChanged(hashCode())) {
      krakenFfModel = new SimpleMotorFeedforward(krakenDrivekS.get(), krakenDrivekV.get());
    }
    if (krakenDrivekP.hasChanged(hashCode()) || krakenDrivekD.hasChanged(hashCode())) {
      io.setDrivePID(krakenDrivekP.get(), 0.0, krakenDrivekD.get());
    }
    if (krakenTurnkP.hasChanged(hashCode()) || krakenTurnkD.hasChanged(hashCode())) {
      io.setTurnPID(krakenTurnkP.get(), 0.0, krakenTurnkD.get());
    }

    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + index, inputs);

    double actualSpeedMetersPerSec = getVelocityMetersPerSec();
    double angleErrorRad = MathUtil.angleModulus(desiredAngle.minus(getAngle()).getRadians());
    double speedErrorMetersPerSec = desiredSpeedMetersPerSec - actualSpeedMetersPerSec;
    double speedRatio =
        Math.abs(desiredSpeedMetersPerSec) > SPEED_RATIO_EPSILON_MPS
            ? actualSpeedMetersPerSec / desiredSpeedMetersPerSec
            : 1.0;
    lastAngleDeltaRad = MathUtil.angleModulus(getAngle().minus(lastTurnPosition).getRadians());
    boolean suspiciousJump =
        Math.abs(lastAngleDeltaRad) > ANGLE_JUMP_THRESHOLD_RAD
            && Math.abs(inputs.turnVelocityRadPerSec) < ANGLE_JUMP_MAX_TURN_RATE_RAD_PER_SEC;
    if (suspiciousJump) {
      angleJumpCount++;
    }
    angleJumpDetected = suspiciousJump;
    lastTurnPosition = getAngle();

    Logger.recordOutput("Swerve/Module" + index + "/DesiredSpeedMps", desiredSpeedMetersPerSec);
    Logger.recordOutput("Swerve/Module" + index + "/ActualSpeedMps", actualSpeedMetersPerSec);
    Logger.recordOutput("Swerve/Module" + index + "/SpeedErrorMps", speedErrorMetersPerSec);
    Logger.recordOutput("Swerve/Module" + index + "/SpeedRatio", speedRatio);
    Logger.recordOutput("Swerve/Module" + index + "/DesiredAngleRad", desiredAngle.getRadians());
    Logger.recordOutput("Swerve/Module" + index + "/ActualAngleRad", getAngle().getRadians());
    Logger.recordOutput("Swerve/Module" + index + "/AngleErrorRad", angleErrorRad);
    Logger.recordOutput("Swerve/Module" + index + "/LastAngleDeltaRad", lastAngleDeltaRad);
    Logger.recordOutput("Swerve/Module" + index + "/AngleJumpDetected", angleJumpDetected);
    Logger.recordOutput("Swerve/Module" + index + "/AngleJumpCount", angleJumpCount);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    desiredSpeedMetersPerSec = state.speedMetersPerSecond;
    desiredAngle = state.angle;
    // Mechanical Advantage-style control for full Kraken modules
    double speedRadPerSec = state.speedMetersPerSecond / WHEEL_RADIUS;
    io.setDriveVelocity(speedRadPerSec, krakenFfModel.calculate(speedRadPerSec));
    if (Math.abs(state.angle.minus(getAngle()).getDegrees()) < TURN_DEADBAND_DEGREES) {
      io.setTurnOpenLoop(0.0);
    } else {
      io.setTurnPosition(state.angle);
    }
  }

  /** Runs the module with the specified output while controlling to zeroRotation degrees. */
  public void runCharacterization(double output) {
    desiredSpeedMetersPerSec = 0.0;
    desiredAngle = new Rotation2d();
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }

  /** Runs a steer-only SysId sweep while keeping the drive stage disabled. */
  public void runTurnCharacterization(double output) {
    desiredSpeedMetersPerSec = 0.0;
    desiredAngle = new Rotation2d();
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(output);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    desiredSpeedMetersPerSec = 0.0;
    desiredAngle = getAngle();
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * WHEEL_RADIUS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * WHEEL_RADIUS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  public double getVoltage() {
    return inputs.driveAppliedVolts;
  }

  public double getDriveVoltage() {
    return inputs.driveAppliedVolts;
  }

  public double getTurnVoltage() {
    return inputs.turnAppliedVolts;
  }

  public double getTurnPositionRad() {
    return inputs.turnPosition.getRadians();
  }

  public double getTurnVelocityRadPerSec() {
    return inputs.turnVelocityRadPerSec;
  }

  public int getIndex() {
    return index;
  }

  public double getDesiredSpeedMetersPerSec() {
    return desiredSpeedMetersPerSec;
  }

  public Rotation2d getDesiredAngle() {
    return desiredAngle;
  }

  public double getSpeedErrorMetersPerSec() {
    return desiredSpeedMetersPerSec - getVelocityMetersPerSec();
  }

  public double getSpeedRatio() {
    return Math.abs(desiredSpeedMetersPerSec) > SPEED_RATIO_EPSILON_MPS
        ? getVelocityMetersPerSec() / desiredSpeedMetersPerSec
        : 1.0;
  }

  public double getAngleErrorRad() {
    return MathUtil.angleModulus(desiredAngle.minus(getAngle()).getRadians());
  }

  public boolean isAngleJumpDetected() {
    return angleJumpDetected;
  }

  public int getAngleJumpCount() {
    return angleJumpCount;
  }
}
