package org.Griffins1884.frc2026.subsystems;

import static org.Griffins1884.frc2026.Config.Subsystems.*;
import static org.Griffins1884.frc2026.GlobalConstants.MODE;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.Griffins1884.frc2026.Config;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.commands.ShooterCommands;
import org.Griffins1884.frc2026.commands.TurretCommands;
import org.Griffins1884.frc2026.generic.arms.Arms;
import org.Griffins1884.frc2026.generic.elevators.Elevators;
import org.Griffins1884.frc2026.generic.rollers.Rollers;
import org.Griffins1884.frc2026.generic.turrets.GenericPositionTurretSystem.ControlMode;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerSubsystem.IndexerGoal;
import org.Griffins1884.frc2026.subsystems.intake.IntakePivotSubsystem.IntakePivotGoal;
import org.Griffins1884.frc2026.subsystems.intake.IntakeSubsystem.IntakeGoal;
import org.Griffins1884.frc2026.subsystems.leds.LEDIOPWM;
import org.Griffins1884.frc2026.subsystems.leds.LEDIOSim;
import org.Griffins1884.frc2026.subsystems.leds.LEDSubsystem;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterConstants;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotSubsystem.ShooterPivotGoal;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.Griffins1884.frc2026.util.AllianceFlipUtil;
import org.Griffins1884.frc2026.util.TurretUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Superstructure extends SubsystemBase {
  public enum SuperState {
    IDLING,
    INTAKING,
    SHOOTING,
    SHOOT_INTAKE,
    FERRYING,
    TESTING
  }

  public record SuperstructureOutcome(
      SuperState state,
      SuperState requestedState,
      IntakeGoal intakeGoal,
      IndexerGoal indexerGoal,
      double shooterTargetVelocityRpm,
      IntakePivotGoal intakePivotGoal,
      ShooterPivotGoal shooterPivotGoal,
      boolean shooterPivotManual,
      double shooterPivotPosition,
      String turretAction,
      Translation2d turretTarget) {}

  public record StateRequestResult(boolean accepted, String reason) {}

  private final SwerveSubsystem drive;
  @Setter private TurretSubsystem turret;

  @Getter
  private final LoggedDashboardChooser<SuperState> stateChooser =
      new LoggedDashboardChooser<>("Superstructure State");

  @Getter private SuperState requestedState = SuperState.IDLING;
  @Getter private SuperState currentState = SuperState.IDLING;
  private boolean stateOverrideActive = false;
  private boolean autoStateEnabled = true;
  private SuperState lastChooserSelection = null;
  private boolean manualControlActive = false;
  @Setter private boolean shooterPivotExternalControl = false;
  private DoubleSupplier manualTurretAxis = () -> 0.0;
  private DoubleSupplier manualPivotAxis = () -> 0.0;
  private Supplier<Optional<Pose2d>> autoStartPoseSupplier = Optional::empty;

  private final Debouncer ballPresentDebouncer =
      new Debouncer(SuperstructureConstants.BALL_PRESENCE_DEBOUNCE_SEC.get(), DebounceType.kBoth);
  @Setter private boolean turretExternalControl = false;
  @Setter @Getter private boolean shootEnabled = false;
  @Setter @Getter private boolean intakeRollersHeld = false;
  @Getter private boolean intakeDeployed = false;
  @Getter private boolean shootReadyLatched = false;
  private final LEDSubsystem leds =
      Config.Subsystems.LEDS_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new LEDSubsystem(new LEDIOPWM())
              : new LEDSubsystem(new LEDIOSim()))
          : null;

  private IntakeGoal lastIntakeGoal = IntakeGoal.IDLING;
  private IndexerGoal lastIndexerGoal = IndexerGoal.IDLING;
  private double lastShooterTargetVelocityRpm = 0.0;
  private IntakePivotGoal lastIntakePivotGoal = IntakePivotGoal.IDLING;
  private ShooterPivotGoal lastShooterPivotGoal = ShooterPivotGoal.IDLING;
  private boolean lastShooterPivotManual = false;
  private double lastShooterPivotPosition = 0.0;
  private String lastTurretAction = "HOLD";
  private Translation2d lastTurretTarget = null;
  private final Rollers rollers = new Rollers();
  @Getter private final Elevators elevators = new Elevators();
  @Getter private final Arms arms = new Arms();
  private static final double SYS_ID_IDLE_WAIT_SECONDS = 0.5;

  public Superstructure(SwerveSubsystem drive) {
    this.drive = drive;
    configureStateChooser();
    if (LEDS_ENABLED) {
      leds.setDefaultCommand(
          leds.ledCommand(
              DriverStation::isEnabled,
              () -> drive != null ? drive.getPose() : null,
              this::getAutoStartPose,
              this::getCurrentState,
              this::hasBall));
    }
  }

  public Command setSuperStateCmd(SuperState stateRequest) {
    return Commands.runOnce(() -> requestState(stateRequest));
  }

  public void setAutoStateEnabled(boolean enabled) {
    autoStateEnabled = enabled;
    if (enabled) {
      clearStateOverride();
    }
  }

  public void bindManualControlSuppliers(DoubleSupplier turretAxis, DoubleSupplier pivotAxis) {
    manualTurretAxis = turretAxis != null ? turretAxis : () -> 0.0;
    manualPivotAxis = pivotAxis != null ? pivotAxis : () -> 0.0;
  }

  public void clearStateOverride() {
    stateOverrideActive = false;
  }

  public void toggleManualControl() {
    manualControlActive = !manualControlActive;
  }

  public void toggleShootEnabled() {
    setShootEnabled(!shootEnabled);
  }

  public void toggleIntakeDeploy() {
    intakeDeployed = !intakeDeployed;
  }

  public void requestIntakeDeployRezero() {
    if (arms.intakePivot != null) {
      arms.intakePivot.requestZeroCalibration();
    }
  }

  public void cancelIntakeDeployRezero() {
    if (arms.intakePivot != null) {
      arms.intakePivot.cancelZeroCalibration();
    }
  }

  public boolean isIntakeDeployRezeroInProgress() {
    return arms.intakePivot != null && arms.intakePivot.isZeroCalibrationInProgress();
  }

  public StateRequestResult requestStateFromDashboard(SuperState state) {
    return requestState(state);
  }

  public StateRequestResult requestState(SuperState state) {
    if (state == null) {
      return new StateRequestResult(false, "Invalid state");
    }
    String rejectReason = getDashboardRejectReason(state);
    if (rejectReason != null) {
      return new StateRequestResult(false, rejectReason);
    }
    requestStateInternal(state, true);
    return new StateRequestResult(true, "");
  }

  public boolean hasBall() {
    return isBallPresent();
  }

  public void setAutoStartPoseSupplier(Supplier<Optional<Pose2d>> supplier) {
    autoStartPoseSupplier = supplier != null ? supplier : Optional::empty;
  }

  private Optional<Pose2d> getAutoStartPose() {
    return autoStartPoseSupplier.get();
  }

  @Override
  public void periodic() {
    SuperState previousState = currentState;
    if (DriverStation.isTeleopEnabled()) {
      if (stateOverrideActive) {
        if (requestedState != currentState) {
          enterState(requestedState);
          currentState = requestedState;
        }
        if (currentState != previousState) {
          shootReadyLatched = false;
        }
        applyState(currentState);
        Logger.recordOutput("Superstructure/AutoState", "TELEOP_OVERRIDE");
      } else {
        currentState = computeTeleopState();
        requestedState = currentState;
        if (currentState != previousState) {
          shootReadyLatched = false;
        }
        applyTeleopControl();
        Logger.recordOutput("Superstructure/AutoState", "TELEOP_DRIVER_CONTROL");
      }
    } else {
      updateRequestedStateFromChooser();
      if (autoStateEnabled && !stateOverrideActive) {
        SuperState autoState = computeAutoState();
        if (autoState != null) {
          requestStateInternal(autoState, false);
        }
        Logger.recordOutput(
            "Superstructure/AutoState", autoState != null ? autoState.toString() : "UNKNOWN");
      }
      if (requestedState != currentState) {
        enterState(requestedState);
        currentState = requestedState;
      }
      if (currentState != previousState) {
        shootReadyLatched = false;
      }
      applyState(currentState);
    }

    if (manualControlActive) {
      applyManualJog();
    } else {
      stopManualJog();
    }
    Logger.recordOutput("Superstructure/State", currentState.toString());
    Logger.recordOutput("Superstructure/RequestedState", requestedState.toString());
    Logger.recordOutput("Superstructure/turretTarget", lastTurretTarget);
    Logger.recordOutput("Superstructure/AutoStateEnabled", autoStateEnabled);
    Logger.recordOutput("Superstructure/ShootEnabled", shootEnabled);
    Logger.recordOutput("Superstructure/ShootReadyLatched", shootReadyLatched);
    Logger.recordOutput("Superstructure/IntakeRollersHeld", intakeRollersHeld);
    Logger.recordOutput("Superstructure/IntakeDeployed", intakeDeployed);
    Logger.recordOutput("Superstructure/InAllianceZone", isInAllianceZone());
  }

  public void registerSuperstructureCharacterization(
      Supplier<LoggedDashboardChooser<Command>> autoChooser) {
    LoggedDashboardChooser<Command> chooser = autoChooser.get();
    if (chooser == null) {
      return;
    }
    if (INTAKE_PIVOT_ENABLED) {
      addSysIdOptions(
          chooser,
          "Intake Pivot",
          arms.intakePivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
          arms.intakePivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
          arms.intakePivot.sysIdDynamic(SysIdRoutine.Direction.kForward),
          arms.intakePivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
    if (SHOOTER_PIVOT_ENABLED) {
      addSysIdOptions(
          chooser,
          "Shooter Pivot",
          arms.shooterPivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
          arms.shooterPivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
          arms.shooterPivot.sysIdDynamic(SysIdRoutine.Direction.kForward),
          arms.shooterPivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
    if (INTAKE_ENABLED) {
      addSysIdOptions(
          chooser,
          "Intake",
          rollers.intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
          rollers.intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
          rollers.intake.sysIdDynamic(SysIdRoutine.Direction.kForward),
          rollers.intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
    if (INDEXER_ENABLED) {
      addSysIdOptions(
          chooser,
          "Indexer",
          rollers.indexer.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
          rollers.indexer.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
          rollers.indexer.sysIdDynamic(SysIdRoutine.Direction.kForward),
          rollers.indexer.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
    if (SHOOTER_ENABLED) {
      addSysIdOptions(
          chooser,
          "Shooter",
          rollers.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
          rollers.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
          rollers.shooter.sysIdDynamic(SysIdRoutine.Direction.kForward),
          rollers.shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  private void configureStateChooser() {
    stateChooser.addDefaultOption("Idling", SuperState.IDLING);
    stateChooser.addOption("Intaking", SuperState.INTAKING);
    stateChooser.addOption("Shooting", SuperState.SHOOTING);
    stateChooser.addOption("Shoot+Intake", SuperState.SHOOT_INTAKE);
    stateChooser.addOption("Ferrying", SuperState.FERRYING);
    stateChooser.addOption("Testing", SuperState.TESTING);
  }

  private void updateRequestedStateFromChooser() {
    SuperState selected = stateChooser.get();
    if (selected == null) {
      return;
    }
    if (lastChooserSelection == null) {
      lastChooserSelection = selected;
      return;
    }
    if (selected != lastChooserSelection) {
      lastChooserSelection = selected;
      requestStateInternal(selected, true);
    }
  }

  private SuperState computeAutoState() {
    if (drive == null) {
      return null;
    }
    Pose2d pose = drive.getPose();
    if (pose == null || !Double.isFinite(pose.getX())) {
      return null;
    }
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return null;
    }
    double xBlue =
        alliance.get() == DriverStation.Alliance.Red
            ? GlobalConstants.FieldConstants.fieldLength - pose.getX()
            : pose.getX();
    if (GlobalConstants.isDebugMode()) {
      Logger.recordOutput("Superstructure/AutoXBlue", xBlue);
    }

    if (xBlue < SuperstructureConstants.AUTO_STATE_SHOOTING_X_MAX_METERS) {
      return SuperState.SHOOTING;
    }
    if (xBlue < SuperstructureConstants.AUTO_STATE_IDLE_X_MAX_METERS) {
      return SuperState.IDLING;
    }
    if (xBlue < SuperstructureConstants.AUTO_STATE_INTAKE_X_MAX_METERS) {
      return SuperState.INTAKING;
    }
    return SuperState.IDLING;
  }

  private SuperState computeTeleopState() {
    if (shootEnabled && intakeRollersHeld) {
      return SuperState.SHOOT_INTAKE;
    }
    if (shootEnabled) {
      return SuperState.SHOOTING;
    }
    if (intakeRollersHeld || intakeDeployed) {
      return SuperState.INTAKING;
    }
    if (!isInAllianceZone()) {
      return SuperState.FERRYING;
    }
    return SuperState.IDLING;
  }

  private void applyTeleopControl() {
    boolean inAllianceZone = isInAllianceZone();
    Translation2d target = inAllianceZone ? getHubTarget() : getFerryingTarget();

    boolean shooterShouldSpin = inAllianceZone || shootEnabled;

    setIntakePivotGoal(intakeDeployed ? IntakePivotGoal.PICKUP : IntakePivotGoal.IDLING);
    setIntakeGoal(intakeRollersHeld ? IntakeGoal.FORWARD : IntakeGoal.IDLING);
    applyAimingAndShooterSolution(target, shooterShouldSpin);

    boolean indexerActive = shouldEnableIndexer(shootEnabled, shooterShouldSpin);
    setIndexerGoal(indexerActive ? IndexerGoal.FORWARD : IndexerGoal.IDLING);
  }

  private boolean shouldEnableIndexer(boolean indexerRequested, boolean shooterShouldSpin) {
    if (!indexerRequested) {
      return false;
    }
    if (shootReadyLatched) {
      return true;
    }
    if (!shooterShouldSpin) {
      return false;
    }
    if (rollers.shooter == null || rollers.shooter.isAtGoal()) {
      shootReadyLatched = true;
      return true;
    }
    return false;
  }

  private boolean isIndexerRequested() {
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    if (DriverStation.isTeleopEnabled()) {
      return shootEnabled;
    }
    if (DriverStation.isTestEnabled()) {
      return true;
    }
    return false;
  }

  private boolean isTurretExternallyControlled() {
    return turretExternalControl || manualControlActive;
  }

  private boolean isShooterPivotExternallyControlled() {
    return shooterPivotExternalControl || manualControlActive;
  }

  private void applyManualJog() {
    double turretAxis = manualTurretAxis.getAsDouble();
    double pivotAxis = manualPivotAxis.getAsDouble();
    if (turret != null) {
      double percent = (SuperstructureConstants.MANUAL_JOG_VOLTAGE) * turretAxis;
      turret.setOpenLoop(percent);
    }
    if (arms.shooterPivot != null) {
      double percent = (SuperstructureConstants.MANUAL_JOG_VOLTAGE) * pivotAxis;
      arms.shooterPivot.setOpenLoop(percent);
    }
    if (GlobalConstants.isDebugMode()) {
      Logger.recordOutput("Superstructure/ManualPivotAxis", pivotAxis);
    }
  }

  private void stopManualJog() {
    if (turret != null) {
      turret.stopOpenLoop();
    }
    if (arms.shooterPivot != null) {
      arms.shooterPivot.stopOpenLoop();
    }
  }

  private void requestStateInternal(SuperState state, boolean override) {
    requestedState = state;
    if (override) {
      stateOverrideActive = true;
    }
  }

  private String getDashboardRejectReason(SuperState state) {
    // Hook for future interlocks; return null to accept the request.
    return null;
  }

  private void enterState(SuperState state) {}

  private void applyState(SuperState state) {
    boolean indexerRequested = isIndexerRequested();
    switch (state) {
      case IDLING -> applyIdle();
      case INTAKING -> applyIntaking();
      case SHOOTING -> applyShooting(getHubTarget(), indexerRequested);
      case SHOOT_INTAKE -> applyShootingAndIntaking(getHubTarget(), indexerRequested);
      case FERRYING -> applyFerrying(indexerRequested);
      case TESTING -> applyTesting();
    }
  }

  private void applyIdle() {
    clearIdleOverrides();
    setIntakeGoal(IntakeGoal.IDLING);
    setIndexerGoal(IndexerGoal.IDLING);
    setShooterTargetVelocity(0.0);
    setIntakePivotGoal(IntakePivotGoal.IDLING);
    setShooterPivotGoal(ShooterPivotGoal.IDLING, false, 0.0);
    holdTurret();
  }

  private void clearIdleOverrides() {
    if (rollers.intake != null) {
      rollers.intake.clearGoalOverride();
    }
    if (rollers.indexer != null) {
      rollers.indexer.clearGoalOverride();
    }
    if (arms.shooterPivot != null) {
      if (!isShooterPivotExternallyControlled()) {
        arms.shooterPivot.stopOpenLoop();
        arms.shooterPivot.clearGoalOverride();
      }
    }
  }

  private void applyIntaking() {
    setIntakeGoal(IntakeGoal.FORWARD);
    setIndexerGoal(IndexerGoal.IDLING);
    setShooterTargetVelocity(0.0);
    setIntakePivotGoal(IntakePivotGoal.PICKUP);
    setShooterPivotGoal(ShooterPivotGoal.IDLING, false, 0.0);
    holdTurret();
  }

  private void applyShooting(Translation2d target, boolean indexerRequested) {
    if (SuperstructureConstants.SHOOTING_WHILE_MOVING) {
      final Translation2d oldTarget = target;
      target =
          TurretCommands.shootingWhileMoving(
              drive::getPose,
              () -> oldTarget,
              drive::getFieldVelocity,
              drive::getFieldAcceleration);
    }
    setIntakeGoal(IntakeGoal.IDLING);
    setIntakePivotGoal(IntakePivotGoal.IDLING);
    applyAimingAndShooterSolution(target, true);
    boolean indexerActive = shouldEnableIndexer(indexerRequested, true);
    setIndexerGoal(indexerActive ? IndexerGoal.FORWARD : IndexerGoal.IDLING);
  }

  private void applyShootingAndIntaking(Translation2d target, boolean indexerRequested) {
    if (SuperstructureConstants.SHOOTING_WHILE_MOVING) {
      final Translation2d oldTarget = target;
      target =
          TurretCommands.shootingWhileMoving(
              drive::getPose,
              () -> oldTarget,
              drive::getFieldVelocity,
              drive::getFieldAcceleration);
    }
    setIntakeGoal(IntakeGoal.FORWARD);
    setIntakePivotGoal(IntakePivotGoal.PICKUP);
    applyAimingAndShooterSolution(target, true);
    boolean indexerActive = shouldEnableIndexer(indexerRequested, true);
    setIndexerGoal(indexerActive ? IndexerGoal.FORWARD : IndexerGoal.IDLING);
  }

  private void applyFerrying(boolean indexerRequested) {
    Translation2d target = getFerryingTarget();
    setIntakeGoal(IntakeGoal.FORWARD);
    setIntakePivotGoal(IntakePivotGoal.PICKUP);
    applyAimingAndShooterSolution(target, true);
    boolean indexerActive = shouldEnableIndexer(indexerRequested, true);
    setIndexerGoal(indexerActive ? IndexerGoal.FORWARD : IndexerGoal.IDLING);
  }

  private void applyTesting() {
    setIntakeGoal(IntakeGoal.TESTING);
    setIndexerGoal(IndexerGoal.TESTING);
    setShooterTargetVelocity(ShooterConstants.TARGET_RPM);
    setIntakePivotGoal(IntakePivotGoal.TESTING);
    setShooterPivotGoal(ShooterPivotGoal.TESTING, false, 0.0);
    Translation2d target = getHubTarget();
    aimTurretAt(target);
    ShooterCommands.calc(drive.getPose(), target, currentState);
    // if (turret != null) {
    //   turret.setGoalRad(TurretConstants.TEST_GOAL_RAD.get());
    //   lastTurretAction = "TEST_GOAL";
    // } else {
    //   holdTurret();
    // }
  }

  private void setIntakeGoal(IntakeGoal goal) {
    lastIntakeGoal = goal;
    if (rollers.intake != null) {
      rollers.intake.setGoal(goal);
    }
  }

  private void setIndexerGoal(IndexerGoal goal) {
    lastIndexerGoal = goal;
    if (rollers.indexer != null) {
      rollers.indexer.setGoal(goal);
    }
  }

  private void setShooterTargetVelocity(double targetRpm) {
    lastShooterTargetVelocityRpm = targetRpm;
    if (rollers.shooter != null) {
      rollers.shooter.setTargetVelocityRpm(targetRpm);
    }
  }

  private void setIntakePivotGoal(IntakePivotGoal goal) {
    lastIntakePivotGoal = goal;
    if (arms.intakePivot != null) {
      arms.intakePivot.setGoal(goal);
    }
  }

  private void setShooterPivotGoal(ShooterPivotGoal goal, boolean manual, double manualPosition) {
    if (isShooterPivotExternallyControlled()) {
      return;
    }
    lastShooterPivotGoal = goal;
    lastShooterPivotManual = manual;
    lastShooterPivotPosition = manualPosition;
    if (arms.shooterPivot == null) {
      return;
    }
    if (manual) {
      arms.shooterPivot.setGoalPosition(manualPosition);
    } else {
      arms.shooterPivot.clearGoalOverride();
    }
    arms.shooterPivot.setGoal(goal);
  }

  private void holdTurret() {
    lastTurretTarget = null;
    if (isTurretExternallyControlled()) {
      lastTurretAction = "EXTERNAL";
      return;
    }
    if (turret != null) {
      if (turret.getControlMode() == ControlMode.OPEN_LOOP) {
        lastTurretAction = "OPEN_LOOP";
        return;
      }
      turret.setGoalRad(turret.getPositionRad());
      lastTurretAction = "HOLD";
      return;
    }
    lastTurretAction = "HOLD";
  }

  private void aimTurretAt(Translation2d target) {
    if (isTurretExternallyControlled()) {
      lastTurretAction = "EXTERNAL";
      lastTurretTarget = null;
      return;
    }
    if (turret == null || drive == null || target == null) {
      holdTurret();
      return;
    }
    turret.setGoalRad(TurretUtil.turretAngleToTarget(drive.getPose(), target));
    lastTurretAction = "AIM_TARGET";
    lastTurretTarget = target;
  }

  private void applyAimingAndShooterSolution(
      Translation2d target, boolean shooterEnabledForTarget) {
    aimTurretAt(target);

    if (rollers.shooter == null) {
      return;
    }

    if (!shooterEnabledForTarget) {
      setShooterTargetVelocity(0.0);
      setShooterPivotGoal(ShooterPivotGoal.IDLING, false, 0.0);
      return;
    }

    if (arms.shooterPivot == null || drive == null || target == null) {
      setShooterTargetVelocity(ShooterConstants.TARGET_RPM);
      return;
    }
    if (isShooterPivotExternallyControlled()) {
      setShooterTargetVelocity(ShooterConstants.TARGET_RPM);
      return;
    }

    Pose2d pose = drive.getPose();
    if (pose == null) {
      setShooterTargetVelocity(ShooterConstants.TARGET_RPM);
      return;
    }

    Map<ShooterCommands.Vals, Double> data = ShooterCommands.calc(pose, target, currentState);
    double targetAngle = data.getOrDefault(ShooterCommands.Vals.ANGLE, 0.0);
    double targetRpm = data.getOrDefault(ShooterCommands.Vals.RPM, 0.0);

    lastShooterPivotGoal = ShooterPivotGoal.IDLING;
    lastShooterPivotManual = true;
    lastShooterPivotPosition = targetAngle;

    arms.shooterPivot.setGoal(ShooterPivotGoal.IDLING);
    arms.shooterPivot.setGoalPosition(targetAngle);

    setShooterTargetVelocity(targetRpm);
  }

  public boolean isInAllianceZone() {
    if (drive == null) {
      return false;
    }
    Pose2d pose = drive.getPose();
    if (pose == null || !Double.isFinite(pose.getX())) {
      return false;
    }
    Optional<DriverStation.Alliance> alliance = AllianceFlipUtil.resolveAlliance(pose);
    double xBlue;
    if (alliance.isPresent()) {
      xBlue =
          alliance.get() == DriverStation.Alliance.Red
              ? GlobalConstants.FieldConstants.fieldLength - pose.getX()
              : pose.getX();
    } else {
      xBlue = Math.min(pose.getX(), GlobalConstants.FieldConstants.fieldLength - pose.getX());
    }
    return xBlue <= SuperstructureConstants.ALLIANCE_ZONE_MAX_X_METERS.get();
  }

  public Translation2d getHubTarget() {
    Pose2d pose = drive != null ? drive.getPose() : null;
    boolean isBlue =
        AllianceFlipUtil.resolveAlliance(pose).orElseGet(() -> inferAllianceFromPose(pose))
            == DriverStation.Alliance.Blue;
    return isBlue
        ? GlobalConstants.FieldConstants.Hub.topCenterPoint.toTranslation2d()
        : GlobalConstants.FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
  }

  public Translation2d getCurrentTurretTarget() {
    return isInAllianceZone() ? getHubTarget() : getFerryingTarget();
  }

  public Translation2d getFerryingTarget() {
    Pose2d pose = drive != null ? drive.getPose() : null;
    boolean isBlue =
        AllianceFlipUtil.resolveAlliance(pose).orElseGet(() -> inferAllianceFromPose(pose))
            == DriverStation.Alliance.Blue;
    boolean yChange = false;
    if (drive != null) {
      yChange = drive.getPose().getY() > 4.0;
    }
    Translation2d target;
    if (isBlue) {
      target = new Translation2d(3, yChange ? 7.0 : 1.0);
    } else {
      target = new Translation2d(13.5, yChange ? 7.0 : 1.0);
    }
    return target;
  }

  private DriverStation.Alliance inferAllianceFromPose(Pose2d pose) {
    if (pose == null || !Double.isFinite(pose.getX())) {
      if (lastTurretTarget != null) {
        return lastTurretTarget.getX() <= GlobalConstants.FieldConstants.fieldLength * 0.5
            ? DriverStation.Alliance.Blue
            : DriverStation.Alliance.Red;
      }
      return DriverStation.Alliance.Red;
    }
    return pose.getX() <= GlobalConstants.FieldConstants.fieldLength * 0.5
        ? DriverStation.Alliance.Blue
        : DriverStation.Alliance.Red;
  }

  private boolean isBallSenseAvailable() {
    return rollers.indexer != null || rollers.shooter != null || rollers.intake != null;
  }

  private boolean isBallPresent() {
    double currentAmps = getBallSenseCurrentAmps();
    boolean present =
        ballPresentDebouncer.calculate(
            currentAmps >= SuperstructureConstants.BALL_PRESENT_CURRENT_AMPS.get());
    Logger.recordOutput("Superstructure/BallCurrentAmps", currentAmps);
    Logger.recordOutput("Superstructure/BallPresent", present);
    return present;
  }

  private double getBallSenseCurrentAmps() {
    double ind = 0, shoot = 0, intake = 0;
    if (rollers.indexer != null) {
      ind = rollers.indexer.getSupplyCurrentAmps();
    }
    if (rollers.shooter != null) {
      shoot = rollers.shooter.getSupplyCurrentAmps();
    }
    if (rollers.intake != null) {
      intake = rollers.intake.getSupplyCurrentAmps();
    }
    return Math.max(ind, Math.max(shoot, intake));
  }

  private void addSysIdOptions(
      LoggedDashboardChooser<Command> chooser,
      String name,
      Command quasistaticForward,
      Command quasistaticReverse,
      Command dynamicForward,
      Command dynamicReverse) {
    Command qForwardFull = quasistaticForward.asProxy();
    Command qReverseFull = quasistaticReverse.asProxy();
    Command dForwardFull = dynamicForward.asProxy();
    Command dReverseFull = dynamicReverse.asProxy();
    chooser.addOption(
        name + " | SysId (Full Routine)",
        sysIdRoutine(name, qForwardFull, qReverseFull, dForwardFull, dReverseFull));
    chooser.addOption(
        name + " | SysId (Quasistatic Forward)",
        sysIdSingle(name, "QuasistaticForward", quasistaticForward.asProxy()));
    chooser.addOption(
        name + " | SysId (Quasistatic Reverse)",
        sysIdSingle(name, "QuasistaticReverse", quasistaticReverse.asProxy()));
    chooser.addOption(
        name + " | SysId (Dynamic Forward)",
        sysIdSingle(name, "DynamicForward", dynamicForward.asProxy()));
    chooser.addOption(
        name + " | SysId (Dynamic Reverse)",
        sysIdSingle(name, "DynamicReverse", dynamicReverse.asProxy()));
  }

  private void logSysIdStatus(boolean active, String name, String phase) {
    if (GlobalConstants.isDebugMode()) {
      Logger.recordOutput("Superstructure/SysId/Active", active);
      Logger.recordOutput("Superstructure/SysId/Name", name);
      Logger.recordOutput("Superstructure/SysId/Phase", phase);
    }
  }

  private Command sysIdPhase(String name, String phase) {
    return Commands.runOnce(() -> logSysIdStatus(true, name, phase));
  }

  private Command sysIdSingle(String name, String phase, Command command) {
    return command
        .beforeStarting(() -> logSysIdStatus(true, name, phase))
        .finallyDo(interrupted -> logSysIdStatus(false, "NONE", "NONE"))
        .withName(name + "SysId-" + phase);
  }

  private Command sysIdRoutine(
      String name,
      Command quasistaticForward,
      Command quasistaticReverse,
      Command dynamicForward,
      Command dynamicReverse) {
    return Commands.sequence(
            sysIdPhase(name, "QuasistaticForward"),
            quasistaticForward,
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            sysIdPhase(name, "QuasistaticReverse"),
            quasistaticReverse,
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            sysIdPhase(name, "DynamicForward"),
            dynamicForward,
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            sysIdPhase(name, "DynamicReverse"),
            dynamicReverse)
        .finallyDo(interrupted -> logSysIdStatus(false, "NONE", "NONE"))
        .withName(name + "SysIdRoutine");
  }

  public void close() {
    if (leds != null) {
      leds.close();
    }
  }

  public SuperstructureOutcome getOutcomeSnapshot() {
    return new SuperstructureOutcome(
        currentState,
        requestedState,
        lastIntakeGoal,
        lastIndexerGoal,
        lastShooterTargetVelocityRpm,
        lastIntakePivotGoal,
        lastShooterPivotGoal,
        lastShooterPivotManual,
        lastShooterPivotPosition,
        lastTurretAction,
        lastTurretTarget);
  }
}
