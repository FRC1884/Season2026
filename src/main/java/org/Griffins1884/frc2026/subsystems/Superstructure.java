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
import org.Griffins1884.frc2026.commands.AlignConstants;
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
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotSubsystem.ShooterPivotGoal;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterSubsystem.ShooterGoal;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
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
      ShooterGoal shooterGoal,
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
  private final LEDSubsystem leds =
      Config.Subsystems.LEDS_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new LEDSubsystem(new LEDIOPWM())
              : new LEDSubsystem(new LEDIOSim()))
          : null;

  private IntakeGoal lastIntakeGoal = IntakeGoal.IDLING;
  private IndexerGoal lastIndexerGoal = IndexerGoal.IDLING;
  private ShooterGoal lastShooterGoal = ShooterGoal.IDLING;
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
    return Commands.runOnce(() -> requestState(stateRequest, true));
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

  public StateRequestResult requestStateFromDashboard(SuperState state) {
    if (state == null) {
      return new StateRequestResult(false, "Invalid state");
    }
    String rejectReason = getDashboardRejectReason(state);
    if (rejectReason != null) {
      return new StateRequestResult(false, rejectReason);
    }
    requestState(state, true);
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
    updateRequestedStateFromChooser();
    if (autoStateEnabled && !stateOverrideActive) {
      SuperState autoState = computeAutoState();
      if (autoState != null) {
        requestState(autoState, false);
      }
      Logger.recordOutput(
          "Superstructure/AutoState", autoState != null ? autoState.toString() : "UNKNOWN");
    }
    if (requestedState != currentState) {
      enterState(requestedState);
      currentState = requestedState;
    }

    applyState(currentState);
    if (manualControlActive) {
      applyManualJog();
    } else {
      stopManualJog();
    }
    Logger.recordOutput("Superstructure/State", currentState.toString());
    Logger.recordOutput("Superstructure/RequestedState", requestedState.toString());
    Logger.recordOutput("Superstructure/turretTarget", lastTurretTarget);
    Logger.recordOutput("Superstructure/AutoStateEnabled", autoStateEnabled);
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
      requestState(selected, true);
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

  private void requestState(SuperState state, boolean override) {
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
    switch (state) {
      case IDLING -> applyIdle();
      case INTAKING -> applyIntaking();
      case SHOOTING ->
          applyShooting(
              (DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                  ? GlobalConstants.FieldConstants.Hub.topCenterPoint.toTranslation2d()
                  : GlobalConstants.FieldConstants.Hub.oppTopCenterPoint.toTranslation2d(),
              false);
      case SHOOT_INTAKE ->
          applyShootingAndIntaking(
              (DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                  ? GlobalConstants.FieldConstants.Hub.topCenterPoint.toTranslation2d()
                  : GlobalConstants.FieldConstants.Hub.oppTopCenterPoint.toTranslation2d());
      case FERRYING -> applyFerrying();
      case TESTING -> applyTesting();
    }
  }

  private void applyIdle() {
    clearIdleOverrides();
    setIntakeGoal(IntakeGoal.IDLING);
    setIndexerGoal(IndexerGoal.IDLING);
    setShooterGoal(ShooterGoal.IDLING);
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
    if (rollers.shooter != null) {
      rollers.shooter.clearGoalOverride();
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
    setShooterGoal(ShooterGoal.IDLING);
    setIntakePivotGoal(IntakePivotGoal.PICKUP);
    setShooterPivotGoal(ShooterPivotGoal.IDLING, false, 0.0);
    holdTurret();
  }

  private void applyShooting(Translation2d target, boolean autoStopOnEmpty) {
    boolean movingShotEnabled = SuperstructureConstants.SHOOTING_WHILE_MOVING;
    boolean driveAvailable = drive != null;
    boolean motionSampleValid =
        driveAvailable
            && drive.isFieldMotionSampleValid()
            && drive.getFieldMotionSampleAgeSec()
                <= AlignConstants.TURRET_MAX_MOTION_SAMPLE_AGE_SECONDS.get();
    Logger.recordOutput("Superstructure/ShootingWhileMoving/Enabled", movingShotEnabled);
    Logger.recordOutput("Superstructure/ShootingWhileMoving/DriveAvailable", driveAvailable);
    Logger.recordOutput("Superstructure/ShootingWhileMoving/MotionSampleValid", motionSampleValid);

    if (rollers.shooter != null
        && turret != null
        && turret.isAtGoal()
        && rollers.shooter.isAtGoal()) {
      if (movingShotEnabled && motionSampleValid) {
        final Translation2d oldTarget = target;
        target =
            TurretCommands.shootingWhileMoving(
                drive::getPose,
                () -> oldTarget,
                drive::getFieldVelocity,
                drive::getFieldAcceleration);
        Logger.recordOutput(
            "Superstructure/ShootingWhileMoving/LeadOffsetMeters",
            oldTarget.minus(target).getNorm());
      } else {
        Logger.recordOutput("Superstructure/ShootingWhileMoving/LeadOffsetMeters", 0.0);
      }
      setIntakeGoal(IntakeGoal.IDLING);
      setIndexerGoal(IndexerGoal.FORWARD);
      setShooterGoal(ShooterGoal.FORWARD);
      setIntakePivotGoal(IntakePivotGoal.IDLING);
      aimTurretAt(target);
      aimShooterPivotAt(target);

      if (autoStopOnEmpty && SuperstructureConstants.AUTO_STOP_ON_EMPTY && isBallSenseAvailable()) {
        if (!isBallPresent()) {
          requestState(SuperState.IDLING, false);
        }
      }
    } else {
      setIntakeGoal(IntakeGoal.IDLING);
      setIndexerGoal(IndexerGoal.IDLING);
      setShooterGoal(ShooterGoal.FORWARD);
      setIntakePivotGoal(IntakePivotGoal.IDLING);
      aimTurretAt(target);
      aimShooterPivotAt(target);
    }
  }

  private void applyShootingAndIntaking(Translation2d target) {
    if (rollers.shooter != null && rollers.shooter.isAtGoal()) {
      setIntakeGoal(IntakeGoal.FORWARD);
      setIndexerGoal(IndexerGoal.FORWARD);
      setShooterGoal(ShooterGoal.FORWARD);
      setIntakePivotGoal(IntakePivotGoal.PICKUP);
      aimTurretAt(target);
      aimShooterPivotAt(target);
    } else {
      setIntakeGoal(IntakeGoal.FORWARD);
      setIndexerGoal(IndexerGoal.IDLING);
      setShooterGoal(ShooterGoal.FORWARD);
      setIntakePivotGoal(IntakePivotGoal.PICKUP);
      aimTurretAt(target);
      aimShooterPivotAt(target);
    }
  }

  private void applyFerrying() {
    Translation2d target;
    if (drive != null) target = new Translation2d(0, 0); // TODO: find a way to get the ferry target
    else target = new Translation2d(0.0, 0.0);
    setIntakeGoal(IntakeGoal.FORWARD);
    setIndexerGoal(IndexerGoal.FORWARD);
    setShooterGoal(ShooterGoal.FORWARD);
    setIntakePivotGoal(IntakePivotGoal.PICKUP);
    aimTurretAt(target);
    aimShooterPivotAt(target);
  }

  private void applyTesting() {
    setIntakeGoal(IntakeGoal.TESTING);
    setIndexerGoal(IndexerGoal.TESTING);
    setShooterGoal(ShooterGoal.TESTING);
    setIntakePivotGoal(IntakePivotGoal.TESTING);
    setShooterPivotGoal(ShooterPivotGoal.TESTING, false, 0.0);
    Translation2d target =
        (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            ? GlobalConstants.FieldConstants.Hub.topCenterPoint.toTranslation2d()
            : GlobalConstants.FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
    aimTurretAt(target);
    ShooterCommands.calc(drive.getPose(), target);
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

  private void setShooterGoal(ShooterGoal goal) {
    lastShooterGoal = goal;
    if (rollers.shooter != null) {
      rollers.shooter.setGoal(goal);
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

  private void aimShooterPivotAt(Translation2d target) {
    if (isShooterPivotExternallyControlled()) {
      return;
    }
    if (arms.shooterPivot == null || drive == null || target == null) {
      return;
    }
    Pose2d pose = drive.getPose();
    if (pose == null) {
      return;
    }

    Map<ShooterCommands.Vals, Double> data = ShooterCommands.calc(pose, target);

    lastShooterPivotGoal = ShooterPivotGoal.IDLING;
    lastShooterPivotManual = true;
    lastShooterPivotPosition = data.get(ShooterCommands.Vals.ANGLE);

    arms.shooterPivot.setGoal(ShooterPivotGoal.IDLING);
    arms.shooterPivot.setGoalPosition((double) data.get(ShooterCommands.Vals.ANGLE));

    rollers.shooter.setGoal(ShooterGoal.IDLING);
    rollers.shooter.setGoalVelocity((double) data.get(ShooterCommands.Vals.RPM));
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
        lastShooterGoal,
        lastIntakePivotGoal,
        lastShooterPivotGoal,
        lastShooterPivotManual,
        lastShooterPivotPosition,
        lastTurretAction,
        lastTurretTarget);
  }
}
