package org.Griffins1884.frc2026.subsystems;

import static org.Griffins1884.frc2026.Config.Subsystems.*;
import static org.Griffins1884.frc2026.GlobalConstants.MODE;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import org.Griffins1884.frc2026.subsystems.climber.ClimberSubsystem.ClimberGoal;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerSubsystem.IndexerGoal;
import org.Griffins1884.frc2026.subsystems.intake.IntakePivotSubsystem.IntakePivotGoal;
import org.Griffins1884.frc2026.subsystems.intake.IntakeSubsystem.intakeGoal;
import org.Griffins1884.frc2026.subsystems.leds.LEDIOPWM;
import org.Griffins1884.frc2026.subsystems.leds.LEDIOSim;
import org.Griffins1884.frc2026.subsystems.leds.LEDSubsystem;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotSubsystem.ShooterPivotGoal;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterSubsystem.ShooterGoal;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Superstructure extends SubsystemBase {
  public enum SuperState {
    IDLING,
    INTAKING,
    SHOOTING,
    FERRYING,
    ENDGAME_CLIMB,
    AUTO_CLIMB,
    CLIMB_DETACH,
    TESTING
  }

  private enum ClimbMode {
    ENDGAME,
    AUTO,
    DETACH
  }

  private enum ClimbPhase {
    IDLE,
    EXTEND,
    LATCH,
    PULL,
    DYNAMIC,
    DONE,
    DETACH_EXTEND,
    DETACH_DRIVE,
    DETACH_RETRACT
  }

  private final Supplier<Pose2d> drivePoseSupplier;
  @Setter private TurretSubsystem turret;

  @Getter
  private final LoggedDashboardChooser<SuperState> stateChooser =
      new LoggedDashboardChooser<>("Superstructure State");

  private SuperState requestedState = SuperState.IDLING;
  private SuperState currentState = SuperState.IDLING;
  private boolean stateOverrideActive = false;

  private final Debouncer ballPresentDebouncer =
      new Debouncer(SuperstructureConstants.BALL_PRESENCE_DEBOUNCE_SEC.get(), DebounceType.kBoth);
  private final Timer climbTimer = new Timer();
  private ClimbPhase climbPhase = ClimbPhase.IDLE;
  private int climbLevel = 0;
  private ClimbMode activeClimbMode = null;
  @Setter private boolean climbShootEnabled = false;
  private double climbHoldPosition = Double.NaN;
  @Setter private boolean turretExternalControl = false;

  private final LEDSubsystem leds =
      Config.Subsystems.LEDS_ENABLED
          ? (MODE == GlobalConstants.RobotMode.REAL
              ? new LEDSubsystem(new LEDIOPWM())
              : new LEDSubsystem(new LEDIOSim()))
          : null;
  private final Rollers rollers = new Rollers();
  private final Elevators elevators = new Elevators();
  private final Arms arms = new Arms();
  private static final double SYS_ID_IDLE_WAIT_SECONDS = 0.5;

  public Superstructure(Supplier<Pose2d> drivePoseSupplier) {
    this.drivePoseSupplier = drivePoseSupplier;
    configureStateChooser();
    climbTimer.start();
    if (LEDS_ENABLED) {
      leds.setDefaultCommand(
          leds.ledCommand(
              DriverStation::isEnabled,
              DriverStation::isFMSAttached,
              () -> (DriverStation.getMatchTime() <= 30),
              () -> true,
              () -> false,
              () -> false,
              () -> false));
    }
  }

  public Command setSuperStateCmd(SuperState stateRequest) {
    return Commands.runOnce(() -> requestState(stateRequest, true));
  }

  public void clearStateOverride() {
    stateOverrideActive = false;
  }

  @Override
  public void periodic() {
    updateRequestedStateFromChooser();
    if (requestedState != currentState) {
      enterState(requestedState);
      currentState = requestedState;
    }

    applyState(currentState);
    Logger.recordOutput("Superstructure/State", currentState.toString());
    Logger.recordOutput("Superstructure/RequestedState", requestedState.toString());
    Logger.recordOutput("Superstructure/ClimbPhase", climbPhase.toString());
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
    if (CLIMBER_ENABLED) {
      addSysIdOptions(
          chooser,
          "Climber",
          elevators.climber.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
          elevators.climber.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
          elevators.climber.sysIdDynamic(SysIdRoutine.Direction.kForward),
          elevators.climber.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  private void configureStateChooser() {
    stateChooser.addDefaultOption("Idling", SuperState.IDLING);
    stateChooser.addOption("Intaking", SuperState.INTAKING);
    stateChooser.addOption("Shooting", SuperState.SHOOTING);
    stateChooser.addOption("Ferrying", SuperState.FERRYING);
    stateChooser.addOption("Endgame Climb", SuperState.ENDGAME_CLIMB);
    stateChooser.addOption("Auto Climb", SuperState.AUTO_CLIMB);
    stateChooser.addOption("Climb Detach", SuperState.CLIMB_DETACH);
    stateChooser.addOption("Testing", SuperState.TESTING);
  }

  private void updateRequestedStateFromChooser() {
    if (stateOverrideActive) {
      return;
    }
    SuperState selected = stateChooser.get();
    if (selected != null) {
      requestedState = selected;
    }
  }

  private void requestState(SuperState state, boolean override) {
    requestedState = state;
    if (override) {
      stateOverrideActive = true;
    }
  }

  private void enterState(SuperState state) {
    if (state == SuperState.ENDGAME_CLIMB) {
      startClimb(ClimbMode.ENDGAME);
    } else if (state == SuperState.AUTO_CLIMB) {
      startClimb(ClimbMode.AUTO);
    } else if (state == SuperState.CLIMB_DETACH) {
      startClimb(ClimbMode.DETACH);
    } else if (isClimbState(currentState)) {
      resetClimb();
    }
  }

  private boolean isClimbState(SuperState state) {
    return state == SuperState.ENDGAME_CLIMB
        || state == SuperState.AUTO_CLIMB
        || state == SuperState.CLIMB_DETACH;
  }

  private void applyState(SuperState state) {
    switch (state) {
      case IDLING -> applyIdle();
      case INTAKING -> applyIntaking();
      case SHOOTING -> applyShooting(SuperstructureConstants.getHopperTarget(), true);
      case FERRYING -> applyFerrying();
      case ENDGAME_CLIMB -> applyClimb(ClimbMode.ENDGAME);
      case AUTO_CLIMB -> applyClimb(ClimbMode.AUTO);
      case CLIMB_DETACH -> applyClimb(ClimbMode.DETACH);
      case TESTING -> applyTesting();
    }
  }

  private void applyIdle() {
    clearIdleOverrides();
    setIntakeGoal(intakeGoal.IDLING);
    setIndexerGoal(IndexerGoal.IDLING);
    setShooterGoal(ShooterGoal.IDLING);
    setIntakePivotGoal(IntakePivotGoal.IDLING, false, 0.0);
    setShooterPivotGoal(ShooterPivotGoal.IDLING, false, 0.0);
    holdTurret();
    stopClimber();
  }

  private void clearIdleOverrides() {
    if (rollers.intake != null) {
      rollers.intake.clearGoalOverride();
    }
    if (rollers.indexer != null) {
      rollers.indexer.clearGoalOverride();
    }
    if (rollers.shooter != null) {
      rollers.shooter.stopOpenLoop();
      rollers.shooter.clearGoalOverride();
    }
    if (arms.intakePivot != null) {
      arms.intakePivot.stopOpenLoop();
      arms.intakePivot.clearGoalOverride();
    }
    if (arms.shooterPivot != null) {
      arms.shooterPivot.stopOpenLoop();
      arms.shooterPivot.clearGoalOverride();
    }
  }

  private void applyIntaking() {
    boolean intakeReady = arms.intakePivot == null || arms.intakePivot.isAtGoal();
    setIntakeGoal(intakeReady ? intakeGoal.FORWARD : intakeGoal.IDLING);
    setIndexerGoal(IndexerGoal.IDLING);
    setShooterGoal(ShooterGoal.IDLING);
    setIntakePivotGoal(
        IntakePivotGoal.IDLING, true, SuperstructureConstants.INTAKE_PIVOT_POSITION.get());
    setShooterPivotGoal(ShooterPivotGoal.IDLING, false, 0.0);
    holdTurret();
    stopClimber();
  }

  private void applyShooting(Translation2d target, boolean autoStopOnEmpty) {
    setIntakeGoal(intakeGoal.IDLING);
    setIndexerGoal(IndexerGoal.FORWARD);
    setShooterGoal(ShooterGoal.FORWARD);
    setIntakePivotGoal(IntakePivotGoal.IDLING, false, 0.0);
    aimTurretAt(target);
    aimShooterPivotAt(target);
    stopClimber();

    if (autoStopOnEmpty && SuperstructureConstants.AUTO_STOP_ON_EMPTY && isBallSenseAvailable()) {
      if (!isBallPresent()) {
        requestState(SuperState.IDLING, false);
      }
    }
  }

  private void applyFerrying() {
    Translation2d target = SuperstructureConstants.getFerryTarget();
    setIntakeGoal(intakeGoal.FORWARD);
    setIndexerGoal(IndexerGoal.FORWARD);
    setShooterGoal(ShooterGoal.FORWARD);
    setIntakePivotGoal(
        IntakePivotGoal.IDLING, true, SuperstructureConstants.INTAKE_PIVOT_POSITION.get());
    aimTurretAt(target);
    aimShooterPivotAt(target);
    stopClimber();
  }

  private void applyClimb(ClimbMode mode) {
    if (climbShootEnabled) {
      applyShooting(SuperstructureConstants.getHopperTarget(), false);
    } else {
      setIntakeGoal(intakeGoal.IDLING);
      setIndexerGoal(IndexerGoal.IDLING);
      setShooterGoal(ShooterGoal.IDLING);
      setIntakePivotGoal(IntakePivotGoal.IDLING, false, 0.0);
      setShooterPivotGoal(ShooterPivotGoal.IDLING, false, 0.0);
      holdTurret();
    }
    updateClimb(mode);
  }

  private void applyTesting() {
    setIntakeGoal(intakeGoal.IDLING);
    setIndexerGoal(IndexerGoal.IDLING);
    setShooterGoal(ShooterGoal.IDLING);
    setIntakePivotGoal(IntakePivotGoal.TESTING, false, 0.0);
    setShooterPivotGoal(ShooterPivotGoal.TESTING, false, 0.0);
    holdTurret();
    stopClimber();
  }

  private void setIntakeGoal(intakeGoal goal) {
    if (rollers.intake != null) {
      rollers.intake.setGoal(goal);
    }
  }

  private void setIndexerGoal(IndexerGoal goal) {
    if (rollers.indexer != null) {
      rollers.indexer.setGoal(goal);
    }
  }

  private void setShooterGoal(ShooterGoal goal) {
    if (rollers.shooter != null) {
      rollers.shooter.setGoal(goal);
    }
  }

  private void setIntakePivotGoal(IntakePivotGoal goal, boolean manual, double manualPosition) {
    if (arms.intakePivot == null) {
      return;
    }
    if (manual) {
      arms.intakePivot.setGoalPosition(manualPosition);
    } else {
      arms.intakePivot.clearGoalOverride();
    }
    arms.intakePivot.setGoal(goal);
  }

  private void setShooterPivotGoal(ShooterPivotGoal goal, boolean manual, double manualPosition) {
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
    if (turretExternalControl) {
      return;
    }
    if (turret != null) {
      if (turret.getControlMode() == ControlMode.OPEN_LOOP) {
        return;
      }
      turret.setGoalRad(turret.getPositionRad());
    }
  }

  private void stopClimber() {
    if (elevators.climber != null) {
      elevators.climber.stopOpenLoop();
      elevators.climber.clearGoalOverride();
      elevators.climber.setGoal(ClimberGoal.IDLING);
    }
  }

  private void setClimberGoalPosition(double positionMeters) {
    if (elevators.climber == null) {
      return;
    }
    elevators.climber.setGoalPosition(positionMeters);
    elevators.climber.setGoal(ClimberGoal.IDLING);
  }

  private boolean isClimberAtGoal() {
    return elevators.climber != null && elevators.climber.isAtGoal();
  }

  private void transitionClimbToDone() {
    if (elevators.climber != null) {
      climbHoldPosition = elevators.climber.getPositionMeters();
    }
    climbPhase = ClimbPhase.DONE;
  }

  private void aimTurretAt(Translation2d target) {
    if (turretExternalControl) {
      return;
    }
    if (turret == null || drivePoseSupplier == null || target == null) {
      holdTurret();
      return;
    }
    TurretCommands.autoAimToTarget(
        turret, drivePoseSupplier, (Pose2d) -> java.util.Optional.of(target));
  }

  private void aimShooterPivotAt(Translation2d target) {
    if (arms.shooterPivot == null || drivePoseSupplier == null || target == null) {
      return;
    }
    Pose2d pose = drivePoseSupplier.get();
    if (pose == null) {
      return;
    }

    arms.shooterPivot.setGoalPosition(ShooterCommands.calc(pose));
    arms.shooterPivot.setGoal(ShooterPivotGoal.IDLING);
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
    if (rollers.indexer != null) {
      return rollers.indexer.getSupplyCurrentAmps();
    }
    if (rollers.shooter != null) {
      return rollers.shooter.getSupplyCurrentAmps();
    }
    if (rollers.intake != null) {
      return rollers.intake.getSupplyCurrentAmps();
    }
    return 0.0;
  }

  private void startClimb(ClimbMode mode) {
    activeClimbMode = mode;
    climbLevel = 0;
    climbHoldPosition = Double.NaN;
    climbTimer.reset();
    climbPhase = mode == ClimbMode.DETACH ? ClimbPhase.DETACH_EXTEND : ClimbPhase.EXTEND;
  }

  private void resetClimb() {
    activeClimbMode = null;
    climbLevel = 0;
    climbHoldPosition = Double.NaN;
    climbTimer.reset();
    climbPhase = ClimbPhase.IDLE;
  }

  private void updateClimb(ClimbMode mode) {
    if (activeClimbMode != mode || climbPhase == ClimbPhase.IDLE) {
      startClimb(mode);
    }
    if (elevators.climber == null) {
      return;
    }

    switch (climbPhase) {
      case EXTEND -> {
        setClimberGoalPosition(SuperstructureConstants.CLIMB_EXTEND_HEIGHT_METERS.get());
        if (isClimberAtGoal()) {
          climbPhase = ClimbPhase.LATCH;
        }
      }
      case LATCH -> {
        setClimberGoalPosition(SuperstructureConstants.CLIMB_LATCH_HEIGHT_METERS.get());
        if (isClimberAtGoal()) {
          climbPhase = ClimbPhase.PULL;
        }
      }
      case PULL -> {
        setClimberGoalPosition(SuperstructureConstants.CLIMB_PULL_HEIGHT_METERS.get());
        if (isClimberAtGoal()) {
          climbLevel++;
          if (activeClimbMode == ClimbMode.AUTO
              && climbLevel >= SuperstructureConstants.AUTO_CLIMB_LEVELS) {
            transitionClimbToDone();
          } else if (activeClimbMode == ClimbMode.ENDGAME
              && climbLevel >= SuperstructureConstants.ENDGAME_CLIMB_LEVELS) {
            transitionClimbToDone();
          } else {
            climbPhase = ClimbPhase.DYNAMIC;
          }
        }
      }
      case DYNAMIC -> {
        setClimberGoalPosition(SuperstructureConstants.CLIMB_DYNAMIC_HEIGHT_METERS.get());
        if (isClimberAtGoal()) {
          climbPhase = ClimbPhase.LATCH;
        }
      }
      case DETACH_EXTEND -> {
        setClimberGoalPosition(SuperstructureConstants.CLIMB_DETACH_EXTEND_HEIGHT_METERS.get());
        if (isClimberAtGoal()) {
          climbPhase = ClimbPhase.DETACH_DRIVE;
          climbTimer.reset();
        }
      }
      case DETACH_DRIVE -> {
        setClimberGoalPosition(SuperstructureConstants.CLIMB_DETACH_EXTEND_HEIGHT_METERS.get());
        if (climbTimer.hasElapsed(SuperstructureConstants.CLIMB_DETACH_DRIVE_SECONDS.get())) {
          climbPhase = ClimbPhase.DETACH_RETRACT;
        }
      }
      case DETACH_RETRACT -> {
        setClimberGoalPosition(SuperstructureConstants.CLIMB_DETACH_RETRACT_HEIGHT_METERS.get());
        if (isClimberAtGoal()) {
          transitionClimbToDone();
        }
      }
      case DONE -> {
        if (Double.isNaN(climbHoldPosition)) {
          climbHoldPosition = elevators.climber.getPositionMeters();
        }
        setClimberGoalPosition(climbHoldPosition);
      }
      case IDLE -> stopClimber();
    }
  }

  private void addSysIdOptions(
      LoggedDashboardChooser<Command> chooser,
      String name,
      Command quasistaticForward,
      Command quasistaticReverse,
      Command dynamicForward,
      Command dynamicReverse) {
    chooser.addOption(
        name + " | SysId (Full Routine)",
        sysIdRoutine(name, quasistaticForward, quasistaticReverse, dynamicForward, dynamicReverse));
    chooser.addOption(name + " | SysId (Quasistatic Forward)", quasistaticForward);
    chooser.addOption(name + " | SysId (Quasistatic Reverse)", quasistaticReverse);
    chooser.addOption(name + " | SysId (Dynamic Forward)", dynamicForward);
    chooser.addOption(name + " | SysId (Dynamic Reverse)", dynamicReverse);
  }

  private Command sysIdRoutine(
      String name,
      Command quasistaticForward,
      Command quasistaticReverse,
      Command dynamicForward,
      Command dynamicReverse) {
    return Commands.sequence(
            quasistaticForward,
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            quasistaticReverse,
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            dynamicForward,
            Commands.waitSeconds(SYS_ID_IDLE_WAIT_SECONDS),
            dynamicReverse)
        .withName(name + "SysIdRoutine");
  }
}
