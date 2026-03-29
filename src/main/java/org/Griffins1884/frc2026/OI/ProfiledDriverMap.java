package org.Griffins1884.frc2026.OI;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.Griffins1884.frc2026.subsystems.objectivetracker.OperatorBoardDataModels;

public final class ProfiledDriverMap implements DriverMap {
  private final PersistedDriverProfileSource profileSource;
  private final Map<Integer, GenericHID> hidByPort = new HashMap<>();

  public ProfiledDriverMap(String fallbackControllerType, int fallbackPort) {
    this.profileSource = new PersistedDriverProfileSource(fallbackControllerType, fallbackPort);
  }

  @Override
  public DoubleSupplier getXAxis() {
    return axisSupplier(DriverBindingTargets.DRIVE_STRAFE);
  }

  @Override
  public DoubleSupplier getYAxis() {
    return axisSupplier(DriverBindingTargets.DRIVE_FORWARD);
  }

  @Override
  public DoubleSupplier getRotAxis() {
    return axisSupplier(DriverBindingTargets.DRIVE_ROTATE);
  }

  @Override
  public Trigger resetOdometry() {
    return actionTrigger(DriverBindingTargets.ACTION_RESET_ODOMETRY);
  }

  @Override
  public Trigger alignWithBall() {
    return actionTrigger(DriverBindingTargets.ACTION_ALIGN_WITH_BALL);
  }

  @Override
  public Trigger shootToggle() {
    return actionTrigger(DriverBindingTargets.ACTION_SHOOT_TOGGLE);
  }

  @Override
  public Trigger intakeRollersHold() {
    return actionTrigger(DriverBindingTargets.ACTION_INTAKE_ROLLERS_HOLD);
  }

  @Override
  public Trigger intakeDeployToggle() {
    return actionTrigger(DriverBindingTargets.ACTION_INTAKE_DEPLOY_TOGGLE);
  }

  @Override
  public Command rumble() {
    return startEnd(
        () -> activeHid().setRumble(kBothRumble, 1.0),
        () -> activeHid().setRumble(kBothRumble, 0.0));
  }

  private DoubleSupplier axisSupplier(String targetId) {
    return () -> {
      PersistedDriverProfileSource.ResolvedProfile profile = profileSource.getActiveProfile();
      OperatorBoardDataModels.JoystickBinding binding = profile.bindingForTarget(targetId);
      if (binding == null || !binding.enabled()) {
        return 0.0;
      }
      return profile
          .controllerLayout()
          .readAxis(hidForPort(profile.controllerPort()), binding.inputId());
    };
  }

  private Trigger actionTrigger(String targetId) {
    return new Trigger(
        () -> {
          PersistedDriverProfileSource.ResolvedProfile profile = profileSource.getActiveProfile();
          OperatorBoardDataModels.JoystickBinding binding = profile.bindingForTarget(targetId);
          if (binding == null || !binding.enabled()) {
            return false;
          }
          return profile
              .controllerLayout()
              .readInputAsBoolean(
                  hidForPort(profile.controllerPort()), binding.inputKind(), binding.inputId());
        });
  }

  private GenericHID activeHid() {
    PersistedDriverProfileSource.ResolvedProfile profile = profileSource.getActiveProfile();
    return hidForPort(profile.controllerPort());
  }

  private GenericHID hidForPort(int port) {
    synchronized (hidByPort) {
      return hidByPort.computeIfAbsent(Math.max(port, 0), GenericHID::new);
    }
  }
}
