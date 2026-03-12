package org.Griffins1884.frc2026.OI;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class PS5ProDriverMap extends CommandPS5Controller implements DriverMap {
  // WPILib doesn't define DualSense Edge rear buttons; these rely on DS exposing raw buttons.
  private static final int LEFT_BACK_BUTTON = 15;
  private static final int RIGHT_BACK_BUTTON = 16;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public PS5ProDriverMap(int port) {
    super(port);
  }

  @Override
  public DoubleSupplier getXAxis() {
    return () -> -getLeftX();
  }

  @Override
  public DoubleSupplier getYAxis() {
    return () -> -getLeftY();
  }

  @Override
  public DoubleSupplier getRotAxis() {
    return () -> -getRightX();
  }

  @Override
  public Trigger resetOdometry() {
    return options();
  }

  @Override
  public Trigger alignWithBall() {
    return circle();
  }

  @Override
  public Trigger slowMode() {
    return L2();
  }

  @Override
  public Trigger shootToggle() {
    return cross();
  }

  @Override
  public Trigger intakeRollersHold() {
    return R1();
  }

  @Override
  public Trigger intakeDeployToggle() {
    return triangle();
  }

  @Override
  public Trigger leftBackButton() {
    return button(LEFT_BACK_BUTTON);
  }

  @Override
  public Trigger rightBackButton() {
    return button(RIGHT_BACK_BUTTON);
  }

  @Override
  public Command rumble() {
    return startEnd(
        () -> getHID().setRumble(kBothRumble, 1), () -> getHID().setRumble(kBothRumble, 0));
  }
}
