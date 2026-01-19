package org.Griffins1884.frc2026.OI;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class PS5DriverMap extends CommandPS5Controller implements DriverMap {
  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public PS5DriverMap(int port) {
    super(port);
  }

  @Override
  public Trigger leftAlign() {
    return L1();
  }

  @Override
  public Trigger rightAlign() {
    return R1();
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
  public Trigger stopWithX() {
    return cross();
  }

  @Override
  public Trigger coralStation() {
    return R2();
  }

  @Override
  public Trigger slowMode() {
    return R2();
  }

  @Override
  public Command rumble() {
    return startEnd(
        () -> getHID().setRumble(kBothRumble, 1), () -> getHID().setRumble(kBothRumble, 0));
  }
}
