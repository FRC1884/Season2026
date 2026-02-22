package org.Griffins1884.frc2026.OI;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class XboxOperatorMap extends CommandXboxController implements OperatorMap {

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public XboxOperatorMap(int port) {
    super(port);
  }

  @Override
  public Command rumble() {
    return startEnd(
        () -> getHID().setRumble(kBothRumble, 1), () -> getHID().setRumble(kBothRumble, 0));
  }

  public Trigger testing() {
    return null;
  }

  @Override
  public Trigger intake() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger shooter() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger idling() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger ferrying() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger intakePivotZero() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger autoManualToggle() {
    return new Trigger(() -> false);
  }

  @Override
  public DoubleSupplier manualTurretAxis() {
    return () -> 0.0;
  }

  @Override
  public DoubleSupplier manualPivotAxis() {
    return () -> 0.0;
  }
}
