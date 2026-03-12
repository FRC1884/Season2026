package org.Griffins1884.frc2026.OI;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface DriverMap {
  DoubleSupplier getXAxis();

  DoubleSupplier getYAxis();

  DoubleSupplier getRotAxis();

  Trigger resetOdometry();

  Trigger alignWithBall();

  Trigger slowMode();

  // Placeholder mapping for "start/stop shooting" control.
  Trigger shootToggle();

  // Placeholder mapping for "run intake rollers while held" control.
  Trigger intakeRollersHold();

  // Placeholder mapping for "toggle intake deploy" control.
  Trigger intakeDeployToggle();

  default Trigger leftBackButton() {
    return new Trigger(() -> false);
  }

  default Trigger rightBackButton() {
    return new Trigger(() -> false);
  }

  default Command rumble() {
    return none();
  }
}
