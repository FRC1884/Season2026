package org.Griffins1884.frc2026.OI;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface OperatorMap {
  default Command rumble() {
    return none();
  }

  Trigger intake();

  Trigger shooter();

  Trigger autoManualToggle();

  Trigger idling();

  Trigger ferrying();

  DoubleSupplier manualTurretAxis();

  DoubleSupplier manualPivotAxis();
}
