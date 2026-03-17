package org.Griffins1884.frc2026.subsystems.intake;

import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.mechanisms.RobotMechanismDefinitions;
import org.Griffins1884.frc2026.mechanisms.rollers.VoltageRollerMechanism;
import org.Griffins1884.frc2026.mechanisms.rollers.VoltageRollerMechanism.VoltageGoal;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

@Setter
@Getter
public class IntakeSubsystem extends VoltageRollerMechanism<IntakeSubsystem.IntakeGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum IntakeGoal implements VoltageGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> IntakeConstants.FORWARD_RPM.get()), // Maximum forward velocity
    REVERSE(() -> IntakeConstants.REVERSE_RPM.get()), // Maximum reverse velocity
    STOW(() -> 4),
    TESTING(new LoggedTunableNumber("Intake/Testing", 0.0));

    private final DoubleSupplier voltageSupplier;

    @Override
    public DoubleSupplier getVoltageSupplier() {
      return voltageSupplier;
    }
  }

  @Setter private IntakeGoal goal = IntakeGoal.IDLING;

  public IntakeSubsystem(String name, IntakeIO io) {
    super(
        name,
        RobotMechanismDefinitions.INTAKE,
        io,
        new VoltageRollerConfig(IntakeConstants.MAX_VOLTAGE));
  }
}
