package org.Griffins1884.frc2026.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.generic.rollers.GenericVelocityRollerSystem;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

@Setter
@Getter
public class ShooterSubsystem extends GenericVelocityRollerSystem<ShooterSubsystem.ShooterGoal> {
  @RequiredArgsConstructor
  @Getter
  public enum ShooterGoal implements VelocityGoal {
    IDLING(() -> 0.0), // Intake is off
    FORWARD(() -> ShooterConstants.TARGET_RPM), // Constant flywheel velocity
    REVERSE(() -> -ShooterConstants.TARGET_RPM), // Optional reverse for clearing
    TESTING(new LoggedTunableNumber("Shooter/Testing", 0.0));

    private final DoubleSupplier velocitySupplier;

    @Override
    public DoubleSupplier getVelocitySupplier() {
      return velocitySupplier;
    }
  }

  @Setter private ShooterGoal goal = ShooterGoal.IDLING;
  private Debouncer currentDebouncer = new Debouncer(0.1);
  private boolean highGainsActive = false;

  public ShooterSubsystem(String name, ShooterIO io) {
    super(
        name,
        io,
        new VelocityRollerConfig(
            ShooterConstants.GAINS_LOW,
            ShooterConstants.VELOCITY_TOLERANCE,
            ShooterConstants.MAX_VOLTAGE));
  }

  @Override
  protected GlobalConstants.Gains getActiveGains(double requestedVelocityRpm) {
    double rpm = Math.abs(requestedVelocityRpm);
    double switchRpm = ShooterConstants.GAINS_SWITCH_RPM.get();
    double hysteresis = ShooterConstants.GAINS_SWITCH_HYSTERESIS_RPM.get();
    if (highGainsActive) {
      if (rpm < switchRpm - hysteresis) {
        highGainsActive = false;
      }
    } else {
      if (rpm > switchRpm + hysteresis) {
        highGainsActive = true;
      }
    }
    return highGainsActive ? ShooterConstants.GAINS_HIGH : ShooterConstants.GAINS_LOW;
  }

  @Override
  protected String getActiveGainsLabel(double requestedVelocityRpm) {
    return highGainsActive ? "HIGH" : "LOW";
  }

  @Override
  protected double getAdditionalCompensationVolts(
      double goalVelocityRpm, double measuredVelocityRpm) {
    double goalSign = Math.signum(goalVelocityRpm);
    if (goalSign == 0.0) {
      return 0.0;
    }

    double velocityDeficitRpm =
        Math.max(0.0, Math.abs(goalVelocityRpm) - Math.abs(measuredVelocityRpm));
    if (velocityDeficitRpm <= 0.0) {
      return 0.0;
    }

    double errorCompensationVolts =
        velocityDeficitRpm * ShooterConstants.RECOVERY_ERROR_GAIN_VOLTS_PER_RPM.get();
    double currentOverThresholdAmps =
        Math.max(
            0.0, getSupplyCurrentAmps() - ShooterConstants.RECOVERY_CURRENT_THRESHOLD_AMPS.get());
    double currentCompensationVolts =
        currentOverThresholdAmps * ShooterConstants.RECOVERY_CURRENT_GAIN_VOLTS_PER_AMP.get();
    double totalBoostVolts =
        MathUtil.clamp(
            errorCompensationVolts + currentCompensationVolts,
            0.0,
            ShooterConstants.RECOVERY_MAX_BOOST_VOLTS.get());
    return goalSign * totalBoostVolts;
  }
}
