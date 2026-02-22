package org.Griffins1884.frc2026.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import org.Griffins1884.frc2026.generic.rollers.Rollers;
import org.Griffins1884.frc2026.subsystems.indexer.IndexerSubsystem;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterIO;
import org.Griffins1884.frc2026.subsystems.shooter.ShooterSubsystem;
import org.Griffins1884.frc2026.subsystems.turret.TurretIO;
import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
import org.junit.jupiter.api.Test;

class SuperstructureShootingLogicTest {
  @Test
  void movingCompensationRequiresEnableAndValidSample() {
    assertTrue(Superstructure.shouldUseMovingShotCompensation(true, true));
    assertFalse(Superstructure.shouldUseMovingShotCompensation(true, false));
    assertFalse(Superstructure.shouldUseMovingShotCompensation(false, true));
  }

  @Test
  void feedAllowedWhenReadyAndMovingShotDisabled() {
    assertTrue(Superstructure.shouldFeedInShooting(true, true, false, false));
  }

  @Test
  void feedAllowedWhenReadyAndMovingShotEnabledWithValidMotion() {
    assertTrue(Superstructure.shouldFeedInShooting(true, true, true, true));
  }

  @Test
  void feedBlockedWhenMovingShotEnabledButMotionInvalid() {
    assertFalse(Superstructure.shouldFeedInShooting(true, true, true, false));
  }

  @Test
  void feedBlockedWhenAnyActuatorNotReady() {
    assertFalse(Superstructure.shouldFeedInShooting(false, true, false, true));
    assertFalse(Superstructure.shouldFeedInShooting(true, false, false, true));
  }

  @Test
  void applyShooting_rechecksReadinessAfterAimBeforeFeeding() throws Exception {
    Superstructure superstructure = new Superstructure(null);
    try {
      TestTurret turret = new TestTurret();
      turret.setReady(true);
      superstructure.setTurret(turret);

      Rollers rollers = extractRollers(superstructure);
      assertNotNull(rollers);
      rollers.shooter = new AlwaysReadyShooter();
      rollers.indexer = null;
      rollers.intake = null;

      Method applyShooting =
          Superstructure.class.getDeclaredMethod(
              "applyShooting", Translation2d.class, boolean.class);
      applyShooting.setAccessible(true);
      applyShooting.invoke(superstructure, new Translation2d(2.0, 1.0), false);

      Superstructure.SuperstructureOutcome outcome = superstructure.getOutcomeSnapshot();
      assertFalse(
          turret.isAtGoal(),
          "Turret should be marked not-ready immediately after a new goal command.");
      assertTrue(outcome.indexerGoal() == IndexerSubsystem.IndexerGoal.IDLING);
    } finally {
      superstructure.close();
    }
  }

  private static Rollers extractRollers(Superstructure superstructure) throws Exception {
    Field rollersField = Superstructure.class.getDeclaredField("rollers");
    rollersField.setAccessible(true);
    return (Rollers) rollersField.get(superstructure);
  }

  private static class TestTurret extends TurretSubsystem {
    private boolean ready = true;

    private TestTurret() {
      super(new TurretIO() {});
    }

    @Override
    public void setGoalRad(double goalRad) {
      ready = false;
      super.setGoalRad(goalRad);
    }

    @Override
    public boolean isAtGoal() {
      return ready;
    }

    void setReady(boolean value) {
      ready = value;
    }
  }

  private static class AlwaysReadyShooter extends ShooterSubsystem {
    private AlwaysReadyShooter() {
      super("ShooterTest", new ShooterIO() {});
    }

    @Override
    public boolean isAtGoal() {
      return true;
    }
  }
}
