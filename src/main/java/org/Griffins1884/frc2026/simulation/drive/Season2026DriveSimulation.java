package org.Griffins1884.frc2026.simulation.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.DoubleSupplier;
import org.griffins1884.sim3d.ChassisFootprint;
import org.griffins1884.sim3d.ChassisMassProperties;
import org.griffins1884.sim3d.CommandableDriveSimulationAdapter;
import org.griffins1884.sim3d.DriveSimulationAdapter;
import org.griffins1884.sim3d.SwerveCorner;
import org.griffins1884.sim3d.TerrainAwareSwerveSimulation;
import org.griffins1884.sim3d.TerrainContactModel;
import org.griffins1884.sim3d.TerrainContactSample;
import org.griffins1884.sim3d.TerrainDriveLaws;
import org.griffins1884.sim3d.TerrainFeature;
import org.griffins1884.sim3d.TerrainSample;
import org.griffins1884.sim3d.integration.DriveSimulationFactories;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/** Season2026-owned boundary around the current drivetrain simulation implementation. */
public final class Season2026DriveSimulation {
  private final CommandableDriveSimulationAdapter commandable;
  private final TerrainAwareSwerveSimulation terrainAware;

  private Season2026DriveSimulation(
      CommandableDriveSimulationAdapter commandable, TerrainAwareSwerveSimulation terrainAware) {
    this.commandable = commandable;
    this.terrainAware = terrainAware;
  }

  public static Season2026DriveSimulation mapleTerrainAware(
      SwerveDriveSimulation mapleSimulation,
      TerrainContactModel terrainContactModel,
      ChassisFootprint chassisFootprint,
      ChassisMassProperties chassisMassProperties,
      DoubleSupplier clockSecondsSupplier) {
    DriveSimulationAdapter adapter =
        DriveSimulationFactories.mapleTerrainAware(
            mapleSimulation,
            terrainContactModel,
            chassisFootprint,
            chassisMassProperties,
            clockSecondsSupplier);
    if (!(adapter instanceof CommandableDriveSimulationAdapter commandable)) {
      throw new IllegalStateException("Selected GriffinSim backend does not expose command hooks.");
    }
    if (!(adapter instanceof TerrainAwareSwerveSimulation terrainAware)) {
      throw new IllegalStateException(
          "Current SIM IO wiring still requires the Maple-backed TerrainAwareSwerveSimulation.");
    }
    return new Season2026DriveSimulation(commandable, terrainAware);
  }

  public void resetState(Pose2d pose, ChassisSpeeds speeds) {
    commandable.resetState(pose, speeds);
  }

  public SwerveDriveSimulation mapleSimulation() {
    return terrainAware.mapleSimulation();
  }

  public SwerveModuleSimulation module(int index) {
    return terrainAware.getModules()[index];
  }

  public Pose3d getPose3d() {
    return terrainAware.getPose3d();
  }

  public Rotation2d gyroReading() {
    return terrainAware.getGyroSimulation().getGyroReading();
  }

  public AngularVelocity measuredAngularVelocity() {
    return terrainAware.getGyroSimulation().getMeasuredAngularVelocity();
  }

  public Rotation2d[] cachedGyroReadings() {
    return terrainAware.getGyroSimulation().getCachedGyroReadings();
  }

  public void setGyroRotation(Rotation2d rotation) {
    terrainAware.getGyroSimulation().setRotation(rotation);
  }

  public TerrainSampleSnapshot terrainSample() {
    return TerrainSampleSnapshot.from(terrainAware.getTerrainSample());
  }

  public double pitchRateRadPerSec() {
    return terrainAware.getPitchRateRadPerSec();
  }

  public double rollRateRadPerSec() {
    return terrainAware.getRollRateRadPerSec();
  }

  public double driveAuthorityScale(Season2026SwerveCorner corner) {
    return TerrainDriveLaws.driveAuthorityScale(
        terrainAware.getTractionState(), toLegacy(corner), terrainAware.getTerrainContactSample());
  }

  public double steerAuthorityScale(Season2026SwerveCorner corner) {
    return TerrainDriveLaws.steerAuthorityScale(
        terrainAware.getTractionState(), toLegacy(corner), terrainAware.getTerrainContactSample());
  }

  public SupportSnapshot supportSnapshot() {
    var support = terrainAware.getSupportDiagnostics();
    return new SupportSnapshot(
        support.supportPlaneHeightMeters(),
        support.chassisZAboveTerrainMeters(),
        support.bodyBottomToTerrainGapMeters(),
        support.terrainHeightMeters(),
        support.bodySupported(),
        support.supportContactCount(),
        support.frontLeftSupported(),
        support.frontRightSupported(),
        support.rearLeftSupported(),
        support.rearRightSupported(),
        support.actualAirborne());
  }

  public ValidationSnapshot validationSnapshot() {
    var chassis = terrainAware.getChassisState3d();
    TerrainContactSample contact = terrainAware.getTerrainContactSample();
    var traction = terrainAware.getTractionState();
    SupportSnapshot support = supportSnapshot();
    return new ValidationSnapshot(
        chassis.pose().getZ(),
        chassis.pose().getRotation().getX(),
        chassis.pose().getRotation().getY(),
        chassis.fieldRelativeLinearVelocityMetersPerSec().getZ(),
        chassis.fieldRelativeLinearAccelerationMetersPerSecSq().getZ(),
        rollRateRadPerSec(),
        pitchRateRadPerSec(),
        support,
        TerrainContactSnapshot.from(contact),
        new TractionSnapshot(
            traction == null || traction.tractionAvailable(),
            traction == null ? Double.NaN : traction.totalNormalForceNewtons(),
            traction == null ? Double.NaN : traction.averageNormalizedLoad(),
            traction == null ? Double.NaN : traction.frontLeft().normalForceNewtons(),
            traction == null ? Double.NaN : traction.frontRight().normalForceNewtons(),
            traction == null ? Double.NaN : traction.rearLeft().normalForceNewtons(),
            traction == null ? Double.NaN : traction.rearRight().normalForceNewtons()));
  }

  private static SwerveCorner toLegacy(Season2026SwerveCorner corner) {
    return switch (corner) {
      case FRONT_LEFT -> SwerveCorner.FRONT_LEFT;
      case FRONT_RIGHT -> SwerveCorner.FRONT_RIGHT;
      case REAR_LEFT -> SwerveCorner.REAR_LEFT;
      case REAR_RIGHT -> SwerveCorner.REAR_RIGHT;
    };
  }

  public record TerrainSampleSnapshot(
      Pose3d pose3d, double rollRadians, double pitchRadians, double heightMeters) {
    private static TerrainSampleSnapshot from(TerrainSample sample) {
      return new TerrainSampleSnapshot(
          sample.pose3d(), sample.rollRadians(), sample.pitchRadians(), sample.heightMeters());
    }
  }

  public record TerrainContactSnapshot(
      String feature,
      double terrainHeightMeters,
      double underbodyClearanceMarginMeters,
      boolean traversableSurface,
      boolean clearanceSatisfied) {
    private static TerrainContactSnapshot from(TerrainContactSample sample) {
      if (sample == null) {
        return new TerrainContactSnapshot(
            TerrainFeature.FLAT.name(), Double.NaN, Double.NaN, false, false);
      }
      return new TerrainContactSnapshot(
          sample.feature().name(),
          sample.terrainSample().heightMeters(),
          sample.underbodyClearanceMarginMeters(),
          sample.traversableSurface(),
          sample.clearanceSatisfied());
    }
  }

  public record SupportSnapshot(
      double supportPlaneHeightMeters,
      double chassisZAboveTerrainMeters,
      double bodyBottomToTerrainGapMeters,
      double terrainHeightMeters,
      boolean bodySupported,
      int supportContactCount,
      boolean frontLeftSupported,
      boolean frontRightSupported,
      boolean rearLeftSupported,
      boolean rearRightSupported,
      boolean actualAirborne) {}

  public record TractionSnapshot(
      boolean tractionAvailable,
      double totalNormalForceNewtons,
      double averageNormalizedLoad,
      double frontLeftNormalForceNewtons,
      double frontRightNormalForceNewtons,
      double rearLeftNormalForceNewtons,
      double rearRightNormalForceNewtons) {}

  public record ValidationSnapshot(
      double chassisZMeters,
      double rollRadians,
      double pitchRadians,
      double verticalVelocityMetersPerSecond,
      double verticalAccelerationMetersPerSecondSq,
      double rollRateRadPerSec,
      double pitchRateRadPerSec,
      SupportSnapshot support,
      TerrainContactSnapshot terrainContact,
      TractionSnapshot traction) {}
}
