package org.Griffins1884.frc2026.simulation.physics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;
import org.Griffins1884.frc2026.simulation.engine.ArticulatedModuleAssembly;
import org.Griffins1884.frc2026.simulation.engine.CollisionShape;
import org.Griffins1884.frc2026.simulation.engine.ContactState;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMaterial;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Quat;
import org.Griffins1884.frc2026.simulation.engine.PhysicsMath.Vec3;
import org.Griffins1884.frc2026.simulation.engine.PhysicsWorld;
import org.Griffins1884.frc2026.simulation.engine.RigidBody;
import org.Griffins1884.frc2026.simulation.engine.RigidBodyType;
import org.Griffins1884.frc2026.simulation.engine.SwerveWheelContactModel;
import org.Griffins1884.frc2026.simulation.engine.WheelContactTelemetry;
import org.Griffins1884.frc2026.simulation.maple.Rebuilt2026FieldModel;
import org.Griffins1884.frc2026.simulation.sensors.LocalSwerveSensorModel;
import org.Griffins1884.frc2026.simulation.sensors.SwerveSensorConfig;
import org.Griffins1884.frc2026.simulation.sensors.SwerveSensorSample;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants;

/**
 * Deterministic authoritative local physics world for the active WPILib SIM runtime.
 *
 * <p>The robot is represented as an explicit reduced articulated system: - one dynamic chassis
 * rigid body - four dynamic module pod rigid bodies - four deterministic module joint assemblies
 * constraining pod mount and steering alignment - wheel-ground forces applied to the module bodies,
 * not to abstract chassis mount points
 */
public final class LocalSwervePhysicsSimulation {
  public static final int SUBTICKS_PER_PERIOD = 5;
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final double SUBTICK_SECONDS = LOOP_PERIOD_SECONDS / SUBTICKS_PER_PERIOD;

  private static final double MODULE_STEER_ACCEL_RAD_PER_SEC2 =
      SwerveConstants.MAX_STEERING_VELOCITY * 10.0;
  private static final double MODULE_DRIVE_ACCEL_RAD_PER_SEC2 =
      (SwerveConstants.MAX_LINEAR_ACCELERATION / SwerveConstants.getWheelRadiusMeters()) * 1.5;
  private static final double CHASSIS_HALF_HEIGHT_METERS = 0.09;
  private static final double WHEEL_MOUNT_Z_METERS = -0.11;
  private static final double GAMEPIECE_RADIUS_METERS = 0.12;
  private static final double MODULE_HALF_LENGTH_METERS = 0.055;
  private static final double MODULE_HALF_WIDTH_METERS = 0.055;
  private static final double MODULE_HALF_HEIGHT_METERS = 0.03;
  private static final double MODULE_MASS_KG = 3.2;

  private final PhysicsWorld world = new PhysicsWorld(new Vec3(0.0, 0.0, -9.80665));
  private final RigidBody chassisBody;
  private final RigidBody[] moduleBodies = new RigidBody[4];
  private final ArticulatedModuleAssembly[] moduleAssemblies = new ArticulatedModuleAssembly[4];
  private final SwerveWheelContactModel wheelContactModel;
  private final LocalSwerveSensorModel sensorModel =
      new LocalSwerveSensorModel(SwerveSensorConfig.defaults(), 0x1884_2026L);
  private final ModuleState[] modules = {
    new ModuleState(), new ModuleState(), new ModuleState(), new ModuleState()
  };
  private final List<RigidBody> dynamicGamePieces = new ArrayList<>();

  private double simTimeSeconds = 0.0;
  private final double[] cachedTimestamps = new double[SUBTICKS_PER_PERIOD];
  private final Rotation2d[] cachedYawPositions = new Rotation2d[SUBTICKS_PER_PERIOD];
  private final double[][] cachedDrivePositionsRad = new double[4][SUBTICKS_PER_PERIOD];
  private final Rotation2d[][] cachedTurnPositions = new Rotation2d[4][SUBTICKS_PER_PERIOD];
  private final double[][] cachedTurnPositionsRotations = new double[4][SUBTICKS_PER_PERIOD];

  public LocalSwervePhysicsSimulation(Pose2d initialPose) {
    PhysicsMaterial robotMaterial = new PhysicsMaterial(1.1, 0.08, 0.02, 1.0);
    Vec3 chassisHalfExtents =
        new Vec3(
            SwerveConstants.BUMPER_LENGTH * 0.5,
            SwerveConstants.BUMPER_WIDTH * 0.5,
            CHASSIS_HALF_HEIGHT_METERS);
    chassisBody =
        new RigidBody(
            1,
            "chassis",
            RigidBodyType.DYNAMIC,
            CollisionShape.box(
                chassisHalfExtents.x(), chassisHalfExtents.y(), chassisHalfExtents.z()),
            robotMaterial,
            SwerveConstants.ROBOT_MASS,
            inverseBoxInertia(SwerveConstants.ROBOT_MASS, chassisHalfExtents),
            Vec3.ZERO,
            Quat.IDENTITY,
            1);
    world.addBody(chassisBody);
    for (RigidBody fieldBody : Rebuilt2026FieldModel.createStaticCollisionBodies()) {
      world.addBody(fieldBody);
    }

    PhysicsMaterial moduleMaterial = new PhysicsMaterial(1.2, 0.06, 0.02, 1.05);
    for (int moduleIndex = 0; moduleIndex < moduleBodies.length; moduleIndex++) {
      Vec3 mount =
          new Vec3(
              SwerveConstants.MODULE_TRANSLATIONS[moduleIndex].getX(),
              SwerveConstants.MODULE_TRANSLATIONS[moduleIndex].getY(),
              WHEEL_MOUNT_Z_METERS);
      moduleBodies[moduleIndex] =
          new RigidBody(
              100 + moduleIndex,
              "module-" + moduleIndex,
              RigidBodyType.DYNAMIC,
              CollisionShape.box(
                  MODULE_HALF_LENGTH_METERS, MODULE_HALF_WIDTH_METERS, MODULE_HALF_HEIGHT_METERS),
              moduleMaterial,
              MODULE_MASS_KG,
              inverseBoxInertia(
                  MODULE_MASS_KG,
                  new Vec3(
                      MODULE_HALF_LENGTH_METERS,
                      MODULE_HALF_WIDTH_METERS,
                      MODULE_HALF_HEIGHT_METERS)),
              Vec3.ZERO,
              Quat.IDENTITY,
              100 + moduleIndex);
      world.addBody(moduleBodies[moduleIndex]);
      world.ignorePair(chassisBody.id(), moduleBodies[moduleIndex].id());
      moduleAssemblies[moduleIndex] =
          new ArticulatedModuleAssembly(chassisBody, moduleBodies[moduleIndex], mount, Vec3.ZERO);
    }
    wheelContactModel =
        new SwerveWheelContactModel(moduleAssemblies, SwerveConstants.getWheelRadiusMeters());
    resetState(initialPose != null ? initialPose : new Pose2d(), new ChassisSpeeds());
  }

  public synchronized void simulationPeriodic() {
    for (int sample = 0; sample < SUBTICKS_PER_PERIOD; sample++) {
      advanceOneSubtick();
      clampDynamicBodiesToFieldBounds();
      cachedTimestamps[sample] = simTimeSeconds;
      cachedYawPositions[sample] = getPose().getRotation();
      for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
        cachedDrivePositionsRad[moduleIndex][sample] = modules[moduleIndex].drivePositionRad;
        cachedTurnPositions[moduleIndex][sample] =
            Rotation2d.fromRadians(modules[moduleIndex].turnPositionRad);
        cachedTurnPositionsRotations[moduleIndex][sample] =
            modules[moduleIndex].turnPositionRad / (2.0 * Math.PI);
      }
    }
    sensorModel.observe(Math.round(simTimeSeconds * 1_000_000_000.0), captureRawSwerveState());
  }

  public synchronized void resetState(Pose2d pose, ChassisSpeeds chassisSpeeds) {
    Rebuilt2026FieldModel.LocalTerrainSample terrain = Rebuilt2026FieldModel.sample(pose);
    double chassisZ =
        terrain.heightMeters() + SwerveConstants.getWheelRadiusMeters() - WHEEL_MOUNT_Z_METERS;
    chassisBody.setPose(
        new Vec3(pose.getX(), pose.getY(), chassisZ),
        Quat.fromYawPitchRoll(
            pose.getRotation().getRadians(), terrain.pitchRadians(), terrain.rollRadians()));
    chassisBody.setVelocities(
        new Vec3(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0.0),
        new Vec3(0.0, 0.0, chassisSpeeds.omegaRadiansPerSecond));
    simTimeSeconds = 0.0;

    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      ModuleState module = modules[moduleIndex];
      module.drivePositionRad = 0.0;
      module.driveVelocityRadPerSec = 0.0;
      module.driveCommandVelocityRadPerSec = 0.0;
      module.driveAppliedVolts = 0.0;
      module.driveClosedLoop = true;
      module.turnPositionRad = 0.0;
      module.turnVelocityRadPerSec = 0.0;
      module.turnTargetPositionRad = 0.0;
      module.turnAppliedVolts = 0.0;
      module.turnClosedLoop = true;
      module.zeroTrimRotations = 0.0;
      moduleAssemblies[moduleIndex].setDesiredSteerRadians(0.0);
      moduleAssemblies[moduleIndex].resetPoseFromChassis();
    }
    dynamicGamePieces.clear();
    seedCachedSamples();
  }

  public synchronized Pose2d getPose() {
    return new Pose2d(
        chassisBody.position().x(),
        chassisBody.position().y(),
        Rotation2d.fromRadians(chassisBody.orientation().yawRadians()));
  }

  public synchronized Pose3d getPose3d() {
    return new Pose3d(
        chassisBody.position().x(),
        chassisBody.position().y(),
        chassisBody.position().z(),
        chassisBody.orientation().toRotation3d());
  }

  public synchronized double getYawRateRadPerSec() {
    return currentSensorSample().yawVelocityRadPerSec();
  }

  public synchronized Rotation2d getYawRotation() {
    return currentSensorSample().yaw();
  }

  public synchronized Rotation2d getPitchRotation() {
    return currentSensorSample().pitch();
  }

  public synchronized Rotation2d getRollRotation() {
    return currentSensorSample().roll();
  }

  public synchronized double getPitchRadians() {
    return currentSensorSample().pitch().getRadians();
  }

  public synchronized double getRollRadians() {
    return currentSensorSample().roll().getRadians();
  }

  public synchronized double getPitchRateRadPerSec() {
    return currentSensorSample().pitchVelocityRadPerSec();
  }

  public synchronized double getRollRateRadPerSec() {
    return currentSensorSample().rollVelocityRadPerSec();
  }

  public synchronized double getHeightMeters() {
    return chassisBody.position().z();
  }

  public synchronized double[] getCachedTimestamps() {
    return currentSensorSample().cachedTimestamps().clone();
  }

  public synchronized Rotation2d[] getCachedYawPositions() {
    return currentSensorSample().cachedYawPositions().clone();
  }

  public synchronized double[] getCachedDrivePositionsRad(int moduleIndex) {
    return currentSensorSample().drivePositionsRad()[moduleIndex].clone();
  }

  public synchronized Rotation2d[] getCachedTurnPositions(int moduleIndex) {
    return currentSensorSample().turnPositions()[moduleIndex].clone();
  }

  public synchronized double[] getCachedTurnPositionsRotations(int moduleIndex) {
    return currentSensorSample().turnPositionsRotations()[moduleIndex].clone();
  }

  public synchronized double getDrivePositionRad(int moduleIndex) {
    return currentSensorSample().drivePositionsRad()[moduleIndex][SUBTICKS_PER_PERIOD - 1];
  }

  public synchronized double getDriveVelocityRadPerSec(int moduleIndex) {
    return currentSensorSample().driveVelocitiesRadPerSec()[moduleIndex][SUBTICKS_PER_PERIOD - 1];
  }

  public synchronized Rotation2d getTurnPosition(int moduleIndex) {
    return currentSensorSample().turnPositions()[moduleIndex][SUBTICKS_PER_PERIOD - 1];
  }

  public synchronized double getTurnVelocityRadPerSec(int moduleIndex) {
    return currentSensorSample().turnVelocitiesRadPerSec()[moduleIndex][SUBTICKS_PER_PERIOD - 1];
  }

  public synchronized double getDriveAppliedVolts(int moduleIndex) {
    return modules[moduleIndex].driveAppliedVolts;
  }

  public synchronized double getTurnAppliedVolts(int moduleIndex) {
    return modules[moduleIndex].turnAppliedVolts;
  }

  public synchronized double getDriveCurrentAmps(int moduleIndex) {
    return Math.abs(modules[moduleIndex].driveAppliedVolts) * 3.8;
  }

  public synchronized double getTurnCurrentAmps(int moduleIndex) {
    return Math.abs(modules[moduleIndex].turnAppliedVolts) * 1.8;
  }

  public synchronized double getTurnPositionRotations(int moduleIndex) {
    return moduleBodies[moduleIndex].orientation().yawRadians() / (2.0 * Math.PI);
  }

  public synchronized double getZeroTrimRotations(int moduleIndex) {
    return modules[moduleIndex].zeroTrimRotations;
  }

  public synchronized void captureZeroTrim(int moduleIndex) {
    modules[moduleIndex].zeroTrimRotations = getTurnPositionRotations(moduleIndex);
  }

  public synchronized void clearZeroTrim(int moduleIndex) {
    modules[moduleIndex].zeroTrimRotations = 0.0;
  }

  public synchronized void setDriveOpenLoop(int moduleIndex, double volts) {
    ModuleState module = modules[moduleIndex];
    module.driveClosedLoop = false;
    module.driveAppliedVolts = volts;
  }

  public synchronized void setTurnOpenLoop(int moduleIndex, double volts) {
    ModuleState module = modules[moduleIndex];
    module.turnClosedLoop = false;
    module.turnAppliedVolts = volts;
  }

  public synchronized void setDriveVelocity(
      int moduleIndex, double velocityRadPerSec, double feedforwardVolts) {
    ModuleState module = modules[moduleIndex];
    module.driveClosedLoop = true;
    module.driveCommandVelocityRadPerSec = velocityRadPerSec;
    module.driveAppliedVolts = feedforwardVolts;
  }

  public synchronized void setTurnPosition(int moduleIndex, Rotation2d rotation) {
    ModuleState module = modules[moduleIndex];
    module.turnClosedLoop = true;
    module.turnTargetPositionRad = rotation != null ? rotation.getRadians() : 0.0;
    moduleAssemblies[moduleIndex].setDesiredSteerRadians(module.turnTargetPositionRad);
  }

  public synchronized void resetYaw(Rotation2d yaw) {
    Rebuilt2026FieldModel.LocalTerrainSample terrain = Rebuilt2026FieldModel.sample(getPose());
    chassisBody.setPose(
        new Vec3(
            chassisBody.position().x(), chassisBody.position().y(), chassisBody.position().z()),
        Quat.fromYawPitchRoll(
            yaw != null ? yaw.getRadians() : 0.0, terrain.pitchRadians(), terrain.rollRadians()));
    for (ArticulatedModuleAssembly assembly : moduleAssemblies) {
      assembly.resetPoseFromChassis();
    }
    sensorModel.observe(Math.round(simTimeSeconds * 1_000_000_000.0), captureRawSwerveState());
  }

  public synchronized List<Pose3d> getGamePiecePoses() {
    return dynamicGamePieces.stream()
        .map(
            body ->
                new Pose3d(
                    body.position().x(),
                    body.position().y(),
                    body.position().z(),
                    body.orientation().toRotation3d()))
        .toList();
  }

  public synchronized List<WheelContactTelemetry> getWheelContactTelemetry() {
    return wheelContactModel.lastTelemetry();
  }

  public synchronized List<ContactState> getLastContacts() {
    return world.lastContacts();
  }

  public synchronized List<RigidBody> getBodies() {
    return world.bodies();
  }

  public synchronized void spawnGamePiece(Pose3d pose3d, Vec3 linearVelocityMetersPerSecond) {
    int id = 2000 + dynamicGamePieces.size();
    RigidBody gamePiece =
        new RigidBody(
            id,
            "gamepiece-" + id,
            RigidBodyType.DYNAMIC,
            CollisionShape.sphere(GAMEPIECE_RADIUS_METERS),
            new PhysicsMaterial(0.7, 0.25, 0.03, 0.7),
            0.35,
            new Vec3(18.0, 18.0, 18.0),
            new Vec3(pose3d.getX(), pose3d.getY(), pose3d.getZ()),
            Quat.IDENTITY,
            id);
    gamePiece.setVelocities(linearVelocityMetersPerSecond, Vec3.ZERO);
    dynamicGamePieces.add(gamePiece);
    world.addBody(gamePiece);
  }

  public synchronized double getSimTimeSeconds() {
    return simTimeSeconds;
  }

  private void advanceOneSubtick() {
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      ModuleState module = modules[moduleIndex];
      double driveTarget =
          module.driveClosedLoop
              ? module.driveCommandVelocityRadPerSec
              : MathUtil.clamp(module.driveAppliedVolts / 12.0, -1.0, 1.0)
                  * (SwerveConstants.MAX_LINEAR_SPEED / SwerveConstants.getWheelRadiusMeters());
      double driveDelta =
          MathUtil.clamp(
              driveTarget - module.driveVelocityRadPerSec,
              -MODULE_DRIVE_ACCEL_RAD_PER_SEC2 * SUBTICK_SECONDS,
              MODULE_DRIVE_ACCEL_RAD_PER_SEC2 * SUBTICK_SECONDS);
      module.driveVelocityRadPerSec += driveDelta;
      module.drivePositionRad += module.driveVelocityRadPerSec * SUBTICK_SECONDS;

      double turnTargetVelocity =
          module.turnClosedLoop
              ? MathUtil.clamp(
                  MathUtil.angleModulus(module.turnTargetPositionRad - module.turnPositionRad)
                      * 20.0,
                  -SwerveConstants.MAX_STEERING_VELOCITY,
                  SwerveConstants.MAX_STEERING_VELOCITY)
              : MathUtil.clamp(module.turnAppliedVolts / 12.0, -1.0, 1.0)
                  * SwerveConstants.MAX_STEERING_VELOCITY;
      double turnDelta =
          MathUtil.clamp(
              turnTargetVelocity - module.turnVelocityRadPerSec,
              -MODULE_STEER_ACCEL_RAD_PER_SEC2 * SUBTICK_SECONDS,
              MODULE_STEER_ACCEL_RAD_PER_SEC2 * SUBTICK_SECONDS);
      module.turnVelocityRadPerSec += turnDelta;
      module.turnPositionRad =
          MathUtil.angleModulus(
              module.turnPositionRad + module.turnVelocityRadPerSec * SUBTICK_SECONDS);
      moduleAssemblies[moduleIndex].setDesiredSteerRadians(module.turnPositionRad);
      wheelContactModel.setCommand(
          moduleIndex, module.turnPositionRad, module.driveVelocityRadPerSec);
    }

    world.step(
        SUBTICK_SECONDS,
        physicsWorld -> {
          for (ArticulatedModuleAssembly assembly : moduleAssemblies) {
            assembly.applyJointForces();
          }
          wheelContactModel.apply(physicsWorld, SUBTICK_SECONDS);
        });

    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      modules[moduleIndex].turnPositionRad = moduleBodies[moduleIndex].orientation().yawRadians();
      modules[moduleIndex].turnVelocityRadPerSec = moduleBodies[moduleIndex].angularVelocity().z();
    }
    simTimeSeconds += SUBTICK_SECONDS;
  }

  private void clampDynamicBodiesToFieldBounds() {
    double minX = 0.0;
    double maxX = org.Griffins1884.frc2026.GlobalConstants.FieldConstants.fieldLength;
    double minY = 0.0;
    double maxY = org.Griffins1884.frc2026.GlobalConstants.FieldConstants.fieldWidth;
    clampBodyToBounds(chassisBody, minX, maxX, minY, maxY);
    for (RigidBody body : moduleBodies) {
      clampBodyToBounds(body, minX, maxX, minY, maxY);
    }
    for (RigidBody body : dynamicGamePieces) {
      clampBodyToBounds(body, minX, maxX, minY, maxY);
    }
  }

  private void clampBodyToBounds(
      RigidBody body, double minX, double maxX, double minY, double maxY) {
    Vec3 position = body.position();
    double x = MathUtil.clamp(position.x(), minX, maxX);
    double y = MathUtil.clamp(position.y(), minY, maxY);
    if (x != position.x() || y != position.y()) {
      body.setPose(new Vec3(x, y, position.z()), body.orientation());
      body.setVelocities(
          new Vec3(
              x != position.x() ? 0.0 : body.linearVelocity().x(),
              y != position.y() ? 0.0 : body.linearVelocity().y(),
              body.linearVelocity().z()),
          body.angularVelocity());
    }
  }

  private SwerveSensorSample currentSensorSample() {
    SwerveSensorSample sample = sensorModel.current();
    return sample != null ? sample : captureRawSwerveState().toSample(simTimeSeconds);
  }

  private LocalSwerveSensorModel.RawSwerveState captureRawSwerveState() {
    double[][] drivePositions = new double[modules.length][SUBTICKS_PER_PERIOD];
    double[][] driveVelocities = new double[modules.length][SUBTICKS_PER_PERIOD];
    Rotation2d[][] turnPositions = new Rotation2d[modules.length][SUBTICKS_PER_PERIOD];
    double[][] turnVelocities = new double[modules.length][SUBTICKS_PER_PERIOD];
    double[][] turnPositionsRotations = new double[modules.length][SUBTICKS_PER_PERIOD];
    for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
      for (int sample = 0; sample < SUBTICKS_PER_PERIOD; sample++) {
        drivePositions[moduleIndex][sample] = cachedDrivePositionsRad[moduleIndex][sample];
        driveVelocities[moduleIndex][sample] = modules[moduleIndex].driveVelocityRadPerSec;
        turnPositions[moduleIndex][sample] = cachedTurnPositions[moduleIndex][sample];
        turnVelocities[moduleIndex][sample] = modules[moduleIndex].turnVelocityRadPerSec;
        turnPositionsRotations[moduleIndex][sample] =
            cachedTurnPositions[moduleIndex][sample].getRadians() / (2.0 * Math.PI);
      }
    }
    return new LocalSwerveSensorModel.RawSwerveState(
        Rotation2d.fromRadians(chassisBody.orientation().yawRadians()),
        Rotation2d.fromRadians(chassisBody.orientation().toRotation3d().getY()),
        Rotation2d.fromRadians(chassisBody.orientation().toRotation3d().getX()),
        chassisBody.angularVelocity().z(),
        chassisBody.angularVelocity().y(),
        chassisBody.angularVelocity().x(),
        cachedTimestamps.clone(),
        cachedYawPositions.clone(),
        drivePositions,
        driveVelocities,
        turnPositions,
        turnVelocities,
        turnPositionsRotations);
  }

  private void seedCachedSamples() {
    for (int sample = 0; sample < SUBTICKS_PER_PERIOD; sample++) {
      cachedTimestamps[sample] =
          simTimeSeconds - LOOP_PERIOD_SECONDS + ((sample + 1) * SUBTICK_SECONDS);
      cachedYawPositions[sample] = getPose().getRotation();
      for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
        cachedDrivePositionsRad[moduleIndex][sample] = modules[moduleIndex].drivePositionRad;
        cachedTurnPositions[moduleIndex][sample] =
            Rotation2d.fromRadians(modules[moduleIndex].turnPositionRad);
        cachedTurnPositionsRotations[moduleIndex][sample] =
            modules[moduleIndex].turnPositionRad / (2.0 * Math.PI);
      }
    }
    sensorModel.observe(0L, captureRawSwerveState());
  }

  private Vec3 inverseBoxInertia(double massKg, Vec3 halfExtents) {
    double width = halfExtents.x() * 2.0;
    double depth = halfExtents.y() * 2.0;
    double height = halfExtents.z() * 2.0;
    double ixx = (massKg / 12.0) * ((depth * depth) + (height * height));
    double iyy = (massKg / 12.0) * ((width * width) + (height * height));
    double izz = (massKg / 12.0) * ((width * width) + (depth * depth));
    return new Vec3(1.0 / ixx, 1.0 / iyy, 1.0 / izz);
  }

  private static final class ModuleState {
    private double drivePositionRad;
    private double driveVelocityRadPerSec;
    private double driveCommandVelocityRadPerSec;
    private double driveAppliedVolts;
    private boolean driveClosedLoop;
    private double turnPositionRad;
    private double turnVelocityRadPerSec;
    private double turnTargetPositionRad;
    private double turnAppliedVolts;
    private boolean turnClosedLoop;
    private double zeroTrimRotations;
  }
}
