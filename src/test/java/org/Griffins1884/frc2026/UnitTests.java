// import static org.junit.jupiter.api.Assertions.*;

// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.simulation.DriverStationSim;
// import edu.wpi.first.wpilibj.simulation.SimHooks;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import java.lang.reflect.Field;
// import java.util.Map;
// import java.util.Optional;
// import org.Griffins1884.frc2026.GlobalConstants;
// import org.Griffins1884.frc2026.commands.AlignConstants;
// import org.Griffins1884.frc2026.commands.DriveCommands;
// import org.Griffins1884.frc2026.commands.ShooterCommands;
// import org.Griffins1884.frc2026.commands.TurretCommands;
// import org.Griffins1884.frc2026.subsystems.Superstructure;
// import org.Griffins1884.frc2026.subsystems.Superstructure.SuperState;
// import org.Griffins1884.frc2026.subsystems.SuperstructureConstants;
// import org.Griffins1884.frc2026.subsystems.climber.ClimberIOSim;
// import org.Griffins1884.frc2026.subsystems.climber.ClimberSubsystem;
// import org.Griffins1884.frc2026.subsystems.indexer.IndexerIOSim;
// import org.Griffins1884.frc2026.subsystems.indexer.IndexerSubsystem;
// import org.Griffins1884.frc2026.subsystems.intake.IntakeIOSim;
// import org.Griffins1884.frc2026.subsystems.intake.IntakePivotIOSim;
// import org.Griffins1884.frc2026.subsystems.intake.IntakePivotSubsystem;
// import org.Griffins1884.frc2026.subsystems.intake.IntakeSubsystem;
// import org.Griffins1884.frc2026.subsystems.shooter.ShooterIOSim;
// import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotIOSim;
// import org.Griffins1884.frc2026.subsystems.shooter.ShooterPivotSubsystem;
// import org.Griffins1884.frc2026.subsystems.shooter.ShooterSubsystem;
// import org.Griffins1884.frc2026.subsystems.swerve.GyroIOSim;
// import org.Griffins1884.frc2026.subsystems.swerve.ModuleIOSim;
// import org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants;
// import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
// import org.Griffins1884.frc2026.subsystems.turret.TurretIOSim;
// import org.Griffins1884.frc2026.subsystems.turret.TurretSubsystem;
// import org.Griffins1884.frc2026.util.TurretUtil;
// import org.ironmaple.simulation.SimulatedArena;
// import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeAll;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Disabled;
// import org.junit.jupiter.api.Test;

// @Disabled
// public class UnitTests {

//   // Mock IO for sensor testing
//   private static class MockIntakeIOSim extends IntakeIOSim {
//     public double overrideCurrent = -1;

//     public MockIntakeIOSim() {
//       super(DCMotor.getNeoVortex(2), 1, 1);
//     }

//     @Override
//     public void updateInputs(GenericRollerSystemIOInputs inputs) {
//       super.updateInputs(inputs);
//       if (overrideCurrent >= 0) {
//         inputs.supplyCurrentAmps = overrideCurrent;
//       }
//     }
//   }

//   private static MockIntakeIOSim mockIntakeIO;

//   private static SwerveSubsystem swerve;
//   private static Superstructure superstructure;
//   private static TurretSubsystem turret;
//   private static SimulatedArena arena;
//   private static final double POSE_TOLERANCE = 0.1; // 10cm
//   private static final double ANGLE_TOLERANCE = 2.0; // 2 degrees
//   private static final double VOLTAGE_TOLERANCE = 0.1; // 0.1V
//   private static final double VELOCITY_TOLERANCE = 0.1; // 0.1 rad/s
//   private static final double ELEVATOR_TOLERANCE = 0.01; // 1cm
//   private static final double TURRET_TOLERANCE = 0.01; // 0.01 rad

//   @BeforeAll
//   public static void setupAll() {
//     assert HAL.initialize(500, 0);
//     try {
//       Field f = SimulatedArena.class.getDeclaredField("instance");
//       f.setAccessible(true);
//       f.set(null, null);
//     } catch (NoSuchFieldException | IllegalAccessException e) {
//     }

//     arena = SimulatedArena.getInstance();
//     SwerveDriveSimulation sim =
//         new SwerveDriveSimulation(SwerveConstants.MAPLE_SIM_CONFIG, new Pose2d());
//     arena.addDriveTrainSimulation(sim);
//     swerve =
//         new SwerveSubsystem(
//             new GyroIOSim(sim.getGyroSimulation()),
//             new ModuleIOSim(sim.getModules()[0]),
//             new ModuleIOSim(sim.getModules()[1]),
//             new ModuleIOSim(sim.getModules()[2]),
//             new ModuleIOSim(sim.getModules()[3]));
//     superstructure = new Superstructure(swerve);
//     turret = new TurretSubsystem(new TurretIOSim());
//     superstructure.setTurret(turret);

//     // Register subsystems so periodic() runs
//     if (!isSubsystemRegistered(swerve)) {
//       CommandScheduler.getInstance().registerSubsystem(swerve);
//     }
//     if (!isSubsystemRegistered(superstructure)) {
//       CommandScheduler.getInstance().registerSubsystem(superstructure);
//     }
//     if (!isSubsystemRegistered(turret)) {
//       CommandScheduler.getInstance().registerSubsystem(turret);
//     }

//     // Manual initialization of subsystems disabled in Config
//     mockIntakeIO = new MockIntakeIOSim();
//     superstructure.getRollers().intake = new IntakeSubsystem("Intake", mockIntakeIO);
//     superstructure.getRollers().shooter =
//         new ShooterSubsystem("Shooter", new ShooterIOSim(DCMotor.getNeoVortex(2), 1, 1));
//     superstructure.getRollers().indexer =
//         new IndexerSubsystem("Indexer", new IndexerIOSim(DCMotor.getNeoVortex(2), 1, 1));
//     superstructure.getElevators().climber = new ClimberSubsystem("Climber", new ClimberIOSim());
//     superstructure.getArms().intakePivot =
//         new IntakePivotSubsystem("IntakePivot", new IntakePivotIOSim());
//     superstructure.getArms().shooterPivot =
//         new ShooterPivotSubsystem("ShooterPivot", new ShooterPivotIOSim());
//   }

//   @BeforeEach
//   public void setup() {
//     SimHooks.pauseTiming();

//     // Reset singletons
//     CommandScheduler.getInstance().cancelAll();
//     CommandScheduler.getInstance().run(); // Flush cancelled commands

//     // Reset robot state
//     swerve.resetOdometry(new Pose2d());
//     mockIntakeIO.overrideCurrent = -1;
//     CommandScheduler.getInstance().schedule(superstructure.setSuperStateCmd(SuperState.IDLING));
//     for (int i = 0; i < 5; i++) {
//       runSimCycle();
//     }

//     DriverStationSim.setEnabled(true);
//   }

//   @AfterEach
//   public void shutdown() {
//     SimHooks.resumeTiming();
//     CommandScheduler.getInstance().cancelAll();
//   }

//   private static boolean isSubsystemRegistered(Subsystem subsystem) {
//     try {
//       Field f = CommandScheduler.class.getDeclaredField("m_subsystems");
//       f.setAccessible(true);
//       Map<Subsystem, Command> subsystems =
//           (Map<Subsystem, Command>) f.get(CommandScheduler.getInstance());
//       return subsystems.containsKey(subsystem);
//     } catch (NoSuchFieldException | IllegalAccessException e) {
//       e.printStackTrace();
//       return false;
//     }
//   }

//   /**
//    * Runs a single simulation cycle in the correct order. This is the key to fixing the tests.
//    *
//    * <p>The CommandScheduler only runs `periodic()` on registered subsystems. The subsystems
// created
//    * inside `Superstructure` are not registered, so their `periodic()` methods (which update
// their
//    * internal physics sims) must be called manually.
//    */
//   private void setState(SuperState state) {
//     superstructure.requestState(state, false);
//     superstructure.periodic();
//     runSimCycle(); // Ensure state is applied and periodic runs once
//     assertEquals(state, superstructure.getCurrentState());
//   }

//   private void runSimCycle() {
//     // 1. Advance the simulation clock
//     SimHooks.stepTiming(0.02);

//     // 2. Advance the physics simulation using the voltages that were just set
//     arena.simulationPeriodic();

//     // 3. Manually run periodic() on internal subsystems since they aren't registered
//     if (superstructure.getRollers().intake != null)
// superstructure.getRollers().intake.periodic();
//     if (superstructure.getRollers().shooter != null)
// superstructure.getRollers().shooter.periodic();
//     if (superstructure.getRollers().indexer != null)
// superstructure.getRollers().indexer.periodic();
//     if (superstructure.getElevators().climber != null)
//       superstructure.getElevators().climber.periodic();
//     if (superstructure.getArms().intakePivot != null)
//       superstructure.getArms().intakePivot.periodic();
//     if (superstructure.getArms().shooterPivot != null)
//       superstructure.getArms().shooterPivot.periodic();

//     // 4. Run `periodic()` on all registered subsystems and update commands
//     CommandScheduler.getInstance().run();
//   }

//   // Swerve & SysID
//   @Test
//   public void testSubsystemInitialization() {
//     assertNotEquals(null, swerve);
//     CommandScheduler.getInstance().run();
//     Pose2d initialPose = swerve.getPose();
//     assertEquals(0, initialPose.getX(), POSE_TOLERANCE);
//     assertEquals(0, initialPose.getY(), POSE_TOLERANCE);
//     assertEquals(0, initialPose.getRotation().getDegrees(), ANGLE_TOLERANCE);
//   }

//   @Test
//   public void testMovesForward() {
//     for (int i = 0; i < 50; i++) {
//       swerve.runVelocity(new ChassisSpeeds(1, 0, 0));
//       runSimCycle();
//     }
//     System.out.println(swerve.getPose());
//     assertTrue(swerve.getPose().getX() > 0.5); // Moved at least 0.5m
//     assertEquals(0, swerve.getPose().getY(), POSE_TOLERANCE);
//   }

//   @Test
//   public void testMovesLeft() {
//     for (int i = 0; i < 50; i++) {
//       swerve.runVelocity(new ChassisSpeeds(0, 1, 0));
//       runSimCycle();
//     }

//     assertTrue(swerve.getPose().getY() > 0.5); // Moved at least 0.5m
//     assertEquals(0, swerve.getPose().getX(), POSE_TOLERANCE);
//   }

//   @Test
//   public void testRotates() {
//     for (int i = 0; i < 50; i++) {
//       swerve.runVelocity(new ChassisSpeeds(0, 0, 1));
//       runSimCycle();
//     }
//     assertTrue(swerve.getPose().getRotation().getDegrees() > 20); // Rotated at least 20 deg
//   }

//   @Test
//   public void testVelocityLimits() {
//     swerve.runVelocity(new ChassisSpeeds(100, 100, 100));
//     runSimCycle();

//     assertTrue(
//         swerve.getRobotRelativeSpeeds().vxMetersPerSecond <= SwerveConstants.MAX_LINEAR_SPEED);
//     assertTrue(
//         swerve.getRobotRelativeSpeeds().vyMetersPerSecond <= SwerveConstants.MAX_LINEAR_SPEED);
//     assertTrue(
//         swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond <=
// SwerveConstants.MAX_ANGULAR_SPEED);
//   }

//   @Test
//   public void testStops() {
//     // Move a bit first
//     for (int i = 0; i < 25; i++) {
//       swerve.runVelocity(new ChassisSpeeds(1, 1, 1));
//       runSimCycle();
//     }

//     // Stop and check
//     for (int i = 0; i < 50; i++) {
//       swerve.stop();
//       runSimCycle();
//     }
//     Pose2d lastPose = swerve.getPose();
//     for (int i = 0; i < 50; i++) {
//       swerve.stop();
//       runSimCycle();
//     }

//     assertEquals(lastPose.getX(), swerve.getPose().getX(), POSE_TOLERANCE);
//     assertEquals(lastPose.getY(), swerve.getPose().getY(), POSE_TOLERANCE);
//     assertEquals(
//         lastPose.getRotation().getDegrees(),
//         swerve.getPose().getRotation().getDegrees(),
//         ANGLE_TOLERANCE);
//   }

//   @Test
//   public void testStopsWithX() {
//     // Move a bit first
//     for (int i = 0; i < 25; i++) {
//       swerve.runVelocity(new ChassisSpeeds(1, 0, 0));
//       runSimCycle();
//     }

//     // Stop with X and check
//     swerve.stopWithX();
//     runSimCycle();

//     Pose2d lastPose = swerve.getPose();
//     for (int i = 0; i < 50; i++) {
//       swerve.stopWithX();
//       runSimCycle();
//     }

//     assertEquals(lastPose.getX(), swerve.getPose().getX(), POSE_TOLERANCE);
//     assertEquals(lastPose.getY(), swerve.getPose().getY(), POSE_TOLERANCE);
//     assertEquals(
//         lastPose.getRotation().getDegrees(),
//         swerve.getPose().getRotation().getDegrees(),
//         ANGLE_TOLERANCE);

//     for (int i = 0; i < 4; i++) {
//       double expectedAngleRad = SwerveConstants.MODULE_TRANSLATIONS[i].getAngle().getRadians();
//       double actualAngleRad = swerve.getModuleStates()[i].angle.getRadians();
//       double diff = Math.abs(expectedAngleRad - actualAngleRad) % (2 * Math.PI);
//       assertTrue(Math.min(diff, 2 * Math.PI - diff) < Math.toRadians(10)); // 10 deg tolerance
//     }
//   }

//   @Test
//   public void testSysIdWiring() {
//     assertNotEquals(null, swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//     assertNotEquals(null, swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
//   }

//   @Test
//   public void testDriverDeadband() {
//     // Test that values under the deadband do not move the robot
//     Pose2d initialPose = swerve.getPose();
//     for (int i = 0; i < 10; i++) {
//       DriveCommands.joystickDrive(
//           swerve,
//           () -> AlignConstants.ALIGN_MANUAL_DEADBAND.getAsDouble() / 2.0,
//           () -> 0.0,
//           () -> AlignConstants.ALIGN_MANUAL_DEADBAND.getAsDouble() / 2.0);
//       runSimCycle();
//     }
//     Pose2d poseAfterSmallInput = swerve.getPose();
//     assertEquals(initialPose.getX(), poseAfterSmallInput.getX(), POSE_TOLERANCE);
//     assertEquals(initialPose.getY(), poseAfterSmallInput.getY(), POSE_TOLERANCE);
//     assertEquals(
//         initialPose.getRotation().getDegrees(),
//         poseAfterSmallInput.getRotation().getDegrees(),
//         ANGLE_TOLERANCE);

//     // Test that values over the deadband move the robot
//     for (int i = 0; i < 10; i++) {
//       DriveCommands.joystickDrive(swerve, () -> 0.5, () -> 0.0, () -> 0.0);
//       runSimCycle();
//     }
//     assertTrue(swerve.getPose().getX() > poseAfterSmallInput.getX() + 0.01);
//   }

//   @Test
//   public void testFieldRelativeSwerve() {
//     // Set robot to be rotated 90 degrees
//     Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
//     swerve.resetOdometry(startPose);
//     runSimCycle(); // Update pose estimator
//     assertEquals(90, swerve.getPose().getRotation().getDegrees(), ANGLE_TOLERANCE);
//     assertEquals(0, swerve.getPose().getX(), POSE_TOLERANCE);
//     assertEquals(0, swerve.getPose().getY(), POSE_TOLERANCE);

//     // Command forward in field-relative frame (positive X)
//     for (int i = 0; i < 50; i++) {
//       // joystickDrive takes suppliers for x, y, omega.
//       DriveCommands.joystickDrive(swerve, () -> 0.5, () -> 0.0, () -> 0.0);
//       runSimCycle();
//     }

//     // Robot should have moved along the field's X axis, not the robot's X axis (which is now the
//     // field's Y axis)

//     assertTrue(swerve.getPose().getX() > 0.2); // Moved forward along field's X
//     assertTrue(
//         swerve.getPose().getRotation().getDegrees() >= 89
//             && swerve.getPose().getRotation().getDegrees() <= 101);
//   }

//   @Test
//   public void testSlowModeScaling() {
//     // Full speed
//     for (int i = 0; i < 50; i++) {
//       DriveCommands.joystickDrive(swerve, () -> 1.0, () -> 0.0, () -> 0.0);
//       runSimCycle();
//     }
//     Pose2d fullSpeedPose = swerve.getPose();
//     assertTrue(fullSpeedPose.getX() > 0.5);

//     // Reset pose
//     swerve.resetOdometry(new Pose2d());
//     runSimCycle();

//     // Slow mode (1/3 speed)
//     for (int i = 0; i < 50; i++) {
//       DriveCommands.joystickDrive(swerve, () -> 1.0 / 3.0, () -> 0.0, () -> 0.0);
//       runSimCycle();
//     }
//     Pose2d slowModePose = swerve.getPose();

//     assertTrue(slowModePose.getX() > 0.1);
//     // Check that it moved roughly 1/9 of the distance due to squared inputs.
//     // Increased tolerance for ramp up/down.
//     assertEquals(fullSpeedPose.getX() / 3.0, slowModePose.getX(), 0.5);
//   }

//   @Test
//   public void testGyroReset() {
//     // Rotate the robot
//     for (int i = 0; i < 50; i++) {
//       swerve.runVelocity(new ChassisSpeeds(0, 0, 1));
//       runSimCycle();
//     }
//     assertTrue(swerve.getPose().getRotation().getDegrees() > 20);
//     Pose2d poseBeforeReset = swerve.getPose();

//     // Reset odometry (heading only)
//     swerve.resetOdometry(new Pose2d(poseBeforeReset.getTranslation(), new Rotation2d()));
//     runSimCycle();

//     // Check that pose rotation is reset
//     assertEquals(0, swerve.getPose().getRotation().getDegrees(), ANGLE_TOLERANCE);
//     // Check that pose translation is maintained
//     assertEquals(poseBeforeReset.getX(), swerve.getPose().getX(), POSE_TOLERANCE);
//     assertEquals(poseBeforeReset.getY(), swerve.getPose().getY(), POSE_TOLERANCE);
//   }

//   // Subsystems
//   @Test
//   public void testIndexer() {

//
// CommandScheduler.getInstance().schedule(superstructure.setSuperStateCmd(SuperState.SHOOTING));
//     for (int i = 0; i < 50; i++) {
//       runSimCycle();
//     }
//     assertEquals(
//         IndexerSubsystem.IndexerGoal.FORWARD.getVoltageSupplier().getAsDouble(),
//         superstructure.getRollers().indexer.getAppliedVolts(),
//         VOLTAGE_TOLERANCE);
//   }

//   @Test
//   public void testShooter() {
//
// CommandScheduler.getInstance().schedule(superstructure.setSuperStateCmd(SuperState.SHOOTING));
//     for (int i = 0; i < 50; i++) {
//       runSimCycle();
//     }
//     assertEquals(
//         ShooterSubsystem.ShooterGoal.FORWARD.getVoltageSupplier().getAsDouble(),
//         superstructure.getRollers().shooter.getAppliedVolts(),
//         VOLTAGE_TOLERANCE);
//   }

//   @Test
//   public void testClimber() {
//     CommandScheduler.getInstance()
//         .schedule(superstructure.setSuperStateCmd(SuperState.ENDGAME_CLIMB));
//     for (int i = 0; i < 200; i++) {
//       runSimCycle();
//     }
//     assertEquals(
//         SuperstructureConstants.CLIMB_PULL_HEIGHT_METERS.get(),
//         superstructure.getElevators().climber.getPositionMeters(),
//         ELEVATOR_TOLERANCE);
//   }

//   @Test
//   public void testIntake() {

//
// CommandScheduler.getInstance().schedule(superstructure.setSuperStateCmd(SuperState.INTAKING));
//     for (int i = 0; i < 50; i++) {
//       runSimCycle();
//     }
//     assertEquals(
//         IntakeSubsystem.intakeGoal.FORWARD.getVoltageSupplier().getAsDouble(),
//         superstructure.getRollers().intake.getAppliedVolts(),
//         VOLTAGE_TOLERANCE);
//   }

//   @Test
//   public void testTurret() {
//
// CommandScheduler.getInstance().schedule(superstructure.setSuperStateCmd(SuperState.SHOOTING));
//     for (int i = 0; i < 100; i++) {
//       runSimCycle();
//     }
//     assertTrue(Math.abs(turret.getPositionRad()) > 0);
//   }

//   // Commands
//   @Test
//   public void testShooterCommands() {
//     double angle =
//         ShooterCommands.calc(
//             new Pose2d(1, 1, new Rotation2d()),
//             GlobalConstants.FieldConstants.Hub.topCenterPoint.toTranslation2d());
//     superstructure.aimShooterPivotAt(
//         GlobalConstants.FieldConstants.Hub.topCenterPoint.toTranslation2d());
//     assertEquals(superstructure.getArms().shooterPivot.getPosition(), angle, 0.001);
//   }

//   @Test
//   public void testTurretCommands() {
//     Pose2d robotPose = new Pose2d(1, 1, new Rotation2d());
//     Translation2d target = new Translation2d(5, 5);
//     Command command =
//         TurretCommands.autoAimToTarget(turret, () -> robotPose, (pose) -> Optional.of(target));
//     CommandScheduler.getInstance().schedule(command);
//     for (int i = 0; i < 100; i++) {
//       runSimCycle();
//     }
//     double expectedAngle = TurretUtil.turretAngleToTarget(robotPose, target);
//     assertEquals(expectedAngle, turret.getPositionRad(), TURRET_TOLERANCE);
//   }

//   // Sensors & Fault
//   @Test
//   public void testSensorToggles() {
//     // Test ball presence logic using current sensing override
//     mockIntakeIO.overrideCurrent = 0.0;
//     runSimCycle();
//     assertFalse(superstructure.hasBall(), "Ball should not be detected when current is low");

//     // Set current above threshold (e.g. 20A, assuming threshold is likely lower like 10-15A)
//     // Checking Constant: BALL_PRESENT_CURRENT_AMPS is likely around 10-20
//     mockIntakeIO.overrideCurrent = 40.0;
//     // Debounce needs time
//     for (int i = 0; i < 20; i++) {
//       runSimCycle();
//     }
//     assertTrue(superstructure.hasBall(), "Ball should be detected when current is high");

//     mockIntakeIO.overrideCurrent = 0.0;
//     for (int i = 0; i < 20; i++) {
//       runSimCycle();
//     }
//     assertFalse(superstructure.hasBall(), "Ball should not be detected after current drops");
//   }

//   @Test
//   @Disabled
//   public void testMissingSensorFault() {
//     // TODO: Implement missing sensor
//   }

//   @Test
//   @Disabled
//   public void testDisconnectedControllerFault() {
//     // TODO: Implement disconnected controller
//   }

//   @Test
//   @Disabled
//   public void testCurrentClampFault() {
//     // TODO: Implement current clamp
//   }

//   @Test
//   public void testNullSubsystemGating() {
//     // Temporarily null out a subsystem and ensure no crash
//     var originalIntake = superstructure.getRollers().intake;
//     superstructure.getRollers().intake = null;

//     try {
//
// CommandScheduler.getInstance().schedule(superstructure.setSuperStateCmd(SuperState.INTAKING));
//       runSimCycle();
//       assertEquals(SuperState.INTAKING, superstructure.getCurrentState());
//     } finally {
//       superstructure.getRollers().intake = originalIntake; // Restore
//     }
//   }

//   // Driver Control
//   @Test
//   @Disabled
//   public void testButtonBindings() {
//     // TODO: Implement tests to ensure each button triggers the intended command
//   }

//   @Test
//   @Disabled
//   public void testNoBindingConflicts() {
//     // TODO: Implement tests to ensure there are no conflicts
//   }

//   @Test
//   @Disabled
//   public void testKeyOutputsLogged() {
//     // TODO: Implement tests to ensure key outputs are recorded
//   }

//   @Test
//   @Disabled
//   public void testNoMissingCriticalChannels() {
//     // TODO: Implement tests to ensure there are no missing critical channels
//   }

//   // Test superstructure states
//   @Test
//   public void testIdling() {
//     setState(SuperState.IDLING);
//     assertEquals(SuperState.IDLING, superstructure.getCurrentState());
//   }

//   @Test
//   public void testIntaking() {
//     setState(SuperState.INTAKING);
//     assertEquals(SuperState.INTAKING, superstructure.getCurrentState());
//   }

//   @Test
//   public void testShooting() {
//     setState(SuperState.SHOOTING);
//     assertEquals(SuperState.SHOOTING, superstructure.getCurrentState());
//   }

//   @Test
//   public void testFerrying() {
//     setState(SuperState.FERRYING);
//     assertEquals(SuperState.FERRYING, superstructure.getCurrentState());
//   }

//   @Test
//   public void testEndgameClimb() {
//     setState(SuperState.ENDGAME_CLIMB);
//     assertEquals(SuperState.ENDGAME_CLIMB, superstructure.getCurrentState());
//   }

//   @Test
//   public void testAutoClimb() {
//     setState(SuperState.AUTO_CLIMB);
//     assertEquals(SuperState.AUTO_CLIMB, superstructure.getCurrentState());
//   }

//   @Test
//   public void testClimbDetach() {
//     setState(SuperState.CLIMB_DETACH);
//     assertEquals(SuperState.CLIMB_DETACH, superstructure.getCurrentState());
//   }

//   @Test
//   public void testTesting() {
//     setState(SuperState.TESTING);
//     assertEquals(SuperState.TESTING, superstructure.getCurrentState());
//   }

//   // TODO: Implement tests for commands

//   @Test
//   public void testAlignCommands() {
//     // Test align command generation
//     Command alignCmd = DriveCommands.alignToClimbCommand(swerve);
//     assertNotNull(alignCmd);
//     // We don't fully run the align command as it depends on PathPlanner/Pose logic which is
// complex
//     // in simple unit test,
//     // but ensuring it constructs without error is a good first step.
//     CommandScheduler.getInstance().schedule(Commands.runOnce(() -> {}).andThen(alignCmd));
//     runSimCycle();
//   }

//   // TODO: Implement pending drive commands tests
//   @Test
//   public void testJoystickDriveAtAngle() {
//     // Test drive at angle command
//     Command cmd =
//         DriveCommands.joystickDriveAtAngle(
//             swerve, () -> 0.0, () -> 0.0, () -> Rotation2d.fromDegrees(90));
//     CommandScheduler.getInstance().schedule(cmd);

//     // run for longer to reach setpoint
//     for (int i = 0; i < 100; i++) runSimCycle();

//     // Should rotate towards 90
//     assertEquals(90.0, swerve.getRotation().getDegrees(), 15.0);
//   }

//   // TODO: Implement tests for vision & autonomous
// }
