package frc.robot.subsystems.objectivetracker;

import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Standalone launcher to run the Reef controls web server locally for manual testing.
 *
 * <p>Usage: ./gradlew runReefControlsServer
 */
public final class ReefControlsServerMain {
  private ReefControlsServerMain() {}

  public static void main(String[] args) throws InterruptedException {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.stopServer();
    inst.stopClient();
    inst.startLocal();

    new ReefControlsIOServer();

    System.out.println("ReefControls web server running on http://localhost:5801/");
    System.out.println("Press Ctrl+C to stop.");

    Thread.currentThread().join();
  }
}
