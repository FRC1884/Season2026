package frc.robot.auto;

/**
 * Standalone launcher to test the embedded Web UI on a laptop without deploying to the robot.
 *
 * <p>Usage: ./gradlew webUiTest Then open http://localhost:5801/ on your iPad or laptop browser.
 */
public class WebUITestMain {
  public static void main(String[] args) throws Exception {
    // Disable NetworkTables in desktop test to avoid native JNI load
    TaskRegistry registry = new TaskRegistry(false);

    // Seed some sample states so the UI has variety on first load
    registry.markClaimed(TaskRegistry.ReefBranch.F1_A);
    registry.markDone(TaskRegistry.ReefBranch.F1_B);

    // Start the HTTP server on a desktop-friendly port
    int port = 5801;
    WebUI ui = new WebUI(registry, port);
    ui.start();

    System.out.println("WebUI test server running.");
    System.out.println("Open: http://localhost:" + port + "/");
    System.out.println("Toggle 'Autonomous Enabled' and click branch buttons to change state.");
    System.out.println("Press Ctrl+C to stop.");

    // Keep process alive
    Thread.currentThread().join();
  }
}
