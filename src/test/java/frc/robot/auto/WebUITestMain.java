package frc.robot.auto;

import frc.robot.Config.WebUIConfig;

/**
 * Standalone launcher to test the embedded Web UI on a laptop without deploying to the robot.
 *
 * <p>Usage: ./gradlew runWebUITest [-Dwebui.port=5801 -Dwebui.bindAddress=127.0.0.1]
 */
public class WebUITestMain {
  public static void main(String[] args) throws Exception {
    // Disable NetworkTables in desktop test to avoid native JNI load
    TaskRegistry registry = new TaskRegistry(false);

    // Seed some sample states so the UI has variety on first load
    registry.markClaimed(TaskRegistry.ReefBranch.F1_A);
    registry.markDone(TaskRegistry.ReefBranch.F1_B);

    String host = WebUIConfig.BIND_ADDRESS;
    int port = WebUIConfig.PORT;
    WebUI ui = new WebUI(registry, host, port);
    ui.start();

    System.out.println("WebUI test server running.");
    System.out.println(
        "Open: http://" + ("0.0.0.0".equals(host) ? "localhost" : host) + ":" + port + "/");
    System.out.println("Toggle 'Autonomous Enabled' and click branch buttons to change state.");
    System.out.println("Press Ctrl+C to stop.");

    // Keep process alive
    Thread.currentThread().join();
  }
}
