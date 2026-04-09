package org.Griffins1884.frc2026.simulation.transport;

import java.io.EOFException;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;
import org.Griffins1884.frc2026.simulation.contracts.FrameBinaryCodec;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.Griffins1884.frc2026.simulation.contracts.SimulationProtocolVersion;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;
import org.Griffins1884.frc2026.simulation.engine.DeterministicShotEngine;
import org.Griffins1884.frc2026.simulation.engine.EngineStepResult;

/** Socket server exposing the deterministic shot engine over the frame transport. */
public final class DeterministicShotEngineServer implements AutoCloseable {
  private final DeterministicShotEngine engine;
  private final ServerSocket serverSocket;

  public DeterministicShotEngineServer(int port) throws IOException {
    this(new DeterministicShotEngine(), port);
  }

  public DeterministicShotEngineServer(DeterministicShotEngine engine, int port)
      throws IOException {
    this.engine = engine;
    serverSocket = new ServerSocket(port);
  }

  public int port() {
    return serverSocket.getLocalPort();
  }

  public void serveOneClient() throws IOException {
    try (Socket socket = serverSocket.accept();
        SocketFrameTransport transport =
            new SocketFrameTransport(socket.getInputStream(), socket.getOutputStream())) {
      while (!socket.isClosed() && !serverSocket.isClosed()) {
        SocketFrameTransport.ReceivedFrame received = transport.receive();
        if (received.frameType() != SimulationProtocolVersion.FRAME_TYPE_ACTUATOR) {
          throw new IOException("Expected actuator frame but got " + received.frameType());
        }
        ActuatorFrame actuatorFrame = FrameBinaryCodec.decodeActuatorFrame(received.payload());
        EngineStepResult result = engine.step(actuatorFrame);
        for (SensorFrame sensorFrame : result.sensorFrames()) {
          transport.send(FrameBinaryCodec.encodeSensorFrame(sensorFrame));
        }
        WorldSnapshot snapshot = result.worldSnapshot();
        transport.send(FrameBinaryCodec.encodeWorldSnapshot(snapshot));
      }
    } catch (EOFException eof) {
      // Normal client disconnect after the control host completes its run.
    } catch (IOException ex) {
      if (!serverSocket.isClosed()) {
        throw ex;
      }
    }
  }

  @Override
  public void close() throws IOException {
    serverSocket.close();
  }
}
