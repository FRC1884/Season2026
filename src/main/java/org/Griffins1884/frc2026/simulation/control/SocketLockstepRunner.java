package org.Griffins1884.frc2026.simulation.control;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;
import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;
import org.Griffins1884.frc2026.simulation.contracts.FrameBinaryCodec;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.Griffins1884.frc2026.simulation.contracts.SimulationProtocolVersion;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;
import org.Griffins1884.frc2026.simulation.replay.DeterministicReplayTrace;
import org.Griffins1884.frc2026.simulation.replay.ShotReplayScenario;
import org.Griffins1884.frc2026.simulation.runtime.ReplayLogWriter;
import org.Griffins1884.frc2026.simulation.transport.SocketFrameTransport;

/** Runs a lockstep scenario against the socket-backed deterministic engine. */
public final class SocketLockstepRunner {
  public Result run(String host, int port, ShotReplayScenario scenario) throws IOException {
    LockstepScenarioControlHost controlHost = new LockstepScenarioControlHost(scenario);
    List<ActuatorFrame> actuatorFrames = new ArrayList<>(scenario.totalSteps());
    List<SensorFrame> sensorFrames = new ArrayList<>();
    List<WorldSnapshot> worldSnapshots = new ArrayList<>(scenario.totalSteps());
    ByteArrayOutputStream replayOutput = new ByteArrayOutputStream();

    try (Socket socket = new Socket(host, port);
        SocketFrameTransport transport =
            new SocketFrameTransport(socket.getInputStream(), socket.getOutputStream());
        ReplayLogWriter replayLogWriter = new ReplayLogWriter(replayOutput)) {
      for (int step = 0; step < scenario.totalSteps(); step++) {
        long simTimeNanos = Math.round(step * scenario.stepSeconds() * 1_000_000_000.0);
        ActuatorFrame actuatorFrame = controlHost.actuatorFrameForStep(step, simTimeNanos);
        actuatorFrames.add(actuatorFrame);
        replayLogWriter.appendActuatorFrame(actuatorFrame);
        transport.send(FrameBinaryCodec.encodeActuatorFrame(actuatorFrame));

        boolean receivedWorld = false;
        while (!receivedWorld) {
          SocketFrameTransport.ReceivedFrame received = transport.receive();
          if (received.frameType() == SimulationProtocolVersion.FRAME_TYPE_SENSOR) {
            SensorFrame frame = FrameBinaryCodec.decodeSensorFrame(received.payload());
            sensorFrames.add(frame);
            replayLogWriter.appendSensorFrame(frame);
          } else if (received.frameType() == SimulationProtocolVersion.FRAME_TYPE_WORLD_SNAPSHOT) {
            WorldSnapshot snapshot = FrameBinaryCodec.decodeWorldSnapshot(received.payload());
            worldSnapshots.add(snapshot);
            replayLogWriter.appendWorldSnapshot(snapshot);
            receivedWorld = true;
          } else {
            throw new IOException("Unexpected frame type " + received.frameType());
          }
        }
      }
    }

    return new Result(
        new DeterministicReplayTrace(actuatorFrames, sensorFrames, worldSnapshots),
        replayOutput.toByteArray());
  }

  public record Result(DeterministicReplayTrace trace, byte[] replayBytes) {}
}
