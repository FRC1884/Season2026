package org.Griffins1884.frc2026.simulation.runtime;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;
import org.Griffins1884.frc2026.simulation.contracts.FrameBinaryCodec;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.Griffins1884.frc2026.simulation.contracts.SimulationProtocolVersion;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;
import org.Griffins1884.frc2026.simulation.replay.DeterministicReplayTrace;

/** Reads deterministic replay logs written by {@link ReplayLogWriter}. */
public final class ReplayLogReader {
  private static final int MAGIC = 0x4753494d; // GSIM

  public DeterministicReplayTrace read(InputStream inputStream) throws IOException {
    DataInputStream in = new DataInputStream(inputStream);
    int magic = in.readInt();
    if (magic != MAGIC) {
      throw new IllegalArgumentException("Unexpected replay magic: " + magic);
    }
    short version = in.readShort();
    if (version != SimulationProtocolVersion.VERSION) {
      throw new IllegalArgumentException("Unexpected replay version: " + version);
    }

    List<ActuatorFrame> actuatorFrames = new ArrayList<>();
    List<SensorFrame> sensorFrames = new ArrayList<>();
    List<WorldSnapshot> worldSnapshots = new ArrayList<>();

    while (in.available() > 0) {
      ReplayRecordType type = ReplayRecordType.fromWireValue(in.readInt());
      int length = in.readInt();
      byte[] payload = in.readNBytes(length);
      if (payload.length != length) {
        throw new IOException("Unexpected replay EOF");
      }
      switch (type) {
        case ACTUATOR -> actuatorFrames.add(FrameBinaryCodec.decodeActuatorFrame(payload));
        case SENSOR -> sensorFrames.add(FrameBinaryCodec.decodeSensorFrame(payload));
        case WORLD -> worldSnapshots.add(FrameBinaryCodec.decodeWorldSnapshot(payload));
      }
    }

    return new DeterministicReplayTrace(actuatorFrames, sensorFrames, worldSnapshots);
  }
}
