package org.Griffins1884.frc2026.simulation.runtime;

import java.io.Closeable;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import org.Griffins1884.frc2026.simulation.contracts.ActuatorFrame;
import org.Griffins1884.frc2026.simulation.contracts.FrameBinaryCodec;
import org.Griffins1884.frc2026.simulation.contracts.SensorFrame;
import org.Griffins1884.frc2026.simulation.contracts.SimulationProtocolVersion;
import org.Griffins1884.frc2026.simulation.contracts.WorldSnapshot;

/** Binary replay writer for deterministic simulation traces. */
public final class ReplayLogWriter implements Closeable {
  private static final int MAGIC = 0x4753494d; // GSIM

  private final DataOutputStream out;

  public ReplayLogWriter(OutputStream outputStream) {
    out = new DataOutputStream(outputStream);
    try {
      out.writeInt(MAGIC);
      out.writeShort(SimulationProtocolVersion.VERSION);
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  public void appendActuatorFrame(ActuatorFrame frame) {
    writeRecord(ReplayRecordType.ACTUATOR, FrameBinaryCodec.encodeActuatorFrame(frame));
  }

  public void appendSensorFrame(SensorFrame frame) {
    writeRecord(ReplayRecordType.SENSOR, FrameBinaryCodec.encodeSensorFrame(frame));
  }

  public void appendWorldSnapshot(WorldSnapshot snapshot) {
    writeRecord(ReplayRecordType.WORLD, FrameBinaryCodec.encodeWorldSnapshot(snapshot));
  }

  private void writeRecord(ReplayRecordType type, byte[] bytes) {
    try {
      out.writeInt(type.wireValue());
      out.writeInt(bytes.length);
      out.write(bytes);
    } catch (IOException ex) {
      throw new UncheckedIOException(ex);
    }
  }

  @Override
  public void close() throws IOException {
    out.close();
  }
}
