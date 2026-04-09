package org.Griffins1884.frc2026.simulation.transport;

import java.io.Closeable;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import org.Griffins1884.frc2026.simulation.contracts.FrameBinaryCodec;

/** Length-prefixed transport for deterministic frame payloads. */
public final class SocketFrameTransport implements Closeable {
  private final DataInputStream in;
  private final DataOutputStream out;

  public SocketFrameTransport(InputStream inputStream, OutputStream outputStream) {
    in = new DataInputStream(inputStream);
    out = new DataOutputStream(outputStream);
  }

  public synchronized void send(byte[] encodedFrame) throws IOException {
    out.writeInt(encodedFrame.length);
    out.write(encodedFrame);
    out.flush();
  }

  public synchronized ReceivedFrame receive() throws IOException {
    int length = in.readInt();
    byte[] payload = in.readNBytes(length);
    if (payload.length != length) {
      throw new IOException("Unexpected EOF while reading frame payload");
    }
    return new ReceivedFrame(FrameBinaryCodec.peekFrameType(payload), payload);
  }

  @Override
  public void close() throws IOException {
    try {
      in.close();
    } finally {
      out.close();
    }
  }

  public record ReceivedFrame(byte frameType, byte[] payload) {}
}
