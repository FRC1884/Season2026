package org.Griffins1884.frc2026.simulation.sensors;

import java.util.ArrayList;
import java.util.List;

/** Deterministic latency queue keyed by absolute availability time. */
public final class LatencyQueue<T> {
  private final List<Entry<T>> entries = new ArrayList<>();

  public void enqueue(long readyAtNanos, T value) {
    entries.add(new Entry<>(readyAtNanos, value));
  }

  public List<T> releaseReady(long nowNanos) {
    List<T> ready = new ArrayList<>();
    for (int i = 0; i < entries.size(); ) {
      Entry<T> entry = entries.get(i);
      if (entry.readyAtNanos <= nowNanos) {
        ready.add(entry.value);
        entries.remove(i);
      } else {
        i++;
      }
    }
    return ready;
  }

  private record Entry<T>(long readyAtNanos, T value) {}
}
