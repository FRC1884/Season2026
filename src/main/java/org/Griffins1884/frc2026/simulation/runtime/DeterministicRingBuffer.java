package org.Griffins1884.frc2026.simulation.runtime;

import java.util.ArrayDeque;
import java.util.Optional;

/** Small bounded FIFO used to enforce deterministic buffering without unbounded growth. */
public final class DeterministicRingBuffer<T> {
  private final int capacity;
  private final ArrayDeque<T> queue;

  public DeterministicRingBuffer(int capacity) {
    if (capacity <= 0) {
      throw new IllegalArgumentException("capacity must be positive");
    }
    this.capacity = capacity;
    queue = new ArrayDeque<>(capacity);
  }

  public synchronized boolean offer(T item) {
    if (queue.size() >= capacity) {
      return false;
    }
    queue.addLast(item);
    return true;
  }

  public synchronized Optional<T> poll() {
    return Optional.ofNullable(queue.pollFirst());
  }

  public synchronized int size() {
    return queue.size();
  }
}
