package org.Griffins1884.frc2026.simulation.runtime;

/** Record types stored in deterministic replay logs. */
public enum ReplayRecordType {
  ACTUATOR(1),
  SENSOR(2),
  WORLD(3);

  private final int wireValue;

  ReplayRecordType(int wireValue) {
    this.wireValue = wireValue;
  }

  public int wireValue() {
    return wireValue;
  }

  public static ReplayRecordType fromWireValue(int wireValue) {
    for (ReplayRecordType value : values()) {
      if (value.wireValue == wireValue) {
        return value;
      }
    }
    throw new IllegalArgumentException("Unsupported replay record type: " + wireValue);
  }
}
