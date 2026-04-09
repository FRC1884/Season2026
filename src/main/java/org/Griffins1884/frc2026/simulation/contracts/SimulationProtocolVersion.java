package org.Griffins1884.frc2026.simulation.contracts;

/** Versioning constants for deterministic simulation frame encoding. */
public final class SimulationProtocolVersion {
  public static final short VERSION = 1;
  public static final String SCHEMA_ID = "griffinsim-io-v1";

  public static final byte FRAME_TYPE_ACTUATOR = 1;
  public static final byte FRAME_TYPE_SENSOR = 2;
  public static final byte FRAME_TYPE_WORLD_SNAPSHOT = 3;

  private SimulationProtocolVersion() {}
}
