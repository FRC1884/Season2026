package org.Griffins1884.frc2026.subsystems.indexer;

public final class IndexerConstants {
  public static final int LEADER_ID = 0; // TODO: set indexer leader CAN ID
  public static final int FOLLOWER_ID = -1; // TODO: set follower ID if used
  public static final boolean INVERTED = false; // TODO: set inversion
  public static final boolean FOLLOWER_INVERTED = false; // TODO: set follower inversion
  public static final int CURRENT_LIMIT_AMPS = 40; // TODO: tune
  public static final boolean BRAKE_MODE = true;
  public static final double REDUCTION = 1.0;
  public static final boolean IS_FLEX = true;
  public static final double MAX_VOLTAGE = 12.0;

  private IndexerConstants() {}
}
