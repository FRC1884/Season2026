package org.Griffins1884.frc2026.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public final class IndexerConstants {
  public static final CANBus CAN_BUS = new CANBus("rio");

  public static final int[] INDEXER_IDS = {0}; // TODO: set indexer CAN IDs
  public static final boolean[] INDEXER_INVERTED = {false}; // TODO: set per-motor inversion

  public static final int CURRENT_LIMIT_AMPS = 40; // TODO: tune
  public static final boolean BRAKE_MODE = true;
  public static final double REDUCTION = 1.0;
  public static final double MAX_VOLTAGE = 12.0;
  public static final GlobalConstants.Gains gains =
      new GlobalConstants.Gains("Indexer/Gains", 1.0, 0.0, 0.0);
  public static final double VELOCITY_TOLERANCE = 0.0;
  public static final LoggedTunableNumber FORWARD_RPM =
      new LoggedTunableNumber("Indexer/ForwardRpm", 2000.0);
  public static final LoggedTunableNumber REVERSE_RPM =
      new LoggedTunableNumber("Indexer/ReverseRpm", -2000.0);

  private IndexerConstants() {}
}
