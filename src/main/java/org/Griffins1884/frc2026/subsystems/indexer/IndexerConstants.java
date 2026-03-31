package org.Griffins1884.frc2026.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import org.Griffins1884.frc2026.GlobalConstants;
import org.Griffins1884.frc2026.mechanisms.MechanismDefinition;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

public final class IndexerConstants {
  public static final CANBus CAN_BUS = new CANBus("rio");

  public static final int[] INDEXER_IDS = {18};
  public static final boolean[] INDEXER_INVERTED = {false};

  public static final int CURRENT_LIMIT_AMPS = 40;
  public static final MechanismDefinition.KrakenFeatureConfig KRAKEN_FEATURES =
      new MechanismDefinition.KrakenFeatureConfig(true, true, false, 100, true);
  public static final boolean BRAKE_MODE = false;
  public static final double REDUCTION = 1.0;
  public static final double MAX_VOLTAGE = 12.0;
  public static final GlobalConstants.Gains GAINS =
      new GlobalConstants.Gains("Indexer/Gains", 100, 0.02, 1.0, 0.37952, 0.017, 0.025075);
  public static final double VELOCITY_TOLERANCE = 50.0;
  public static final LoggedTunableNumber FORWARD_RPM =
      new LoggedTunableNumber("Indexer/ForwardRpm", 6000.0);
  public static final LoggedTunableNumber REVERSE_RPM =
      new LoggedTunableNumber("Indexer/ReverseRpm", -6000.0);

  private IndexerConstants() {}
}
