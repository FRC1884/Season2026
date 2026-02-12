package org.Griffins1884.frc2026.subsystems.indexer;

import org.Griffins1884.frc2026.generic.rollers.GenericRollerSystemIOKraken;

public class IndexerIOMax extends GenericRollerSystemIOKraken implements IndexerIO {
  public IndexerIOMax() {
    super(
        IndexerConstants.INDEXER_IDS,
        IndexerConstants.CURRENT_LIMIT_AMPS,
        IndexerConstants.INDEXER_INVERTED,
        IndexerConstants.BRAKE_MODE,
        IndexerConstants.REDUCTION,
        IndexerConstants.CAN_BUS);
  }
}
