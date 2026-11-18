package org.reefscape.controls;

/** Atomic application-time observations. Each timestamp refers to the oldest channel in its group. */
public interface ReceiverIO {
    record Sensors(long timeMs, double liftMm, double slideMm, boolean liftLower,
                   boolean liftUpper, boolean slideRetracted, boolean slideExtended,
                   boolean entry, boolean seat, boolean clearOfShelf, long exitAttempt, boolean jam, boolean valid) {}

    record Approach(long timeMs, boolean stationDocked, boolean targetDocked,
                    boolean targetClear, boolean stopped, boolean valid) {}

    record Inputs(Sensors mechanism, Approach chassis, boolean requestRelease) {}

    /** Both belt contacts use positive surface speed toward the robot front.
     * An active zero belt speed requests stationary belts. Neutral disables all setpoints. */
    record Demand(boolean active, double liftMm, double slideMm, double beltMps,
                  boolean drivePermit) {
        public static Demand neutral() { return new Demand(false, 0, 0, 0, false); }
    }

    /** Assign a new attempt to the external piece/exit observer. No motor action. */
    void beginAttempt(long attempt);
    Inputs read();
    void write(Demand demand);
}
