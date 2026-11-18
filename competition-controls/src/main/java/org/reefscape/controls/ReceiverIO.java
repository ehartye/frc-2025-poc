package org.reefscape.controls;

/** Atomic application-time observations. Each timestamp refers to the oldest channel in its group. */
public interface ReceiverIO {
    /** Independent whole-front proof. Time is the synchronized exposure, not packet arrival. */
    record WholeFront(String assemblySha256, String studySha256, String frame, String clock,
                      String pieceId, int sectorMm, int exposurePhase, long captureMs, double lowerGapMm,
                      double coastFriction,double pixelError,String fieldSha256) {
        /** Older measurements omit the current field configuration and uncertainty tier. */
        public WholeFront(String assembly,String study,String frame,String clock,String piece,int sector,
                          int phase,long capture,double gap) {
            this(assembly,study,frame,clock,piece,sector,phase,capture,gap,Double.NaN,Double.NaN,"");
        }
    }
    /** The earliest bound comes from independent prior contact. Both endpoints use controller time. */
    record Exit(boolean clear, long attempt, String pieceKey, long earliestCrossingMs,
                long latestCrossingMs, boolean calibrated, WholeFront wholeFront) {
        /** A legacy clearance remains shelf-only even when calibrated. */
        public Exit(boolean clear,long attempt,String pieceKey,long earliest,long latest,boolean calibrated) {
            this(clear,attempt,pieceKey,earliest,latest,calibrated,null);
        }
    }
    static String pieceKey(long attempt) { return "station-attempt-" + attempt; }

    record Sensors(long timeMs, double liftMm, double slideMm, boolean liftLower,
                   boolean liftUpper, boolean slideRetracted, boolean slideExtended,
                   boolean entry, boolean seat, Exit exit, boolean jam, boolean valid) {
        boolean clearOfShelf() { return exit != null && exit.clear(); }
    }

    record Approach(long timeMs, boolean stationDocked, boolean targetDocked,
                    boolean targetClear, boolean stopped, boolean valid, int targetSectorMm) {
        public Approach(long time,boolean station,boolean target,boolean clear,boolean stopped,boolean valid) {
            this(time,station,target,clear,stopped,valid,Integer.MIN_VALUE);
        }
    }

    record Inputs(Sensors mechanism, Approach chassis, boolean requestRelease, String trackedPieceId,long trackOriginMs) {
        public Inputs(Sensors mechanism,Approach chassis,boolean requestRelease) {
            this(mechanism,chassis,requestRelease,"",-1);
        }
    }

    /** Both belt contacts use positive surface speed toward the robot front.
     * An active zero belt speed requests stationary belts. Neutral disables all setpoints. */
    record Demand(boolean active, double liftMm, double slideMm, double beltMps,
                  boolean drivePermit,double slideVelocityMmPerS) {
        public Demand(boolean active,double lift,double slide,double belt,boolean drivePermit) {
            this(active,lift,slide,belt,drivePermit,0);
        }
        public static Demand neutral() { return new Demand(false, 0, 0, 0, false); }
    }

    /** Assign a new attempt to the external piece/exit observer. No motor action. */
    void beginAttempt(long attempt);
    Inputs read();
    void write(Demand demand);
}
