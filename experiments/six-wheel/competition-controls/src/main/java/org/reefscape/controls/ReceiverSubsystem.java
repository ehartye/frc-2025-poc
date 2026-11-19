package org.reefscape.controls;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.LongSupplier;
import org.reefscape.controls.ReceiverIO.*;

/** One WPILib owner of the two axis goals, paired belt surface request and drive permission. */
public final class ReceiverSubsystem extends SubsystemBase {
    public static final double STATION_MM = 940, CARRY_MM = 670, PLACE_MM = 560;
    public static final double RECEIVE_MM = 240, DEPLOY_MM = 370;
    static final double TOLERANCE_MM = 2;
    private final ReceiverIO io;
    private final LongSupplier clockMs;
    private String fault = "";
    private long nextAttempt;

    public ReceiverSubsystem(ReceiverIO io, LongSupplier clockMs) {
        this.io = io;
        this.clockMs = clockMs;
    }

    long beginAttempt() { io.beginAttempt(++nextAttempt); return nextAttempt; }
    long now() { return clockMs.getAsLong(); }
    Inputs read() { return io.read(); }
    void output(Demand demand) { io.write(demand); }
    void neutral() { output(Demand.neutral()); }
    public String fault() { return fault; }

    void trip(String reason) {
        if (fault.isEmpty()) fault = reason;
        neutral();
    }

    static boolean near(double actual, double target) { return Math.abs(actual - target) <= TOLERANCE_MM; }

    static boolean fresh(long stamp, long now) { return stamp >= 0 && stamp <= now && now - stamp <= 60; }

    static String invalid(Inputs input, long now) {
        if (input == null || input.mechanism() == null || input.chassis() == null) return "missing inputs";
        Sensors s = input.mechanism();
        Approach a = input.chassis();
        if (!s.valid() || !fresh(s.timeMs(),now)) return "invalid or stale mechanism";
        if (!a.valid() || !fresh(a.timeMs(),now)) return "invalid or stale chassis";
        if (!Double.isFinite(s.liftMm()) || !Double.isFinite(s.slideMm())
            || s.slideMm() < -TOLERANCE_MM || s.slideMm() > DEPLOY_MM + TOLERANCE_MM
            || s.liftMm() < PLACE_MM - TOLERANCE_MM || s.liftMm() > STATION_MM + TOLERANCE_MM) {
            return "encoder outside candidate travel";
        }
        if ((s.liftUpper() && s.liftLower()) || (s.slideExtended() && s.slideRetracted())
            || (s.slideRetracted() && !near(s.slideMm(),0))
            || (s.slideExtended() && !near(s.slideMm(),DEPLOY_MM))) return "conflicting limits";
        if (s.jam()) return "jam indicated";
        return "";
    }

    /** Explicit pit/operator acknowledgment after an externally resolved fault. Writes no active goal. */
    public boolean acknowledgeRecovery() {
        if (getCurrentCommand() != null || fault.isEmpty()) return false;
        Inputs input = read();
        if (!invalid(input,now()).isEmpty()) return false;
        Sensors s = input.mechanism();
        if (!input.chassis().stopped() || s.clearOfShelf() || s.seat() || s.entry() || !s.slideRetracted()
            || !near(s.slideMm(),0) || !near(s.liftMm(),STATION_MM) || input.requestRelease()) return false;
        fault = "";
        neutral();
        return true;
    }
}
