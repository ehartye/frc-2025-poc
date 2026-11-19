package org.reefscape.controls;

import java.util.function.LongSupplier;

/** Test-source-only output adapter for an explicitly selected release profile. */
final class DesktopReleaseIO implements ReceiverIO {
    enum Profile { HISTORICAL, RAMP_090 }
    private final ReceiverIO downstream;
    private final LongSupplier clock;
    private final Profile profile;
    private long risingMs = -1;
    Demand raw = Demand.neutral();

    DesktopReleaseIO(ReceiverIO downstream, LongSupplier clock, Profile profile) {
        this.downstream = downstream;
        this.clock = clock;
        this.profile = profile;
    }

    public void beginAttempt(long id) {
        risingMs = -1;
        downstream.beginAttempt(id);
    }

    public Inputs read() { return downstream.read(); }

    public void write(Demand value) {
        raw = value;
        if (!value.active() || value.beltMps() <= 0) risingMs = -1;
        if (profile == Profile.RAMP_090 && value.active() && value.beltMps() > 0) {
            long now = clock.getAsLong();
            if (risingMs < 0) risingMs = now;
            double speed = Math.min(value.beltMps(), 0.90 * Math.min(1.0, (now - risingMs) / 40.0));
            value = new Demand(true, value.liftMm(), value.slideMm(), speed,
                               value.drivePermit(), value.slideVelocityMmPerS());
        }
        downstream.write(value);
    }
}
