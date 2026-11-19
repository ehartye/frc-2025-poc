package org.reefscape.controls;

/** Desktop experiment. Both inputs are digital and the timestamps share the controller clock. */
public final class PortAcquisition {
    public enum State { WAKE, WAIT_ENTRY, BRAKE, PROBE, DWELL, IN_BAND, FAULT }
    public record Output(State state, boolean active, double beltMps, String reason) {}
    private final boolean counterfeed;
    public PortAcquisition(boolean counterfeed) { this.counterfeed = counterfeed; }
    private State state = State.WAKE;
    private long entered, previous = -1;
    private boolean incomingSeatOnly, incomingClear, probeSeat;
    private String reason = "";

    private void move(State next, long now) { state = next; entered = now; }
    private void fail(String cause, long now) { reason = cause; move(State.FAULT, now); }

    public Output update(long now, long sampleAt, boolean valid, boolean calibrated, boolean docked,
                         boolean enabled, boolean entry, boolean seat) {
        if (state == State.FAULT) return output();
        if (!enabled || !docked || !valid || !calibrated || now != sampleAt ||
                (previous >= 0 && now - previous != 20)) {
            fail("unqualified-or-stale-frame", now);
            return output();
        }
        previous = now;
        switch (state) {
            case WAKE -> {
                if (entry || seat) fail("occupied-at-wake", now);
                else move(State.WAIT_ENTRY, now);
            }
            case WAIT_ENTRY -> {
                if (entry) move(State.BRAKE, now);
                else if (seat) fail("seat-before-observed-entry", now);
                else if (now - entered >= 1200) fail("entry-timeout", now);
            }
            case BRAKE -> {
                incomingSeatOnly |= seat && !entry;
                incomingClear |= incomingSeatOnly && !entry && !seat;
                if (now - entered >= 260 && incomingClear && !entry && !seat) move(State.PROBE, now);
                else if (now - entered >= 700) fail("incoming-edges-unresolved", now);
            }
            case PROBE -> {
                if (entry && (!seat || !probeSeat)) fail("ambiguous-probe-edges", now);
                else if (entry && seat) move(State.DWELL, now);
                else if (probeSeat && !seat) fail("lost-probe-seat", now);
                else if (now - entered >= 1600) fail("probe-timeout", now);
                probeSeat |= seat && !entry;
            }
            case DWELL, IN_BAND -> {
                if (!seat || !entry) fail("lost-dual-return", now);
                else if (state == State.DWELL && now - entered >= 240) move(State.IN_BAND, now);
            }
            default -> throw new IllegalStateException();
        }
        return output();
    }

    private Output output() {
        double speed = 0;
        if (state == State.PROBE) speed = 0.16;
        if (counterfeed && (state == State.WAIT_ENTRY || (state == State.BRAKE && previous - entered < 160))) {
            speed = 0.45;
        }
        return new Output(state, state != State.FAULT, speed, reason);
    }
}
