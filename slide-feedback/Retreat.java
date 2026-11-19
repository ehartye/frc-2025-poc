package org.reefscape.controls;

/** Desktop-only controller. Positive travel withdraws the shelf toward the chassis. */
public final class Retreat {
    public enum Policy {
        STOCK("stock20",20,60,150,.200), SERVO("servo5",5,45,150,.200),
        SHORT("short5",5,40,150,.070), REGISTERED("extended5",5,40,180,.070);
        final String label;
        final int period, ack, deadline;
        final double ramp;
        Policy(String label, int period, int ack, int deadline, double ramp) {
            this.label=label; this.period=period; this.ack=ack; this.deadline=deadline; this.ramp=ramp;
        }
    }
    public enum State { WAIT, MOVING, STOWED, FAULT }
    static final String REGISTER = "c1242109e239d880d1562c3e7b4bbcf857d154d9a31e3403e35e182864d00ebe";
    public record Clearance(String register, String attempt, long priorMs, long capturedMs, long receivedMs,
                            boolean sameTrack, boolean frontProven) {}
    public record Feedback(long sampledMs, double potM, double motorM, double speedMps,
                           boolean stowedSwitch, double busV, boolean enabled) {}

    public static final double TRAVEL = .370, DURATION = .370 / .150;
    static final double RATIO = (68.0 / 13) * (76.0 / 21);
    static final double TO_MOTOR = RATIO / .020;
    static final double KT = .97 / (100 - 1.4), KE = (12 - .12 * 1.4) / (11000 * 2 * Math.PI / 60);
    static final double FORCE_PER_AMP = TO_MOTOR * .70 * KT;
    static final double MASS = 5.3571 + .440 + 1.3928 / 4 + 3e-6 * TO_MOTOR * TO_MOTOR * .70;
    private final String attempt;
    private final int period, onsetBound, deadline;
    private final double ramp;
    public State state = State.WAIT;
    public long accepted = -1, observed = -1, finished = -1;
    private long previous = -1;
    private double integratedError, initialPot, initialMotor;
    private Clearance witness;

    public Retreat(String attempt, Policy policy) {
        this.attempt = attempt; period = policy.period; onsetBound = policy.ack; deadline = policy.deadline; ramp = policy.ramp;
    }

    private boolean registered(Clearance frame) {
        boolean a = attempt.equals("sector-minus180/coral-A") && frame.priorMs() == 26593
                && frame.capturedMs() >= 26695 && frame.capturedMs() <= 26711;
        boolean b = attempt.equals("sector-plus220/coral-B") && frame.priorMs() == 26301
                && frame.capturedMs() >= 26402 && frame.capturedMs() <= 26418;
        return (a || b) && frame.receivedMs() == frame.capturedMs()+15;
    }

    private double fault() { state = State.FAULT; return 0; }
    public double update(long now, Clearance packet, Feedback sense) {
        if (state == State.FAULT || state == State.STOWED) return 0;
        if (!sense.enabled() || !Double.isFinite(sense.potM()) || !Double.isFinite(sense.motorM())
                || !Double.isFinite(sense.speedMps()) || !Double.isFinite(sense.busV()) || sense.busV() < 9.5
                || now - sense.sampledMs() < 0 || now - sense.sampledMs() > period
                || Math.abs(sense.potM() - sense.motorM()) > .002 || sense.potM() < -.001
                || sense.potM() > TRAVEL + .001 || (sense.stowedSwitch() && sense.potM() < TRAVEL - .001)
                || (previous >= 0 && now - previous != period)) return fault();
        previous = now;
        if (state == State.WAIT) {
            if (packet == null || now < packet.receivedMs()) return 0;
            if (!registered(packet) || now-packet.receivedMs() >= period || !REGISTER.equals(packet.register()) || !attempt.equals(packet.attempt()) || !packet.sameTrack() || !packet.frontProven()
                    || packet.priorMs() > packet.capturedMs() || packet.receivedMs() < packet.capturedMs()
                    || now - packet.capturedMs() < 7 || now - packet.capturedMs() > 29
                    || now - packet.priorMs() + onsetBound > deadline || Math.abs(sense.potM()) > .001) return fault();
            initialPot = sense.potM(); initialMotor = sense.motorM();
            accepted = now; witness = packet; state = State.MOVING;
        }
        if (packet != null && (!REGISTER.equals(packet.register()) || !packet.frontProven() || !packet.sameTrack() || !attempt.equals(packet.attempt()))) return fault();
        if (observed < 0 && sense.potM() - initialPot >= 4*(.635/4096) && sense.motorM() - initialMotor >= .00020 && sense.speedMps() > .002) {
            if (now - witness.priorMs() > deadline || now - accepted > onsetBound) return fault();
            observed = now;
        }
        if (observed < 0 && now - accepted >= onsetBound) return fault();
        double deceleration = ramp < .1 ? .300 : ramp;
        double t = (now - accepted) / 1000.0;
        double vmax = TRAVEL / (DURATION - (ramp+deceleration)/2), a = vmax / ramp;
        double x, v, acc;
        if (t < ramp) { x = .5*a*t*t; v = a*t; acc = a; }
        else if (t < DURATION - deceleration) { x = vmax*(t-ramp*.5); v = vmax; acc = 0; }
        else if (t < DURATION) { double remain = DURATION-t; double decel = vmax/deceleration; x = TRAVEL-.5*decel*remain*remain; v = decel*remain; acc = -decel; }
        else { x = TRAVEL; v = 0; acc = 0; }
        double error = x - sense.potM();
        if (Math.abs(error) > .015 || t > DURATION + .15) return fault();
        if (t >= DURATION && sense.stowedSwitch() && sense.potM() >= TRAVEL-.0005
                && sense.motorM() >= TRAVEL-.0005 && Math.abs(sense.speedMps()) < .006) {
            finished = now; state = State.STOWED; return 0;
        }
        integratedError = Math.max(-.012, Math.min(.012, integratedError + error*period*.001));
        double effort = MASS*(acc+850*error+10*(v-sense.speedMps())+5500*integratedError);
        effort += 27;
        double amps = effort/FORCE_PER_AMP + Math.copySign(1.4,effort);
        amps = Math.max(-20, Math.min(20,amps));
        double volts = amps*.12 + KE*TO_MOTOR*v;
        return Math.max(-sense.busV(), Math.min(sense.busV(),volts));
    }

    public void cancel() { if (state != State.STOWED) fault(); }
}
