package org.reefscape.controls;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

/** A desktop two-idler motion veto. Its success authorizes only continued probing. */
public final class IdlerWatch {
    public record Reading(long exposureUs, long receivedUs, int adcCode, boolean busOk) {}
    public record Decision(boolean allow, boolean motionSeen, String reason) {}
    private record Point(long us, double mm) {}

    public static final int PERIOD_CODES = 1650;
    public static final double TRAVEL_MM = 60.0;
    public static final double THRESHOLD_MM = 1.50;
    public static final long ARM_GRACE_US = 60_000;
    private static final long MAX_AGE_US = 10_000;
    private static final long HISTORY_US = 55_000;
    private static final long PAIR_US = 40_000;
    private static final class Encoder {
        final Deque<Point> history = new ArrayDeque<>();
        int previous;
        double cumulative;
        long stamp = Long.MIN_VALUE, received = Long.MIN_VALUE;
        boolean invalid;

        void sample(Reading input, int direction) {
            long t = input.exposureUs();
            if (!input.busOk() || input.adcCode()<0 || input.adcCode()>PERIOD_CODES ||
                input.receivedUs()<t || input.receivedUs()-t>4_000 ||
                (stamp!=Long.MIN_VALUE && (t<=stamp || t-stamp>5_250))) {
                invalid=true;
                return;
            }
            if (stamp!=Long.MIN_VALUE) {
                int delta = input.adcCode()-previous;
                if (delta > PERIOD_CODES/2) delta-=PERIOD_CODES;
                if (delta < -PERIOD_CODES/2) delta+=PERIOD_CODES;
                double mm=direction*TRAVEL_MM*delta/PERIOD_CODES;
                // Reject steps beyond the project release-speed ceiling plus fitted endpoint error.
                double limit=1_050.0*(t-stamp)/1_000_000.0+1.06;
                if (Math.abs(mm)>limit) invalid=true;
                cumulative+=mm;
            }
            stamp=t; received=input.receivedUs();
            previous=input.adcCode();
            history.addLast(new Point(t,cumulative));
            while (history.getFirst().us()<t-HISTORY_US) history.removeFirst();
        }

        boolean fresh(long now) {
            return !invalid && history.size()>=3 && stamp<=now && received<=now && now-stamp<=MAX_AGE_US;
        }

        boolean forward(long start) {
            List<Point> points=List.copyOf(history);
            // Both latest samples must show actual signed travel over approximately 40 ms.
            for (int i=points.size()-2;i<points.size();i++) {
                Point end=points.get(i), base=null;
                for (Point p:points) if (p.us()<=end.us()-PAIR_US && p.us()>=start-6_250) base=p;
                if (base==null || end.us()-base.us()>45_250 || end.mm()-base.mm()<=THRESHOLD_MM) return false;
            }
            return true;
        }
    }

    private final Encoder left=new Encoder(), right=new Encoder();
    private long start=Long.MIN_VALUE;
    private boolean motionSeen, failed;
    private String reason="";
    public void sample(boolean leftSide, Reading r) {
        (leftSide?left:right).sample(r,leftSide?1:-1);
    }
    private Decision fail(String why) { failed=true;reason=why;return new Decision(false,false,why); }

    public Decision evaluate(long now, boolean calibration, boolean probing, boolean completed) {
        if (failed) return new Decision(false,false,reason);
        if (!probing && !completed) return new Decision(true,motionSeen,"");
        if (!calibration || !left.fresh(now) || !right.fresh(now)) return fail("uncalibrated-stale-or-invalid-idler");
        if (start==Long.MIN_VALUE) {
            if (!probing) return fail("missing-probe");
            start=now;
        }
        if (completed) return motionSeen?new Decision(true,true,""):fail("dual-without-proven-drive");
        if (now-start>=ARM_GRACE_US) {
            if (!left.forward(start)||!right.forward(start))return fail("independent-idler-travel-missing");
            motionSeen=true;
        }
        return new Decision(true,motionSeen,"");
    }
}

