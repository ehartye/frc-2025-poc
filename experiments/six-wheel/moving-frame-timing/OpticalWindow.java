package org.reefscape.controls;

import java.util.*;

/** Test-only camera packet supervisor. The caller supplies one monotonic controller epoch and both registered views. */
final class OpticalWindow {
    enum Eye { MOVING, FIXED }
    record Frame(Eye eye,String piece,String epoch,String registration,int sequence,long capture,long arrived,
                 boolean seated,double[] bounds) {}
    record Context(String piece,String epoch,String movingRegistration,String fixedRegistration,
                   long release,long priorLower,long firstMoving,long firstFixed,int onsetReserve,int deadline) {}
    record Decision(boolean accepted,long capture,long application,String reason) {}

    private final Context context;
    private final EnumMap<Eye,Frame> previous=new EnumMap<>(Eye.class);
    private final Map<Long,Frame> roof=new HashMap<>();
    private final ArrayDeque<Frame> waiting=new ArrayDeque<>();
    private int aligned;
    private boolean linked;
    private Frame lastPaired,clear;
    private Decision decision;
    private long lastApplication=-1;

    OpticalWindow(Context context) { this.context=context; }

    private void stop(long now,String reason) { decision=new Decision(false,-1,now,reason); }
    private boolean finite(Frame f) {
        if (f.bounds()==null || f.bounds().length!=6) return false;
        for (int i=0;i<6;i+=2) {
            if (!Double.isFinite(f.bounds()[i]) || !Double.isFinite(f.bounds()[i+1])
                || f.bounds()[i]>f.bounds()[i+1]) return false;
        }
        return true;
    }
    private boolean close(Frame a, Frame b,double tolerance) {
        if (!finite(a) || !finite(b)) return false;
        for (int i=0;i<6;i+=2) {
            if (a.bounds()[i]>b.bounds()[i+1]+tolerance || b.bounds()[i]>a.bounds()[i+1]+tolerance) return false;
        }
        return true;
    }
    private void consume(long now,Frame f) {
        if (linked && f.eye()==Eye.MOVING) return;
        String expected=f.eye()==Eye.MOVING ? context.movingRegistration() : context.fixedRegistration();
        if (!context.piece().equals(f.piece()) || !context.epoch().equals(f.epoch()) || !expected.equals(f.registration())) {
            stop(now,"wrong-identity-or-registration"); return;
        }
        if (f.capture()>f.arrived() || f.arrived()>now || now-f.capture()>40) {
            stop(now,"stale-or-impossible-timestamp"); return;
        }
        Frame before=previous.get(f.eye());
        long first=f.eye()==Eye.MOVING ? context.firstMoving() : context.firstFixed();
        if ((before==null && (f.sequence()!=0 || f.capture()!=first)) ||
            (before!=null && (f.sequence()!=before.sequence()+1 || f.capture()!=before.capture()+17))) {
            stop(now,"capture-sequence-discontinuity"); return;
        }
        // Expiry may occur between service ticks. A fresh packet cannot retroactively fill it.
        if (before!=null && f.arrived()-before.capture()>40) {stop(now,"unobserved-age-gap");return;}
        previous.put(f.eye(),f);
        if (f.eye()==Eye.MOVING) roof.put(f.capture(),f);
        else waiting.add(f);
    }
    Decision update(long now,List<Frame> delivered,boolean stationaryRelease,boolean calibrated,boolean samePadEpoch) {
        if (decision!=null) {
            if (decision.accepted()) return new Decision(false,decision.capture(),now,"already-consumed");
            return decision;
        }
        if (now<=lastApplication || !stationaryRelease || !calibrated || !samePadEpoch) {
            stop(now,"invalid-context"); return decision;
        }
        lastApplication=now;
        for (Frame f:delivered) {
            consume(now,f);
            if (decision!=null) return decision;
        }
        while (!waiting.isEmpty()) {
            Frame f=waiting.peek();
            if (!linked) {
                long stamp=f.capture()-Math.floorMod(f.capture(),17);
                if (stamp<context.firstMoving()) {waiting.remove();continue;}
                Frame m=roof.get(stamp);
                if (m==null) break; // Counterpart has not arrived; no look-ahead.
                if (now-m.capture()>40 || now-f.capture()>40) {aligned=0;waiting.remove();continue;}
                aligned=m.seated() && close(m,f,0) ? aligned+1 : 0;
                if (aligned>=3) linked=true;
            }
            waiting.remove();
            if (linked) {
                if (!finite(f) || (lastPaired!=null && !close(lastPaired,f,34))) {
                    stop(now,"outward-track-lost");return decision;
                }
                lastPaired=f;
                double yaw=Math.toRadians(1);
                double half=57.65*Math.cos(yaw)+151.0625*Math.sin(yaw);
                if (clear==null && 370+f.bounds()[0]-half-536>=1) clear=f;
            }
        }
        if (clear!=null) {
            long age=now-clear.capture();
            if (context.priorLower()<context.release() || context.priorLower()>=clear.capture()) stop(now,"invalid-contact-prior");
            else if (age>29) stop(now,"clearance-expired");
            else if (now-context.priorLower()+context.onsetReserve()>context.deadline()) stop(now,"prior-expired");
            else if (age>=7) decision=new Decision(true,clear.capture(),now,"window");
        }
        if (decision==null) {
            for (Eye eye:Eye.values()) {
                if (eye==Eye.MOVING && linked) continue;
                Frame f=previous.get(eye);
                long fresh=f!=null?f.capture():(eye==Eye.MOVING?context.firstMoving():context.firstFixed());
                if (now-fresh>40) {stop(now,"camera-silent");break;}
            }
        }
        return decision;
    }
}
