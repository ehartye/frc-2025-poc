package org.reefscape.controls;

import java.util.*;

/** Observes the two original streams concurrently. It never changes a packet's timestamps or camera. */
final class GapWitness {
    private final OpticalWindow.Context context;
    private final Map<Long,OpticalWindow.Frame> roof=new HashMap<>();
    private final Map<Long,OpticalWindow.Frame> fixed=new HashMap<>();
    private final EnumMap<OpticalWindow.Eye,OpticalWindow.Frame> previous=new EnumMap<>(OpticalWindow.Eye.class);
    private final Set<Long> joined=new TreeSet<>();
    private final List<String> history=new ArrayList<>();
    private long previousService=-1, missing=-1;
    private boolean recovered,failed;
    private String reason="waiting";

    GapWitness(OpticalWindow.Context context){ this.context=context; }
    String reason(){return reason;}
    List<String> history(){return List.copyOf(history);}
    private void fail(String message){failed=true;reason=message;}
    private static boolean finite(OpticalWindow.Frame f){
        if(f==null || f.bounds()==null || f.bounds().length!=6)return false;
        for(int i=0;i<6;i+=2)if(!Double.isFinite(f.bounds()[i]) ||
            !Double.isFinite(f.bounds()[i+1]) || f.bounds()[i]>f.bounds()[i+1])return false;
        return true;
    }
    private static boolean overlap(OpticalWindow.Frame a,OpticalWindow.Frame b,double tolerance){
        if(!finite(a) || !finite(b))return false;
        for(int i=0;i<6;i+=2)if(a.bounds()[i]>b.bounds()[i+1]+tolerance ||
                b.bounds()[i]>a.bounds()[i+1]+tolerance)return false;
        return true;
    }
    private boolean anchored(long stamp){
        return joined.contains(stamp-17) && joined.contains(stamp-34) && joined.contains(stamp-51);
    }
    private boolean usableRoof(long stamp,long now){
        var r=roof.get(stamp);
        var prior=roof.get(stamp-17);
        return finite(r) && r.seated() && now-r.capture()<=40 &&
               r.arrived()<=stamp-17+40 && overlap(prior,r,34) && anchored(stamp);
    }
    private void markGap(long stamp,long now){
        if(missing==stamp)return;
        if(missing>=0 || !usableRoof(stamp,now)){fail("no-independent-coverage");return;}
        missing=stamp;history.add("roof-hold@"+now+"/"+stamp);
    }
    private void consume(long now,OpticalWindow.Frame f){
        boolean moving=f.eye()==OpticalWindow.Eye.MOVING;
        if(!f.piece().equals(context.piece()) || !f.epoch().equals(context.epoch()) ||
           !f.registration().equals(moving?context.movingRegistration():context.fixedRegistration())){
            fail("wrong-registration-epoch-or-piece");return;
        }
        if(f.arrived()>now || f.capture()>f.arrived() || now-f.capture()>40){fail("frame-age");return;}
        var before=previous.get(f.eye());
        if(before==null?(f.sequence()!=0 || f.capture()!=(moving?context.firstMoving():context.firstFixed())):
                f.sequence()!=before.sequence()+1 || f.capture()!=before.capture()+17){fail("sequence");return;}
        if(before!=null && f.arrived()-before.capture()>40){
            if(moving){fail("roof-delivery-gap");return;}
            markGap(f.capture(),now);
            if(failed)return;
        }
        previous.put(f.eye(),f);
        (moving?roof:fixed).put(f.capture(),f);
        if(!moving && (!finite(f) || (before!=null && !overlap(before,f,34)))){
            fail("fixed-track-discontinuity");return;
        }
        var r=roof.get(f.capture());var s=fixed.get(f.capture());
        if(r!=null && s!=null && r.seated() && overlap(r,s,0))joined.add(f.capture());
        if(!moving && missing==f.capture()){
            if(!usableRoof(f.capture(),now) || !joined.contains(f.capture())){fail("reacquisition-mismatch");return;}
            recovered=true;history.add("fixed-reacquired@"+now+"/"+f.capture());
        }
    }
    OpticalWindow.Decision update(long now,List<OpticalWindow.Frame> delivered,boolean live){
        if(failed)return null;
        if(!live || (previousService>=0 && now-previousService!=5)){fail("context-or-clock");return null;}
        previousService=now;
        for(var f:delivered){consume(now,f);if(failed)return null;}
        var f=previous.get(OpticalWindow.Eye.FIXED);
        long last=f==null?context.firstFixed():f.capture();
        if(now-last>40){
            if(f==null || now-(last+17)>40 || !usableRoof(last+17,now)){fail("coverage-expired");return null;}
            markGap(last+17,now);return null;
        }
        if(!recovered || f==null || f.capture()<=missing)return null;
        double half=57.65*Math.cos(Math.toRadians(1))+151.0625*Math.sin(Math.toRadians(1));
        if(370+f.bounds()[0]-half-536<1)return null;
        long age=now-f.capture();
        if(context.priorLower()<context.release() || context.priorLower()>=f.capture() ||
           now-context.priorLower()+context.onsetReserve()>context.deadline() || age>29){
            fail("clearance-or-pressure-expired");return null;
        }
        if(age<7)return null;
        history.add("front-clear@"+now+"/"+f.capture());
        return new OpticalWindow.Decision(true,f.capture(),now,"recovered-window");
    }
}
