package org.reefscape.controls;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Quantized fixtures for the desktop controller. Pipe positions never reach the watchdog. */
public final class WatchFixture {
    record Port(long t, boolean entry, boolean seat, String state, double belt) {}
    record Params(int phase, int origin, int sign, int delay, int filter, int mode) {}
    record Result(long start,long confirm,long band,long fault,int differences,String terminal,String reason,double endBelt) {}

    static Map<String,List<Port>> inputs(Path f) throws IOException {
        Map<String,List<Port>> data=new LinkedHashMap<>();
        for (String line:Files.readAllLines(f)) {
            String[] c=line.split(",");
            data.computeIfAbsent(c[0],k->new ArrayList<>()).add(new Port(Long.parseLong(c[1]),c[2].equals("1"),c[3].equals("1"),c[4],Double.parseDouble(c[5])));
        }
        return data;
    }
    static double distance(double seconds,double speed,double acc) {
        if(seconds<=0)return 0;
        double ramp=speed/acc;
        return 1000*(seconds<ramp?.5*acc*seconds*seconds:speed*(seconds-.5*ramp));
    }
    static double travel(double us,double stopUs,double speed,double acceleration,double slack,boolean locked) {
        if(locked)return 0;
        return Math.max(0,distance((Math.min(us,stopUs)-580_000)/1e6,speed,acceleration)-slack);
    }
    // Modes: normal, both locked, single locked, bad polarity, missing ADC, stale delivery,
    // uncalibrated, converter replay, late lock. A valid rotating idler can still face a smooth pipe.
    static Result simulate(List<Port> source,Params test,double stopUs,boolean overwriteWithHeldSeat) {
        GuardedAcquisition guard=new GuardedAcquisition();
        int mode=test.mode,phase=test.phase,sidePitch=1250;
        long[] next={550_000L+phase,550_000L+Math.floorMod(phase+sidePitch,5000)};
        long first=-1,confirm=-1,band=-1,fault=-1;
        int differences=0;
        String terminal="",reason="";double out=0;
        double normalEnd=source.stream().filter(q->q.state.equals("DWELL")||q.state.equals("FAULT")).findFirst().map(v->v.t*1000.).orElse(2_000_000.);
        for (Port r:source) {
            long now=r.t*1000;
            for(int side=0;side<2;side++) {
                while(next[side]+test.delay<=now) {
                    long sample=next[side];next[side]+=5000;
                    if(mode==4 && side==1 && sample>=595_000)continue;
                    boolean locked=mode==1 || mode==2&&side==1;
                    double speed=test.sign==0?.16:.14;
                    double acceleration=test.sign==0?26.25:16.0;
                    // Keep the encoder physically parked after the source target stops.
                    double end=mode==8?stopUs:Math.min(stopUs,normalEnd);
                    double effective=sample-test.filter;
                    double mm=travel(effective,end,speed,acceleration,test.sign==0?0:.40,locked);
                    double dir=side==0?1.:-1.;
                    // Opposite signed endpoint errors exercise the full differential allowance.
                    double angle=dir*(360*mm/60.0+(effective<600_000?test.sign*3.:-test.sign*3.));
                    if(mode==3 && side==1)angle=-angle;
                    if(mode==7)angle=dir*360*travel(effective,end,.16,26.25,0,false)/60.;
                    double degrees=test.origin+angle; double fraction=degrees/360.-Math.floor(degrees/360.);
                    int nativeCode=(int)Math.round(fraction*4095);
                    int adc=(int)Math.round(nativeCode/4095.*1650);
                    long delivery=sample+test.delay;
                    long reported=mode==5 && sample>590_000 ? sample-15_000:sample;
                    if(mode==13 && side==1 && sample>=595_000)adc=2000;
                    if(mode==14 && sample>=595_000)reported=delivery+5_000;
                    guard.observe(side==0,new IdlerWatch.Reading(reported,delivery,adc,mode!=15 || sample<595_000));
                    if(mode==16 && sample>=595_000)guard.observe(side==0,new IdlerWatch.Reading(reported,delivery,adc,true));
                }
            }
            boolean entry=r.entry,seat=r.seat;
            if(overwriteWithHeldSeat && r.t>=600){entry=false;seat=true;}
            long portStamp=mode==12 && r.t>=600 ? r.t-20 : r.t;
            boolean enabled=mode!=11 || r.t<600;
            boolean docked=mode!=17 || r.t<600;
            var o=guard.update(r.t,portStamp,true,true,docked,enabled,entry,seat,mode!=6);
            if(o.state().equals("PROBE") && first<0)first=r.t;
            if(o.positiveDriveSeen() && confirm<0)confirm=r.t;
            terminal=o.state();reason=o.reason();out=o.beltMps();
            if(terminal.equals("IN_BAND") && band<0)band=r.t;
            if(!terminal.equals(r.state) || Math.abs(out-r.belt)>1.e-8)differences++;
            if(terminal.equals("FAULT")){fault=r.t;break;}
            // The timing sweep needs only the decision. Other runs follow the original port stream.
            if(mode==9 && confirm>=0)break;
        }
        return new Result(first,confirm,band,fault,differences,terminal,reason,out);
    }
    static void print(PrintWriter p,String key,String fixture,Params c,double stop,Result r) {
        p.printf(Locale.ROOT,"%s,%s,%d,%d,%d,%d,%d,%d,%.3f,%d,%d,%d,%d,%d,%s,%s,%.5f%n",key,fixture,c.phase,c.origin,c.sign,c.delay,c.filter,c.mode,stop,r.start,r.confirm,r.band,r.fault,r.differences,r.terminal,r.reason,r.endBelt);
    }
    public static void main(String[] a)throws Exception {
        var all=inputs(Path.of(a[0]));
        try(var report=new PrintWriter(Files.newBufferedWriter(Path.of(a[1])))){
            report.println("source,fixture,phase_us,zero_deg,error,transit_us,lag_us,mode,stall_us,probe_ms,confirmed_ms,band_ms,fault_ms,state_or_command_differences,outcome,reason,last_command");
            for(var e:all.entrySet()){
                int mode=e.getKey().startsWith("locked")?1:0;
                var c=new Params(0,359,0,2000,2000,mode);
                print(report,e.getKey(),"source",c,2e6,simulate(e.getValue(),c,2e6,false));
            }
            var good=all.get("candidate-leftSmall-15.625");
            var jam=all.get("locked-leftSmall-15.625");
            for(int phase=0;phase<5000;phase+=250)for(int zero:new int[]{0,1,173,359})for(int sign:new int[]{-1,0,1})
                for(int delay:new int[]{0,4000})for(int lag:new int[]{0,3000}){
                    var c=new Params(phase,zero,sign,delay,lag,9);
                    print(report,"healthy","range",c,2e6,simulate(good,c,2e6,false));
                    var lock=new Params(phase,zero,sign,delay,lag,1);
                    print(report,"locked","range",lock,580000,simulate(jam,lock,580000,false));
                }
            // Mode 10 replays a passing port sequence and physically plausible encoder data
            // on an explicitly empty system. Identical observations cannot establish custody.
            for(int mode:new int[]{2,3,4,5,6,7,10,11,12,13,14,15,16,17}){
                var c=new Params(4250,359,0,4000,3000,mode);
                print(report,"sensor-challenge","negative",c,2e6,simulate(mode==7?jam:good,c,2e6,mode==7));
            }
            for(int off=0;off<120;off++)for(int phase:new int[]{0,1250,2500,3750}) {
                double stop=(700+off)*1000.;var c=new Params(phase,359,0,4000,3000,8);
                print(report,"rolling-then-stuck","late",c,stop,simulate(good,c,stop,true));
            }
        }
    }
}

