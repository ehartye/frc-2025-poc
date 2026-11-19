package org.reefscape.controls;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.nio.file.*;
import java.io.*;
import java.util.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class PacketClockTest {
    record Record(String piece,OpticalWindow.Eye eye,long capture,String status,double[] bounds) {}
    record Run(String piece,int phase,int period,int offset,String scenario,int delay,int reserve,int deadline) {}
    record Contract(String moving,String fixed,long release,long prior,List<Record> records) {}
    static List<OpticalWindow.Frame> packets(Contract ctx,Run r) {
        var out=new ArrayList<OpticalWindow.Frame>();
        var counts=new EnumMap<OpticalWindow.Eye,Integer>(OpticalWindow.Eye.class);
        for (Record v:ctx.records()) {
            boolean moving=v.eye()==OpticalWindow.Eye.MOVING;
            if (!moving && v.capture()%17!=r.phase()) continue;
            int n=counts.getOrDefault(v.eye(),0);
            counts.put(v.eye(),n+1);
            int delay=r.delay(),seq=n;
            long capture=v.capture(),arrival=capture+delay;
            String epoch="seeded-"+r.piece(), reg=moving?ctx.moving():ctx.fixed(),key=r.piece();
            double[] b=v.bounds();
            boolean seat=v.status().equals("inside");
            switch(r.scenario()) {
                case "fixed-delay" -> {if (moving) arrival=capture+15;}
                case "mixed-jitter" -> arrival=capture+(moving?new int[]{18,12,17,13}[n%4]:new int[]{11,22,12,17}[n%4]);
                case "asymmetric" -> arrival=capture+(moving?26:9);
                case "drop" -> {if (!moving && n==5) continue;}
                case "silence" -> {if (!moving && n>=5) continue;}
                case "reorder" -> {if (!moving && n==5) arrival+=23;}
                case "future-stamp" -> {if (!moving && n==5) capture=arrival+1;}
                case "wrong-revision" -> {if (moving && n==1) reg=ctx.fixed();}
                case "wrong-piece" -> {if (!moving && n==5) key=r.piece().equals("coral-A")?"coral-B":"coral-A";}
                case "old-epoch" -> {if (!moving && n==5) epoch="previous-attempt";}
                case "empty-fixed" -> {if (!moving && n==6) b=null;}
                case "jump" -> {if (!moving && n==6) b=new double[]{800,805,-2,2,-3,3};}
                case "mixed-batch" -> {if (!moving && n==4) arrival+=17;}
                case "one-late" -> {if (!moving && n==5) arrival+=11;}
                default -> {}
            }
            var f=new OpticalWindow.Frame(v.eye(),key,epoch,reg,seq,capture,arrival,seat,b);
            out.add(f);
            boolean clearance=b!=null && 370+b[0]-(57.65*Math.cos(Math.toRadians(1))+151.0625*Math.sin(Math.toRadians(1)))-536>=1;
            if (!moving && ((n==5 && r.scenario().equals("duplicate")) ||
                    (clearance && r.scenario().equals("duplicate-clear")))) out.add(f);
        }
        out.sort(Comparator.comparingLong(OpticalWindow.Frame::arrived));
        return out;
    }

    static class Consumer extends Command {
        final Run run; final Contract contract; final List<OpticalWindow.Frame> packets;
        final OpticalWindow gate;
        final long origin,local;
        int cursor;
        OpticalWindow.Decision result;
        Consumer(Contract c,Run r) {
            run=r;contract=c;packets=packets(c,r);
            long moving=c.release()+Math.floorMod(-c.release(),17);
            long fixed=c.release()+Math.floorMod(r.phase()-c.release(),17);
            gate=new OpticalWindow(new OpticalWindow.Context(r.piece(),"seeded-"+r.piece(),c.moving(),c.fixed(),
                c.release(),r.scenario().equals("future-pad")?c.prior()+300:c.prior(),moving,fixed,r.reserve(),r.deadline()));
            local=c.release()-Math.floorMod(c.release()-r.offset(),r.period());
            origin=RobotController.getFPGATime();
        }
        void sample(long now) {
            var batch=new ArrayList<OpticalWindow.Frame>();
            while (cursor<packets.size() && packets.get(cursor).arrived()<=now) batch.add(packets.get(cursor++));
            result=gate.update(now,batch,!run.scenario().equals("unregistered-pose"),!run.scenario().equals("uncalibrated"),
                !run.scenario().equals("unattributed-pad"));
        }
        @Override public void execute() {
            long delta=RobotController.getFPGATime()-origin;
            assertEquals(0,delta%(1000*run.period()));
            sample(local+delta/1000);
        }
    }

    @Test void replayExposuresOnControllerClock() throws Exception {
        assertTrue(HAL.initialize(500,0));
        Path source=Path.of(System.getenv("CUSTODY_SOURCE"));
        var records=new HashMap<String,List<Record>>();
        var lines=Files.readAllLines(source.resolve("exposures.csv"));
        for(var line:lines.subList(1,lines.size())) {
            var cols=line.split(",",-1); double[] box=null;
            if(!cols[4].isEmpty()) {
                box=new double[6]; for(int i=0;i<6;i++) box[i]=Double.parseDouble(cols[4+i]);
            }
            var record=new Record(cols[0],OpticalWindow.Eye.valueOf(cols[1].toUpperCase(Locale.ROOT)),Long.parseLong(cols[2]),cols[3],box);
            records.computeIfAbsent(cols[0],k->new ArrayList<>()).add(record);
        }
        var scheduler=CommandScheduler.getInstance();
        DriverStationSim.setDsAttached(true);DriverStationSim.setEnabled(true);DriverStationSim.notifyNewData();DriverStation.refreshData();
        SimHooks.pauseTiming();
        try(var wr=Files.newBufferedWriter(Path.of(System.getenv("TIMING_RECEIPT")))) {
            wr.write("piece,phase,period_ms,offset_ms,fault,delay_ms,result,capture_ms,tick_ms,process\n");
            for(String contractLine:Files.readAllLines(source.resolve("contexts.csv")).subList(1,3)) {
                var v=contractLine.split(",");
                var contract=new Contract(v[1],v[2],Long.parseLong(v[3]),Long.parseLong(v[4]),records.get(v[0]));
                for(int p:new int[]{20,5}) {
                    scheduler.setPeriod(p/1000.);
                    for(int phase=0;phase<17;phase++) {
                        for(String scenario:List.of("constant","fixed-delay","mixed-jitter","asymmetric","one-late","drop",
                               "silence","duplicate","reorder","future-stamp","wrong-revision","wrong-piece","old-epoch",
                               "empty-fixed","jump","mixed-batch","uncalibrated","unattributed-pad","unregistered-pose","future-pad","duplicate-clear")) {
                            boolean sweep=scenario.equals("constant")||scenario.equals("fixed-delay");
                            for(int delay=sweep?7:15;delay<=(sweep?41:15);delay++) {
                                for(int offset=0;offset<(sweep?p:1);offset++) {
                                    var run=new Run(v[0],phase,p,offset,scenario,delay,p==20?10:40,p==20?150:180);
                                    var sim=new Consumer(contract,run);
                                    // The nominal phase-zero service alignment and all faults run on actual HAL.
                                    // Offset sweeps use the same clock consumer and deterministic tick arithmetic.
                                    boolean nativeClock=offset==0 && delay==15;
                                    if(nativeClock) scheduler.schedule(sim);
                                    for(int i=0;i<100 && sim.result==null;i++) {
                                        if(nativeClock) {
                                            if(i>0) SimHooks.stepTiming(p/1000.);
                                            scheduler.run();
                                        } else sim.sample(sim.local+(long)i*p);
                                    }
                                    if(nativeClock) scheduler.cancel(sim);
                                    assertNotNull(sim.result);
                                    var res=sim.result;
                                    if(res.accepted()) {
                                        sim.sample(res.application()+p);
                                        assertFalse(sim.result.accepted());
                                        assertEquals("already-consumed",sim.result.reason());
                                    }
                                    wr.write(String.join(",",v[0],""+phase,""+p,""+offset,scenario,""+delay,res.reason(),
                                        ""+res.capture(),""+res.application(),nativeClock?"HAL":"same-consumer")+"\n");
                                }
                            }
                        }
                    }
                }
            }
        } finally {
            scheduler.cancelAll();scheduler.setPeriod(.020);
            SimHooks.resumeTiming();DriverStationSim.setEnabled(false);DriverStationSim.notifyNewData();DriverStation.refreshData();
        }
    }
}
