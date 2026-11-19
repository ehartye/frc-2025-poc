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

class SlideFeedbackTest {
    record Phase(String piece,int phase,long prior,long capture,boolean linked) {
        String attempt(){return (piece.equals("coral-A")?"sector-minus180/":"sector-plus220/")+piece;}
    }
    private static String geometry;
    static class Mechanism {
        double x,v,voltage,peakPhase,peakVolts,stopSpeed;
        final double drag;
        final ArrayDeque<Double> queue=new ArrayDeque<>();
        Mechanism(int delay,double drag) {
            this.drag=drag+2;
            for(int i=0;i<delay;i++) queue.add(0.);
        }
        void step(double demanded,boolean welded,boolean hardStop) {
            queue.add(demanded); voltage=queue.remove();
            double current=Math.max(-20,Math.min(20,(voltage-Retreat.KE*Retreat.TO_MOTOR*v)/.12));
            peakPhase=Math.max(peakPhase,Math.abs(current)); peakVolts=Math.max(peakVolts,Math.abs(voltage));
            double effort=Retreat.FORCE_PER_AMP*Math.copySign(Math.max(0,Math.abs(current)-1.4),current);
            double friction=Math.abs(v)<.00001 ? Math.copySign(Math.min(Math.abs(effort),drag),effort) : Math.copySign(drag,v);
            double next=v+.001*(effort-friction)/Retreat.MASS;
            if (next*v<0 && Math.abs(effort)<drag) next=0;
            if (welded) next=0;
            x+=(v+next)*.0005; v=next;
            if (hardStop && (x<0 || x>Retreat.TRAVEL)) { if(x>Retreat.TRAVEL) stopSpeed=Math.max(stopSpeed,Math.abs(v)); x=Math.max(0,Math.min(Retreat.TRAVEL,x));v=0; }
        }
    }
    static class Run extends Command {
        final Phase witness; final Retreat.Policy choice; final Mechanism plant;
        final String fault; final long start,origin;
        final Retreat controller;
        long now; double demand, previousEncoder; int ticks;
        Run(Phase w,Retreat.Policy cfg,int delay,double drag,String f,int offset) {
            witness=w;choice=cfg;plant=new Mechanism(delay,drag);fault=f;controller=new Retreat(w.attempt(),cfg);
            start=Math.floorDiv(w.prior,cfg.period)*cfg.period+offset;origin=RobotController.getFPGATime();
        }
        @Override public void execute() {
            now=start+(RobotController.getFPGATime()-origin)/1000;
            long arrival=witness.capture+15+(fault.equals("late")?40:0);
            var packet = now<arrival?null:new Retreat.Clearance(fault.equals("geometry")?"unreviewed":geometry, fault.equals("wrong")?"another-pipe":witness.attempt(),
                witness.prior,witness.capture,arrival,witness.linked&&!fault.equals("lost")&&!(fault.equals("contradiction") && controller.accepted>=0 && now-controller.accepted>=80),!fault.equals("unclear"));
            // Direct string-pot and motor-derived displacement have separate quantization.
            double quantum=.635/4096;
            double motorError = fault.equals("motorSlip") && plant.x>.00005 ? .004 : 0;
            double relativeBias = fault.equals("biasLow") ? -.000254 : fault.equals("biasHigh") ? .000254 : 0;
            double pot=Math.rint((plant.x + (plant.x>.000015 ? relativeBias : 0))/quantum)*quantum;
            double increment=2*Math.PI / (42*Retreat.TO_MOTOR);
            double encoder=Math.rint(plant.x/increment)*increment+(fault.equals("encoder")?.010:0)+motorError;
            double speed=ticks==0?0:(encoder-previousEncoder)/(choice.period*.001);
            previousEncoder=encoder;
            var sense=new Retreat.Feedback(now-(fault.equals("stale")?50:0),pot,encoder,speed,
                plant.x>=Retreat.TRAVEL-.00025, fault.equals("brownout")?8:9.5,
                !(fault.equals("disableMoving") && controller.accepted>=0 && now-controller.accepted>85));
            demand=controller.update(now,packet,sense); ticks++;
        }
        @Override public void end(boolean interrupted){demand=0; controller.cancel();}
    }

    @Test void replayObservedPhasesThroughScheduler() throws Exception {
        var root=Path.of(System.getenv("CUSTODY_SOURCE"));
        var out=Path.of(System.getenv("SLIDE_OUTPUT"));
        geometry=Files.readString(root.resolve("register.txt")).strip();
        assertEquals(Retreat.REGISTER,geometry);
        assertTrue(HAL.initialize(500,0));
        DriverStationSim.setDsAttached(true); DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData(); DriverStation.refreshData();
        SimHooks.pauseTiming();
        var scheduler=CommandScheduler.getInstance();
        var seen=new HashSet<String>();
        try (var log=Files.newBufferedWriter(out.resolve("trials.csv"))) {
            log.write("piece,phase,policy,rate_ms,scheduler_offset_ms,actuator_delay_ms,drag_N,fault,command_ms,observed_motion_ms,verified_stow_ms,state,peak_phase_A,peak_motor_V,hard_stop_mps,trace_id\n");
            for(String line:Files.readAllLines(root.resolve("phases.csv")).subList(1,35)) {
                var r=line.split(",");
                var w=new Phase(r[0],Integer.parseInt(r[1]),Long.parseLong(r[2]),Long.parseLong(r[3]),Boolean.parseBoolean(r[4]));
                for(var cfg:Retreat.Policy.values()) {
                    var scenarios=new ArrayList<String>();
                    for (int lag=0;lag<=10;lag++) scenarios.add(lag+",25,normal");
                    if(w.phase==6 && cfg.label.equals("extended5")) for(String f:new String[]{"wrong","geometry","contradiction","late","motorSlip","lost","unclear","stale","encoder","jam","jamMoving","disableMoving","brownout","interrupted"})
                        scenarios.add("4,25,"+f);
                    if(cfg.label.equals("extended5")) for(String bias:new String[]{"biasLow","biasHigh"})
                        for(int lag=0;lag<=10;lag++) scenarios.add(lag+",25,"+bias);
                    if(w.phase==6 && cfg.label.equals("extended5")) for(int drag:new int[]{15,35,80,140}) scenarios.add("4,"+drag+",normal");
                    for(String scenario:scenarios) for(int offset=0;offset<(cfg.label.equals("extended5")?5:1);offset++) {
                        var p=scenario.split(",");int delay=Integer.parseInt(p[0]), drag=Integer.parseInt(p[1]);String fault=p[2];
                        var run=new Run(w,cfg,delay,drag,fault,offset);
                        scheduler.setPeriod(cfg.period/1000.);
                        scheduler.schedule(run);
                        var path=new ArrayList<Double>();

                        for(int ms=0;ms<=3100;ms++){
                            if(ms>0) SimHooks.stepTiming(.001);
                            if(ms%cfg.period==0) scheduler.run();
                            if(fault.equals("interrupted") && run.controller.accepted>=0 && run.now-run.controller.accepted>=80 && scheduler.isScheduled(run)) scheduler.cancel(run);
                            if(run.controller.accepted>=0) path.add(run.plant.x);
                            if(run.controller.state==Retreat.State.STOWED || run.controller.state==Retreat.State.FAULT) {
                                // The remaining coast is still integrated after output is removed.
                                for(int j=0;j<100;j++){run.plant.step(0,fault.equals("jam") || fault.equals("jamMoving"),true);if(run.controller.accepted>=0) path.add(run.plant.x);}
                                break;
                            }
                            run.plant.step(run.demand,fault.equals("jam") || (fault.equals("jamMoving") && run.controller.accepted>=0 && run.now-run.controller.accepted>150),true);
                        }
                        var c=run.controller;
                        assertTrue(c.state==Retreat.State.STOWED || c.state==Retreat.State.FAULT);
                        if(!fault.equals("normal") && !fault.startsWith("bias")) assertEquals(Retreat.State.FAULT,c.state,fault);
                        if(c.state==Retreat.State.STOWED) assertTrue(c.observed>c.accepted && c.observed<=w.prior+cfg.deadline);
                        String key=c.accepted<0?"none":cfg.label+"-"+delay+"-"+drag+"-"+fault+"-"+c.state;
                        if(c.accepted>=0 && seen.add(key)){
                            try(var trace=Files.newBufferedWriter(out.resolve(key+".csv"))){
                                trace.write("since_command_ms,withdrawn_m\n");
                                for(int i=0;i<path.size();i++) trace.write(i+","+path.get(i)+"\n");
                            }
                        }
                        log.write(w.piece+","+w.phase+","+cfg.label+","+cfg.period+","+offset+","+delay+","+drag+","+fault+","
                            +c.accepted+","+c.observed+","+c.finished+","+c.state+","+run.plant.peakPhase+","+run.plant.peakVolts+","+run.plant.stopSpeed+","+key+"\n");
                        scheduler.cancel(run);
                    }
                }
            }
        } finally {scheduler.cancelAll();scheduler.setPeriod(.020);SimHooks.resumeTiming();DriverStationSim.setEnabled(false);DriverStationSim.notifyNewData();DriverStation.refreshData();}
    }
}
