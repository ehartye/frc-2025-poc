package org.reefscape.controls;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class IntegratedTest {
    static final class Trial {
        final String piece; final long epochUs, release, driveWall, stockWall, feedWall, prior;
        final PortAcquisition ports=new PortAcquisition(true);
        final CustodyEvidence seat=new CustodyEvidence();
        final Retreat slide;
        final SlideFeedbackTest.Mechanism motor=new SlideFeedbackTest.Mechanism(10,25);
        final List<String[]> port=new ArrayList<>();
        final List<CustodyEvidence.Frame> station=new ArrayList<>();
        final List<MovingWitnessTest.Frame> moving=new ArrayList<>();
        final List<PacketClockTest.Record> releaseFrames=new ArrayList<>();
        List<OpticalWindow.Frame> flight;
        OpticalWindow gate;
        MovingWitnessTest.Frame latest;
        int pi,si,mi,fi;
        boolean seeded,broken,drive,requested,failed,stowed,issued;
        long commandWall=-1,confirmedWall=-1;
        double volts,previousMotor;
        final String config,stroke;
        final List<double[]> path=new ArrayList<>();

        Trial(String[] row,List<String> regs){
            piece=row[1];epochUs=Long.parseLong(row[2]);release=(long)Math.ceil(Double.parseDouble(row[3]));
            driveWall=Long.parseLong(row[4]);stockWall=18000+1000*Long.parseLong(row[5]);feedWall=Long.parseLong(row[7]);
            boolean a=piece.equals("coral-A"); prior=a?26593:26301;
            stroke=(a?"sector-minus180/":"sector-plus220/")+piece;
            slide=new Retreat(stroke,Retreat.Policy.REGISTERED); config=regs.get(3);
        }
        long local(long wall){return Math.floorDiv(1000*wall-epochUs,1000);}
        void prepare(List<String> reg,String scenario){
            var contract=new PacketClockTest.Contract(reg.get(2),reg.get(1),release,prior,releaseFrames);
            String mode=scenario.equals("lateB") && piece.equals("coral-B") ? "one-late":"constant";
            flight=PacketClockTest.packets(contract,new PacketClockTest.Run(piece,0,5,0,mode,15,40,180));
            long first=release+Math.floorMod(-release,17);
            gate=new OpticalWindow(new OpticalWindow.Context(piece,"seeded-"+piece,reg.get(2),reg.get(1),release,prior,first,first,40,180));
        }
        void event(PrintWriter log,String scenario,long w,String e,String detail){
            log.printf(Locale.ROOT,"%s,%s,%d,%d,%s,%s%n",scenario,piece,w,local(w),e,detail);
        }
        void advance(long w,String mode,PrintWriter log){
            long t=local(w);
            if (!issued && w>=feedWall) {
                if(w<stockWall)throw new IllegalStateException("unstocked feeder");
                issued=true;event(log,mode,w,"feed","station dispenser assigned unique identity");
            }
            if(!issued || t<0 || failed)return;
            while(pi<port.size() && Long.parseLong(port.get(pi)[2])<=t){
                var p=port.get(pi++);long now=Long.parseLong(p[2]);
                var obs=ports.update(now,Long.parseLong(p[3]),p[4].equals("1"),p[5].equals("1"),p[6].equals("1"),
                         p[7].equals("1"),p[8].equals("1"),p[9].equals("1"));
                assertEquals(p[10],obs.state().name());
            }
            if(w%20==0 && !drive){
                var batch=new ArrayList<CustodyEvidence.Frame>();
                while(si<station.size() && station.get(si).arrivedMs()<=t)batch.add(station.get(si++));
                boolean inBand=pi>0 && port.get(pi-1)[10].equals("IN_BAND");
                if(seat.update(t,inBand,true,true,piece,batch) && !seeded){
                    seeded=true;event(log,mode,w,"seat","port plus five fixed stereo exposures");
                }
            }
            if(w%5==0 && t<=release+5){
                while(mi<moving.size() && moving.get(mi).arrival()<=t){
                    var f=moving.get(mi++);
                    if(latest!=null && (f.capture()-latest.capture()!=17 || f.arrival()-latest.capture()>40))broken=true;
                    if(!f.valid() || !piece.equals(f.source()))broken=true;
                    latest=f;
                }
                if(seeded && (latest==null || t-latest.capture()>40))broken=true;
                if(seeded && latest!=null){
                    var b=latest.bounds();double age=(t-latest.capture()+1)/1000.;
                    if(age<0 || b[0]-120*age<55 || b[1]+120*age>100 || b[2]-10*age< -8 || b[3]+10*age>8
                         || b[4]-20*age< -12 || b[5]+20*age>12)broken=true;
                }
            }
            if(w>=driveWall && !drive){
                if(!seeded || broken){failed=true;event(log,mode,w,"hold","custody unavailable at drive");return;}
                drive=true;event(log,mode,w,"carry","continuous moving-head track; drive permit armed");
            }
            if(broken && drive){failed=true;event(log,mode,w,"hold","carry identity lost");return;}
            if(!drive || t<release)return;
            if(!requested){event(log,mode,w,"release","registered held release");requested=true;}
            if(w%5==0 && !stowed){
                if(commandWall<0){
                    var batch=new ArrayList<OpticalWindow.Frame>();
                    while(fi<flight.size() && flight.get(fi).arrived()<=t)batch.add(flight.get(fi++));
                    var d=gate.update(t,batch,true,true,true);
                    if(d!=null && !d.accepted()){failed=true;event(log,mode,w,"hold",d.reason());return;}
                    if(d!=null) {commandWall=w; capture=d.capture();event(log,mode,w,"withdraw","whole-front exposure "+capture);}
                }
                if(commandWall>=0){
                    double q=.635/4096, step=2*Math.PI/(42*Retreat.TO_MOTOR);
                    double pot=Math.rint(motor.x/q)*q, enc=Math.rint(motor.x/step)*step;
                    double speed=(enc-previousMotor)/.005;previousMotor=enc;
                    var feedback=new Retreat.Feedback(t,pot,enc,speed,motor.x>=.36975,9.5,true);
                    volts=slide.update(t,new Retreat.Clearance(config,stroke,prior,capture,capture+15,true,true),feedback);
                    if(slide.state==Retreat.State.FAULT){failed=true;event(log,mode,w,"hold","rack-feedback");return;}
                    if(slide.observed>=0 && confirmedWall<0){confirmedWall=w;event(log,mode,w,"motion","independent linear and motor travel");}
                    if(slide.state==Retreat.State.STOWED){stowed=true;event(log,mode,w,"stow","endpoint and speed confirmed");}
                }
            }
            if(commandWall>=0 && w-commandWall<=3000){
                double position=motor.x,velocity=motor.v;
                motor.step(volts,false,true);
                double current=Math.max(-20,Math.min(20,(motor.voltage-Retreat.KE*Retreat.TO_MOTOR*velocity)/.12));
                path.add(new double[]{position,current,motor.voltage});
            }
        }
        long capture;
    }

    static class Match extends Command {
        final Trial a,b;final String mode;final PrintWriter log; final long origin;
        Match(Trial a,Trial b,String mode,PrintWriter log){this.a=a;this.b=b;this.mode=mode;this.log=log;origin=RobotController.getFPGATime();}
        public void execute(){long w=(RobotController.getFPGATime()-origin)/1000;
            a.advance(w,mode,log);
            if(a.stowed && !mode.equals("single"))b.advance(w,mode,log);
        }
    }

    @Test void bothTripsAndLatePacketShareOneClock() throws Exception {
        Path src=Path.of(System.getenv("CUSTODY_SOURCE")),out=Path.of(System.getenv("INTEGRATED_OUTPUT"));
        List<String> regs=Files.readAllLines(src.resolve("registers.txt"));
        assertEquals(Retreat.REGISTER,regs.get(3));
        assertTrue(HAL.initialize(500,0));
        DriverStationSim.setDsAttached(true);DriverStationSim.setEnabled(true);DriverStationSim.notifyNewData();DriverStation.refreshData();
        SimHooks.pauseTiming(); var scheduler=CommandScheduler.getInstance();scheduler.setPeriod(.001);
        try(var log=new PrintWriter(Files.newBufferedWriter(out.resolve("events.csv")))){
            log.println("scenario,piece,wall_ms,local_ms,event,witness");
            for(String mode:List.of("nominal","lateB","single")){
                Map<String,Trial> tests=new LinkedHashMap<>();
                var input=Files.readAllLines(src.resolve("inputs.csv"));
                for(String line:input.subList(1,input.size())){
                    String[] r=line.split(",",-1);
                    if(r[0].equals("clock"))tests.put(r[1],new Trial(r,regs));
                    else if(r[0].equals("port"))tests.get(r[1]).port.add(r);
                    else {long t=Long.parseLong(r[2]);double[] b=new double[6]; for(int i=0;i<6;i++) b[i]=Double.parseDouble(r[4+i]);
                        var tr=tests.get(r[1]);tr.station.add(new CustodyEvidence.Frame(tr.station.size(),t,t+15,r[1],r[3],b[0],b[1],b[2],b[3],b[4],b[5],regs.get(0)));}
                }
                var frames=Files.readAllLines(src.resolve("frames.csv"));
                for(String line:frames.subList(1,frames.size())){String[] r=line.split(",",-1);double[] b=new double[6];
                    boolean good=r[3].equals("inside");if(good)for(int i=0;i<6;i++)b[i]=Double.parseDouble(r[4+i]);
                    tests.get(r[0]).moving.add(new MovingWitnessTest.Frame(r[0],Long.parseLong(r[1]),Long.parseLong(r[2]),good,b));}
                frames=Files.readAllLines(src.resolve("exposures.csv"));
                for(String line:frames.subList(1,frames.size())){String[] r=line.split(",",-1);double[] b=null;
                    if(!r[4].isEmpty()){b=new double[6];for(int i=0;i<6;i++)b[i]=Double.parseDouble(r[4+i]);}
                    tests.get(r[0]).releaseFrames.add(new PacketClockTest.Record(r[0],OpticalWindow.Eye.valueOf(r[1].toUpperCase(Locale.ROOT)),Long.parseLong(r[2]),r[3],b));}
                for(var tr:tests.values())tr.prepare(regs,mode);
                var a=tests.get("coral-A");var b=tests.get("coral-B");var command=new Match(a,b,mode,log);scheduler.schedule(command);
                for(int i=0;i<=156000;i++){if(i>0)SimHooks.stepTiming(.001);scheduler.run();}
                scheduler.cancel(command);
                for(var tr:tests.values())if(!tr.path.isEmpty()){
                    try(var path=new PrintWriter(Files.newBufferedWriter(out.resolve("path-"+mode+"-"+tr.piece+".csv")))){
                        path.println("ms,withdrawn_m,phase_A,motor_V");for(int i=0;i<tr.path.size();i++){var v=tr.path.get(i);path.println(i+","+v[0]+","+v[1]+","+v[2]);}}}
                assertTrue(a.stowed, mode);assertEquals(mode.equals("nominal"),b.stowed,mode);assertEquals(mode.equals("lateB"),b.failed,mode);
            }
        } finally{scheduler.cancelAll();scheduler.setPeriod(.020);SimHooks.resumeTiming();DriverStationSim.setEnabled(false);DriverStationSim.notifyNewData();DriverStation.refreshData();}
    }
}
