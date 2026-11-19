package org.reefscape.controls;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.nio.file.*;
import java.io.*;
import java.util.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class ReturnStationTest {
    @Test void stationEpochUsesBothPortAndIndependentStereo() throws Exception {
        var src=Path.of(System.getenv("CUSTODY_SOURCE"));
        var source=Files.readAllLines(src.resolve("cases.csv"));
        var register=Files.readString(src.resolve("register.txt")).strip();
        assertTrue(HAL.initialize(500,0));
        SimHooks.pauseTiming();
        var scheduler=CommandScheduler.getInstance();
        scheduler.setPeriod(.020);
        try(var out=new PrintWriter(System.getenv("RETURN_LOG"))){
            out.println("case,challenge,joined_ms,modeled_departure,ready_at_scheduled_tick,competition_drive");
            for(String line:source.subList(1,source.size())){
                String[] r=line.split(",");
                String name=r[0];long band=Long.parseLong(r[1]);
                boolean mechanicalDomain=Boolean.parseBoolean(r[2]);
                boolean calibratedStation=Boolean.parseBoolean(r[3]);
                Map<Long,Boolean> ports=new HashMap<>();
                for(String tick:Files.readAllLines(src.resolve(name+".ports.csv"))){
                    String[] o=tick.split(",");
                    ports.put(Long.parseLong(o[0]),o[1].equals("IN_BAND"));
                }
                List<CustodyEvidence.Frame> frames=new ArrayList<>();
                for(String row:Files.readAllLines(src.resolve(name+".csv"))){
                    String[] v=row.split(",",-1);
                    long capture=Long.parseLong(v[0]);double[] b=new double[6];
                    for(int j=0;j<6;j++)b[j]=Double.parseDouble(v[2+j]);
                    frames.add(new CustodyEvidence.Frame(frames.size(),capture,capture+15,"station-B",v[1],b[0],b[1],b[2],b[3],b[4],b[5],register));
                }
                for(String challenge:List.of("nominal","wrong-piece","missing-camera","stale","late-at-departure")){
                    long[] granted={-1};int[] next={0};boolean[] freshAtScheduled={false};
                    long origin=RobotController.getFPGATime();var gate=new CustodyEvidence();
                    var task=new Command(){
                        public boolean runsWhenDisabled(){return true;}
                        public void execute(){
                            long now=(RobotController.getFPGATime()-origin)/1000;
                            var batch=new ArrayList<CustodyEvidence.Frame>();
                            while(next[0]<frames.size()){
                                var f=frames.get(next[0]);
                                long arrives=f.arrivedMs()+((challenge.equals("stale") || (challenge.equals("late-at-departure") && f.captureMs()>=5430))?60:0);
                                if(arrives>now)break;
                                next[0]++;
                                if(!challenge.equals("missing-camera"))batch.add(new CustodyEvidence.Frame(f.sequence(),f.captureMs(),arrives,
                                  challenge.equals("wrong-piece")?"previous-epoch":f.epoch(),f.fit(),f.minU(),f.maxU(),f.minY(),f.maxY(),f.minBottom(),f.maxBottom(),f.poseSha256()));
                            }
                            boolean fresh=gate.update(now,ports.getOrDefault(now,false),true,true,"station-B",batch);
                            if(fresh && granted[0]<0)granted[0]=now;
                            if(now==5480)freshAtScheduled[0]=fresh;
                        }
                    };
                    scheduler.schedule(task);
                    for(int i=0;i<=280;i++){if(i>0)SimHooks.stepTiming(.020);scheduler.run();}
                    scheduler.cancel(task);
                    if(band<0 || !(challenge.equals("nominal") || challenge.equals("late-at-departure")))assertEquals(-1,granted[0],name);
                    boolean bench=granted[0]>=0 && mechanicalDomain;
                    boolean competition=freshAtScheduled[0] && mechanicalDomain && calibratedStation;
                    out.println(name+","+challenge+","+granted[0]+","+bench+","+(freshAtScheduled[0] && mechanicalDomain)+","+competition);
                }
            }
        }finally{scheduler.cancelAll();scheduler.setPeriod(.020);SimHooks.resumeTiming();}
    }
}
