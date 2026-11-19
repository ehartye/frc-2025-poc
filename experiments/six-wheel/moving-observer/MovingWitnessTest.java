package org.reefscape.controls;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/** Conditional desktop observation only. No subsystem or hardware output. */
class MovingWitnessTest {
    record Frame(String source, long capture, long arrival, boolean valid, double[] bounds) {}

    static class Observer extends Command {
        final String piece, challenge;
        final List<Frame> frames;
        final int station, drive, extend, release;
        long origin;
        int offset, cursor, authorized, revoked;
        boolean seeded, broken;
        Frame latest;
        Observer(String id,String test,List<Frame> data,int band,int go,int slide,int place) {
            piece=id; challenge=test; frames=data; station=band; drive=go; extend=slide; release=place;
        }
        @Override public void execute() {
            long us=RobotController.getFPGATime()-origin;
            assertEquals(0,us%20000);
            long now=offset+us/1000;
            while (cursor<frames.size() && frames.get(cursor).arrival()<=now) {
                var f=frames.get(cursor++);
                if (challenge.equals("stale") && now>=extend-60) continue;
                boolean wrong=challenge.equals("wrong-epoch") && now>=drive;
                boolean empty=challenge.equals("empty") && now>=drive;
                if (latest!=null && (f.capture()-latest.capture()!=17 || latest.arrival()>=f.arrival())) broken=true;
                if (!piece.equals(f.source()) || wrong || empty || !f.valid()) broken=true;
                latest=f;
            }
            if (challenge.equals("uncalibrated")) broken=true;
            if (latest!=null && now-latest.capture()>40) broken=true;
            boolean bounded=latest!=null && latest.valid();
            if (bounded) {
                double age=(now-latest.capture()+1)/1000.;
                double[] b=latest.bounds();
                // One millisecond clock allowance plus tested axis speed ceilings in shelf coordinates.
                bounded = age>=0 && b[0]-120*age>=55 && b[1]+120*age<=100
                       && b[2]-10*age>=-8 && b[3]+10*age<=8
                       && b[4]-20*age>=-12 && b[5]+20*age<=12;
            }
            // Only the independently corroborated station exposure can seed this attempt.
            if (now==station && bounded && !broken) seeded=true;
            if (seeded && !bounded) broken=true;
            boolean permits=seeded && !broken && bounded;
            if (now>=drive && permits) authorized++;
            if (now>=drive && !permits) revoked++;
            if (challenge.equals("normal") && now>=station) assertTrue(permits,piece+" at "+now);
            if (!challenge.equals("normal") && now>=extend+20) assertFalse(permits,challenge+" at "+now);
        }
    }

    @Test void samePieceThroughBothMotions() throws Exception {
        assertTrue(HAL.initialize(500,0));
        var rows=Files.readAllLines(Path.of(System.getenv("CUSTODY_SOURCE"),"frames.csv"));
        var data=new HashMap<String,List<Frame>>();
        for (var line:rows.subList(1,rows.size())) {
            var r=line.split(",",-1);
            double[] b=new double[6];
            boolean good=r[3].equals("inside");
            if (good) for(int i=0;i<6;i++) b[i]=Double.parseDouble(r[4+i]);
            var f=new Frame(r[0],Long.parseLong(r[1]),Long.parseLong(r[2]),good,b);
            data.computeIfAbsent(r[0],x->new ArrayList<>()).add(f);
        }
        var scheduler=CommandScheduler.getInstance();
        DriverStationSim.setDsAttached(true); DriverStationSim.setEnabled(true); DriverStationSim.notifyNewData();
        DriverStation.refreshData(); SimHooks.pauseTiming();
        try {
            for (String piece:List.of("coral-A","coral-B")) {
                boolean a=piece.equals("coral-A");
                for (String mode:List.of("normal","stale","wrong-epoch","empty","uncalibrated")) {
                    var cmd=new Observer(piece,mode,data.get(piece),a?1560:1580,a?5480:5500,a?23440:23160,a?26440:26160);
                    cmd.offset=1400; cmd.origin=RobotController.getFPGATime();
                    scheduler.schedule(cmd);
                    for (int t=cmd.offset;t<=cmd.release;t+=20) {
                        if (t>cmd.offset) SimHooks.stepTiming(.020);
                        scheduler.run();
                    }
                    scheduler.cancel(cmd);
                    System.out.println("MOVING "+piece+" "+mode+" "+cmd.authorized+" "+cmd.revoked);
                }
            }
        } finally {
            scheduler.cancelAll(); SimHooks.resumeTiming(); DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData(); DriverStation.refreshData();
        }
    }
}
