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

class CustodyClockTest {
    static double number(String value) { return value.isEmpty() ? Double.NaN : Double.parseDouble(value); }
    static class Observer extends Command {
        PortAcquisition ports;
        CustodyEvidence custody;
        long start;
        String[] row;
        List<CustodyEvidence.Frame> frames;
        long positive;
        public void execute() {
            long us = RobotController.getFPGATime() - start;
            long now = Long.parseLong(row[1]);
            assertEquals(now * 1000, us);
            var port = ports.update(now, Long.parseLong(row[2]), bit(3), bit(4), bit(5), bit(6), bit(7), bit(8));
            assertEquals(row[12], port.state().name());
            boolean seen = custody.update(now,port.state()==PortAcquisition.State.IN_BAND,bit(9),bit(10),row[11],frames);
            assertEquals(bit(13),seen,row[0]+" @ "+row[1]);
            if (seen) ++positive;
        }
        boolean bit(int i) { return row[i].equals("1"); }
    }
    @Test void replayImagesAndPortsOnTheSameClock() throws Exception {
        assertTrue(HAL.initialize(500,0));
        Path source=Path.of(System.getenv("CUSTODY_SOURCE"));
        var applications=Files.readAllLines(source.resolve("application.csv"));
        var camera=Files.readAllLines(source.resolve("camera.csv"));
        var byTick=new HashMap<String,List<CustodyEvidence.Frame>>();
        for (var line : camera.subList(1,camera.size())) {
            String[] r=line.split(",",-1);
            var f=new CustodyEvidence.Frame(Long.parseLong(r[2]),Long.parseLong(r[3]),Long.parseLong(r[4]),
                r[5],r[6],number(r[7]),number(r[8]),number(r[9]),number(r[10]),number(r[11]),number(r[12]),r[13]);
            byTick.computeIfAbsent(r[0]+" "+r[1],k->new ArrayList<>()).add(f);
        }
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
        SimHooks.pauseTiming();
        var observer=new Observer();
        var scheduler=CommandScheduler.getInstance();
        try {
            scheduler.schedule(observer);
            for (var line : applications.subList(1,applications.size())) {
                String[] r=line.split(",",-1);
                if (r[1].equals("0")) {
                    observer.start=RobotController.getFPGATime();
                    observer.ports=new PortAcquisition(true);
                    observer.custody=new CustodyEvidence();
                } else SimHooks.stepTiming(.020);
                observer.row=r;
                observer.frames=byTick.getOrDefault(r[0]+" "+r[1],List.of());
                scheduler.run();
            }
            System.out.println("CUSTODY_TICKS="+(applications.size()-1)+" FRAMES="+(camera.size()-1)+" POSITIVE="+observer.positive);
        } finally {
            scheduler.cancel(observer);
            SimHooks.resumeTiming();
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            DriverStation.refreshData();
        }
    }
}
