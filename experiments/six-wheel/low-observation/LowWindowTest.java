package org.reefscape.controls;

import java.nio.file.Files;
import java.nio.file.Path;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/** Desktop timing ceiling once a complete debounced contact and optical clearance have been established. */
class LowWindowTest {
    static class Window extends Command {
        long prior, capture, origin, offset;
        long first = -1;
        boolean independentlyIdentified, immediate;
        int ticks;
        @Override public void execute() {
            long elapsed = RobotController.getFPGATime() - origin;
            assertEquals(0, elapsed % 20000);
            long now = offset + elapsed / 1000;
            boolean timing = immediate
                ? now - capture <= 29 && now - prior + WithdrawalTrial.ACTUATION_MS <= 150
                : now >= capture + 1 + 50 && now - prior <= 150;
            if (independentlyIdentified && capture > prior && now >= capture + 15
                && timing && first < 0) first = now;
            ticks++;
        }
    }

    @Test void scoreWindowOnDesktopClock() throws Exception {
        var folder = Path.of(System.getenv("CUSTODY_SOURCE"));
        var rows = new java.util.ArrayList<String>();
        for (var name : new String[] {"withdrawal","immediate"}) {
            var data = Files.readAllLines(folder.resolve(name + ".csv"));
            for (var line : data.subList(1,data.size())) rows.add(line + "," + name);
        }
        assertTrue(HAL.initialize(500,0));
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
        SimHooks.pauseTiming();
        var scheduler = CommandScheduler.getInstance();
        int eligible = 0;
        int ticks = 0;
        try {
            for (var row : rows) {
                var r = row.split(",",-1);
                for (boolean identified : new boolean[] {true,false}) {
                    var test = new Window();
                    test.prior = Long.parseLong(r[3]);
                    test.capture = r[2].isEmpty() ? -1 : Long.parseLong(r[2]);
                    test.offset = 20 * Math.floorDiv(test.prior,20);
                    test.origin = RobotController.getFPGATime();
                    test.independentlyIdentified = identified;
                    test.immediate = r[6].equals("immediate");
                    scheduler.schedule(test);
                    for (int i=0; i<11; ++i) {
                        if (i>0) SimHooks.stepTiming(.020);
                        scheduler.run();
                    }
                    scheduler.cancel(test);
                    long expected = identified && r[5].equals("1") ? Long.parseLong(r[4]) : -1;
                    assertEquals(expected,test.first,row);
                    assertEquals(11,test.ticks);
                    ticks += test.ticks;
                    if (test.first >= 0) eligible++;
                }
            }
            System.out.println("LOW_WINDOWS="+rows.size()+" TICKS="+ticks+" ELIGIBLE="+eligible);
        } finally {
            scheduler.cancelAll();
            SimHooks.resumeTiming();
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            DriverStation.refreshData();
        }
    }
}
