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
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/** Replay the coupled plant's exact digital frames on the desktop FPGA and WPILib scheduler. */
public class PortClockTest {
    static class Capture extends Command {
        PortAcquisition controller;
        String[] fields;
        PortAcquisition.Output output;
        long epoch;
        public void execute() {
            long elapsed = RobotController.getFPGATime() - epoch;
            assertEquals(1000L * Integer.parseInt(fields[2]), elapsed);
            output = controller.update(elapsed/1000, Long.parseLong(fields[3]), bit(4), bit(5),
                                       bit(6), bit(7), bit(8), bit(9));
        }
        boolean bit(int i) { return fields[i].equals("1"); }
    }

    @Test void everyPublicFrameYieldsTheSameOutputOnTheScheduler() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
        SimHooks.pauseTiming();
        var scheduler = CommandScheduler.getInstance();
        var command = new Capture();
        try {
            scheduler.schedule(command);
            var source = Files.readAllLines(Path.of(System.getenv("PORT_TRACE")));
            int trials = 0;
            for (var row : source.subList(1, source.size())) {
                String[] fields = row.split(",", -1);
                command.fields = fields;
                if (fields[2].equals("0")) {
                    command.controller = new PortAcquisition(!fields[0].startsWith("stationary-"));
                    command.epoch = RobotController.getFPGATime();
                    ++trials;
                } else {
                    SimHooks.stepTiming(.020);
                }
                scheduler.run();
                assertEquals(fields[10], command.output.state().name());
                assertEquals(fields[11], Boolean.toString(command.output.active()));
                assertEquals(Double.parseDouble(fields[12]), command.output.beltMps());
                assertEquals(fields[13], command.output.reason());
            }
            System.out.println("PORT_TICKS=" + (source.size()-1) + " TRIALS=" + trials);
        } finally {
            scheduler.cancel(command);
            SimHooks.resumeTiming();
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            DriverStation.refreshData();
        }
    }
}
