package org.reefscape.controls;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.reefscape.controls.ReceiverIO.*;

class DesktopClockTest {
    static class Fixture implements ReceiverIO {
        long ms, attempt;
        double lift = 940, slide;
        boolean entry, seat, release, target, exit, jam;
        Demand output = Demand.neutral();
        public void beginAttempt(long id) { attempt = id; }
        public void write(Demand demand) { output = demand; }
        public Inputs read() {
            return new Inputs(new Sensors(ms, lift, slide, false, false, slide == 0, slide == 370,
                    entry, seat, new Exit(exit, attempt, ReceiverIO.pieceKey(attempt), ms, ms, true), jam, true),
                    new Approach(ms, !target, target, true, true, true), release);
        }
    }

    @Test void traceBothProfilesOnTheDesktopScheduler() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
        var rows = new ArrayList<>(List.of("profile,tick_ms,state,entry,seat,release,raw_mps,issued_mps,active"));
        var scheduler = CommandScheduler.getInstance();
        SimHooks.pauseTiming();
        try {
            for (var name : List.of("HISTORICAL", "RAMP_090", "RAMP_090_ENTRY_CLEAR")) {
                boolean entryClear = name.equals("RAMP_090_ENTRY_CLEAR");
                var profile = entryClear ? DesktopReleaseIO.Profile.RAMP_090 : DesktopReleaseIO.Profile.valueOf(name);
                var io = new Fixture();
                var adapter = new DesktopReleaseIO(io, () -> io.ms, profile);
                var receiver = new ReceiverSubsystem(adapter, () -> io.ms);
                var command = new StationCycle(receiver);
                try {
                    scheduler.schedule(command);
                    long epochMicros = RobotController.getFPGATime();
                    for (int tick = 1; tick <= 45; ++tick) {
                        SimHooks.stepTiming(0.020);
                        long elapsedMicros = RobotController.getFPGATime() - epochMicros;
                        assertEquals(20_000L * tick, elapsedMicros);
                        io.ms = elapsedMicros / 1000;
                        if (tick == 2) io.slide = 240;
                        if (tick == 4) io.entry = true;
                        if (tick == 7) io.seat = true;
                        if (entryClear && tick == 8) io.entry = false;
                        if (tick == 11) io.slide = 0;
                        if (tick == 12) io.lift = 670;
                        if (tick == 13) io.target = true;
                        if (tick == 14) io.lift = 560;
                        if (tick == 15) io.slide = 370;
                        if (tick == 16) io.release = true;
                        if (tick == 22) io.seat = false;
                        if (tick == 25) io.entry = false;
                        if (tick == 36) io.exit = true;
                        if (tick == 45) io.jam = true;
                        scheduler.run();
                        rows.add(String.join(",", name, Long.toString(io.ms), command.state().name(),
                            Boolean.toString(io.entry), Boolean.toString(io.seat), Boolean.toString(io.release),
                            Double.toString(adapter.raw.beltMps()), Double.toString(io.output.beltMps()),
                            Boolean.toString(io.output.active())));
                        if (entryClear && tick == 8) {
                            assertEquals("seat not sustained", receiver.fault());
                            break;
                        }
                        if (tick >= 2 && tick <= 6) assertEquals(-0.2, io.output.beltMps());
                        if (tick >= 16 && tick < 36) {
                            assertEquals(1.05, adapter.raw.beltMps());
                            assertEquals(profile == DesktopReleaseIO.Profile.HISTORICAL ? 1.05 :
                                Math.min(0.90, (tick - 16) * 0.45), io.output.beltMps(), 1e-12);
                        }
                        if (tick >= 36) assertEquals(0, io.output.beltMps());
                        if (tick >= 36 && tick < 45) {
                            assertEquals(StationCycle.State.WAIT_WITHDRAW, command.state());
                            assertEquals(0, io.output.slideVelocityMmPerS());
                        }
                    }
                    assertEquals(StationCycle.State.FAULT, command.state());
                    assertEquals(Demand.neutral(), io.output);
                } finally {
                    scheduler.cancelAll();
                    scheduler.unregisterSubsystem(receiver);
                }
            }
        } finally {
            SimHooks.resumeTiming();
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            DriverStation.refreshData();
        }
        Files.writeString(Path.of(System.getenv("BELT_CLOCK_TRACE")), String.join("\n", rows) + "\n");
    }
}
