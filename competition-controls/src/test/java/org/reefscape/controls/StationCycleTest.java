package org.reefscape.controls;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.*;
import org.reefscape.controls.ReceiverIO.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.reefscape.controls.StationCycle.State.*;

class StationCycleTest {
    static final class FakeIO implements ReceiverIO {
        long time, sensorLag, poseLag, attempt, exitAttempt, earliestLag, latestLag;
        boolean calibrated=true, wrongPiece;
        @Override public void beginAttempt(long id) { attempt=id; exitAttempt=id; }
        double lift=940, slide;
        boolean seat, entry, exit, jam, release, target, station=true, clear=true, stopped=true;
        boolean lower, upper;
        Demand last = Demand.neutral();
        final List<Demand> writes = new ArrayList<>();
        @Override public Inputs read() {
            return new Inputs(new Sensors(time-sensorLag,lift,slide,lower,upper,slide==0,
                slide==370,entry,seat,exit ? new Exit(true,exitAttempt,wrongPiece ? "other" : ReceiverIO.pieceKey(attempt),
                    time-earliestLag,time-latestLag,calibrated) : new Exit(false,0,"",0,0,false),jam,true),
                new Approach(time-poseLag,station,target,clear,stopped,true),release);
        }
        @Override public void write(Demand next) { last=next; writes.add(next); }
    }

    FakeIO io;
    ReceiverSubsystem receiver;
    StationCycle command;
    CommandScheduler scheduler;

    @BeforeAll static void desktopRuntime() {
        assertTrue(HAL.initialize(500,0));
        System.out.println("CONTROLS_RUN_ID="+System.getenv("REEF_CONTROLS_RUN"));
    }
    static void enabled(boolean flag) {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(flag);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
    }
    @BeforeEach void setup() {
        enabled(true);
        scheduler=CommandScheduler.getInstance();
        io=new FakeIO();
        receiver=new ReceiverSubsystem(io,()->io.time);
        command=new StationCycle(receiver);
    }
    @AfterEach void cleanup() {
        scheduler.cancelAll();
        scheduler.unregisterSubsystem(receiver);
        enabled(false);
    }
    void tick() { io.time+=20; scheduler.run(); }
    void acquire() {
        command.schedule(); tick();
        assertEquals(DEPLOY,command.state()); assertEquals(240,io.last.slideMm());
        io.slide=240; tick(); assertEquals(WAIT_ENTRY,command.state());
        assertTrue(io.last.beltMps()<0);
        io.entry=true; tick(); assertEquals(WAIT_SEAT,command.state());
        io.seat=true; tick(); assertEquals(CONFIRM,command.state());
        tick(); tick(); assertEquals(CONFIRM,command.state());
        tick(); assertEquals(RETRACT,command.state());
    }
    void carry() {
        acquire(); io.slide=0; tick(); assertEquals(LOWER_CARRY,command.state());
        assertFalse(io.last.drivePermit());
        io.lift=670; tick(); assertEquals(CARRY,command.state()); assertTrue(io.last.drivePermit());
    }
    void ready() {
        carry();
        io.station=false; io.target=true; tick(); assertEquals(LOWER_PLACE,command.state());
        assertFalse(io.last.drivePermit()); io.lift=560; tick();
        assertEquals(EXTEND_PLACE,command.state());
        io.slide=370; tick(); assertEquals(READY,command.state());
    }

    @Test void deployedCoordinatesMatchTheProjectMission() {
        double[] poses=java.util.Arrays.stream(System.getenv("REEF_CONTROLS_POSES").split(","))
            .mapToDouble(Double::parseDouble).toArray();
        assertArrayEquals(new double[]{940,0,940,240,670,0,560,370},poses);
        assertEquals(ReceiverSubsystem.STATION_MM,poses[2]);
        assertEquals(ReceiverSubsystem.RECEIVE_MM,poses[3]);
        assertEquals(ReceiverSubsystem.CARRY_MM,poses[4]);
        assertEquals(ReceiverSubsystem.PLACE_MM,poses[6]);
        assertEquals(ReceiverSubsystem.DEPLOY_MM,poses[7]);
        ready(); io.release=true; tick();
        assertEquals(Double.parseDouble(System.getenv("REEF_CONTROLS_RELEASE")),io.last.beltMps());
    }

    @Test void acquireConfirmCarryReleaseAndWithdraw() {
        io.release=true; ready(); tick();
        assertEquals(READY,command.state()); // A release pressed before readiness cannot start feed.
        io.release=false; tick(); io.release=true; tick();
        assertEquals(RELEASE_SEAT,command.state()); assertEquals(1.05,io.last.beltMps());
        io.seat=false; tick(); assertEquals(RELEASE_ENTRY,command.state());
        io.entry=false; tick(); assertEquals(WAIT_EXIT,command.state()); assertEquals(370,io.last.slideMm());
        io.exit=true; tick(); assertEquals(WAIT_WITHDRAW,command.state());
        tick(); tick(); assertEquals(370,io.last.slideMm()); tick();
        assertEquals(WITHDRAW,command.state()); assertEquals(370,io.last.slideMm(),1e-8);
        for(int i=0;i<150 && command.isScheduled();i++) {io.slide=io.last.slideMm(); tick();}
        assertEquals(COMPLETE,command.state());
        assertEquals(Demand.neutral(),io.last); assertFalse(command.isScheduled());
        assertTrue(receiver.fault().isEmpty());
    }

    @Test void invalidStuckSensorRequiresEmptyStoppedExplicitRecovery() {
        io.seat=true; command.schedule(); tick();
        assertEquals(FAULT,command.state()); assertFalse(receiver.acknowledgeRecovery());
        io.seat=false; io.stopped=false; assertFalse(receiver.acknowledgeRecovery());
        io.stopped=true; assertTrue(receiver.acknowledgeRecovery()); assertFalse(command.isScheduled());
        command.schedule(); tick(); assertEquals(DEPLOY,command.state());
    }

    @Test void unchangedSeatTimestampCannotCompleteDwell() {
        command.schedule(); tick(); io.slide=240; tick(); io.entry=true; tick(); io.seat=true; tick();
        for(int i=1;i<=3;i++){ io.sensorLag=20L*i; tick(); assertEquals(CONFIRM,command.state()); }
        io.sensorLag=80; tick();
        assertEquals(FAULT,command.state()); assertEquals(Demand.neutral(),io.last);
    }

    @Test void lostCustodyOrTargetPermissionInhibitsActuation() {
        ready(); io.release=true; tick(); io.clear=false; tick();
        assertEquals(FAULT,command.state()); assertEquals(Demand.neutral(),io.last);
        assertEquals("target permit lost",receiver.fault());
    }
    @Test void lostSeatBlocksDriveWithinOneControlTick() {
        carry(); io.seat=false; tick(); assertEquals("custody witnesses lost",receiver.fault());
        assertEquals(Demand.neutral(),io.last);
    }
    @Test void jamAndStalePoseAreLatched() {
        command.schedule(); tick(); io.jam=true; tick(); assertEquals("jam indicated",receiver.fault());
        assertEquals(Demand.neutral(),io.last);
        io.jam=false; assertTrue(receiver.acknowledgeRecovery());
        command.schedule(); tick(); io.poseLag=100; tick();
        assertEquals("invalid or stale chassis",receiver.fault());
    }
    @Test void stuckAxisTimesOutEvenWithFreshEncoderFrames() {
        command.schedule(); tick();
        io.time+=2500; tick();
        assertEquals("timeout in DEPLOY",receiver.fault()); assertEquals(Demand.neutral(),io.last);
    }
    @Test void latchedEntryDuringReleaseCannotStartWithdrawal() {
        ready(); io.release=true; tick(); io.seat=false; tick();
        io.time+=2500; tick(); assertEquals("timeout in RELEASE_ENTRY",receiver.fault());
        assertEquals(Demand.neutral(),io.last);
    }
    @Test void stuckWithdrawalIsDetectedBeforeFullRetractionTimeout() {
        ready(); io.release=true; tick(); io.seat=false; tick(); io.entry=false; tick();
        io.exit=true; tick(); tick(); tick(); tick(); io.time+=170; tick();
        assertEquals("withdrawal tracking disagreement",receiver.fault());
        assertEquals(Demand.neutral(),io.last);
    }
    private void awaitExit() {
        ready(); io.release=true; tick(); io.seat=false; tick(); io.entry=false; tick();
        assertEquals(WAIT_EXIT,command.state());
    }
    @Test void uncalibratedStereoCannotMove() {
        awaitExit(); io.calibrated=false; io.exit=true; tick();
        assertEquals(FAULT,command.state()); assertFalse(io.last.active());
    }
    @Test void wrongPieceCannotMove() {
        awaitExit(); io.wrongPiece=true; io.exit=true; tick(); assertEquals(FAULT,command.state());
    }
    @Test void tooWideAnIntervalCannotAuthorize() {
        awaitExit(); for (int i=0;i<6;i++) tick();
        io.earliestLag=110; io.latestLag=0; io.exit=true; tick(); assertEquals(FAULT,command.state());
    }
    @Test void frameFromFutureCannotAuthorize() {
        awaitExit(); io.latestLag=-1; io.exit=true; tick(); assertEquals(FAULT,command.state());
    }
    @Test void schedulingDelayCannotStartLateRetreat() {
        awaitExit(); io.exit=true; tick(); io.time+=140; tick();
        assertEquals(FAULT,command.state()); assertFalse(io.last.active());
    }

    @Test void exitFromAnotherAttemptCannotAuthorizeWithdrawal() {
        ready(); io.release=true; tick(); io.seat=false; tick(); io.entry=false; tick();
        io.exit=true; io.exitAttempt=io.attempt-1; tick();
        assertEquals("exit belongs to another attempt",receiver.fault());
        assertEquals(Demand.neutral(),io.last);
    }
    @Test void cancellationImmediatelyDropsAllDemandsAndBlocksSilentRestart() {
        carry(); command.cancel(); assertEquals(Demand.neutral(),io.last);
        assertEquals("interrupted or disabled",receiver.fault());
        command.schedule(); tick(); assertEquals(FAULT,command.state());
    }
    @Test void driverStationDisableCancelsAndEnableAloneStaysNeutral() {
        carry(); enabled(false); tick();
        assertFalse(command.isScheduled()); assertEquals(Demand.neutral(),io.last);
        enabled(true); tick(); assertFalse(command.isScheduled());
        assertEquals(Demand.neutral(),io.last); assertFalse(receiver.acknowledgeRecovery());
    }
    @Test void sensorPortGeometryRejectsTheOldSeatLocation() {
        double center=80, radius=57.15;
        assertTrue(Math.abs(40-center)<radius); assertTrue(Math.abs(100-center)<radius);
        assertFalse(Math.abs(-60-center)<=radius);
        // At +160 mm, both projection points clear, while the tail is still over the +165 mm shelf.
        assertTrue(160-radius>100 && 160-radius<165);
    }
}
