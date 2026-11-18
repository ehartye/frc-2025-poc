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
    private static final String[] ROW=System.getenv("REEF_WHOLE_ROBOT_TRIAL").split(",");
    private static final WithdrawalTrial TRIAL=new WithdrawalTrial(ROW[0],ROW[1],ROW[2],Integer.parseInt(ROW[3]),
            Integer.parseInt(ROW[4]),Long.parseLong(ROW[5]),Double.parseDouble(ROW[6]),Long.parseLong(ROW[7]));
    static final class FakeIO implements ReceiverIO {
        long time, sensorLag, poseLag, attempt, exitAttempt, earliestLag, latestLag;
        boolean calibrated=true, wrongPiece, whole;
        String track=TRIAL.pieceId(), reportPiece=TRIAL.pieceId(), assembly=TRIAL.assemblySha256(), study=TRIAL.studySha256();
        String frame=WithdrawalTrial.FRAME, clock=WithdrawalTrial.CLOCK;
        int phase=TRIAL.phase(), reportSector=TRIAL.sectorMm(), dockSector=TRIAL.sectorMm();
        double gap=TRIAL.lowerGapMm();
        long captureLag=15,originMs,captureShift;
        boolean alignToReference;
        long exposureReadAt=-1;
        Exit departure() {
            if (!exit) return new Exit(false,0,"",0,0,false);
            String key=wrongPiece ? "other" : ReceiverIO.pieceKey(attempt);
            if (!whole) return new Exit(true,exitAttempt,key,time-earliestLag,time-latestLag,calibrated);
            if (exposureReadAt<0) exposureReadAt=time;
            return new Exit(true,exitAttempt,key,exposureReadAt-earliestLag,exposureReadAt-latestLag,calibrated,
                    new WholeFront(assembly,study,frame,clock,reportPiece,reportSector,phase,
                                   exposureReadAt-captureLag+captureShift,gap));
        }
        @Override public void beginAttempt(long id) { attempt=id; exitAttempt=id; }
        double lift=940, slide;
        boolean seat, entry, exit, jam, release, target, station=true, clear=true, stopped=true;
        boolean lower, upper;
        Demand last = Demand.neutral();
        final List<Demand> writes = new ArrayList<>();
        @Override public Inputs read() {
            return new Inputs(new Sensors(time-sensorLag,lift,slide,lower,upper,slide==0,
                slide==370,entry,seat,departure(),jam,true),
                new Approach(time-poseLag,station,target,clear,stopped,true,dockSector),release,track,originMs);
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
    void reviewedTrial() { command=new StationCycle(receiver,TRIAL);io.alignToReference=true; }
    void sourceFrame() {
        io.earliestLag=io.captureLag+TRIAL.captureSincePriorBoundMs();
        io.latestLag=io.captureLag-WithdrawalTrial.SYNC_MS;
        io.whole=true;io.exit=true;
    }
    void beforeWholeFrame() {
        io.time=io.originMs+TRIAL.captureFromLipMs()+io.captureLag-20;
        sourceFrame();
    }
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
        if (io.alignToReference) io.time=io.originMs+TRIAL.captureFromLipMs()-400;
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
        reviewedTrial(); io.release=true; ready(); tick();
        assertEquals(READY,command.state()); // A release pressed before readiness cannot start feed.
        io.release=false; tick(); io.release=true; tick();
        assertEquals(RELEASE_SEAT,command.state()); assertEquals(1.05,io.last.beltMps());
        io.seat=false; tick(); assertEquals(RELEASE_ENTRY,command.state());
        io.entry=false; tick(); assertEquals(WAIT_EXIT,command.state()); assertEquals(370,io.last.slideMm());
        io.exit=true; tick(); assertEquals(WAIT_WITHDRAW,command.state());
        assertEquals(0,io.last.beltMps()); assertEquals(0,io.last.slideVelocityMmPerS());
        beforeWholeFrame(); tick();
        assertEquals(WITHDRAW,command.state()); assertEquals(370,io.last.slideMm(),1e-8);
        assertEquals(-150,io.last.slideVelocityMmPerS());
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
        reviewedTrial(); awaitExit(); beforeWholeFrame(); tick(); io.time+=170; tick();
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
        reviewedTrial(); awaitExit(); beforeWholeFrame();
        io.earliestLag=151; tick(); assertEquals(FAULT,command.state());
    }
    @Test void frameFromFutureCannotAuthorize() {
        awaitExit(); io.latestLag=-1; io.exit=true; tick(); assertEquals(FAULT,command.state());
    }
    @Test void schedulingDelayCannotStartLateRetreat() {
        reviewedTrial(); awaitExit(); io.captureLag=41; beforeWholeFrame(); tick();
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
    @Test void legacyShelfSignalKeepsBothBeltsAndSlideStationary() {
        reviewedTrial(); awaitExit(); io.exit=true;
        for (int n=0; n<12; n++) {
            tick(); assertEquals(WAIT_WITHDRAW,command.state());
            assertEquals(370,io.last.slideMm()); assertEquals(0,io.last.slideVelocityMmPerS());
            assertEquals(0,io.last.beltMps()); assertFalse(io.last.drivePermit());
        }
    }
    @Test void completeFrameCannotEnableOrdinaryCommand() {
        io.alignToReference=true; awaitExit(); beforeWholeFrame(); tick();
        assertEquals(WAIT_WITHDRAW,command.state()); assertEquals(0,io.last.slideVelocityMmPerS());
    }
    @Test void wholeFrameRequiresMatchingScopeIdentityGeometryAndExposure() {
        List<java.util.function.Consumer<FakeIO>> mutations=List.of(
            f->f.assembly="0".repeat(64), f->f.study="f".repeat(64), f->f.frame="shelf-local",
            f->f.clock="camera-uptime", f->f.reportPiece="coral-B", f->f.track="", f->f.dockSector=220,
            f->f.reportSector=220, f->f.phase=1, f->f.gap=.99, f->f.gap=Double.POSITIVE_INFINITY,
            f->f.captureShift=-17, f->f.earliestLag-=2);
        for (var change:mutations) {
            reviewedTrial(); awaitExit(); beforeWholeFrame(); change.accept(io); tick();
            assertEquals(FAULT,command.state()); assertEquals(Demand.neutral(),io.last);
            scheduler.unregisterSubsystem(receiver);
            io=new FakeIO(); receiver=new ReceiverSubsystem(io,()->io.time);
        }
    }
    @Test void changingTrackEpochLatchesFault() {
        reviewedTrial(); carry(); io.originMs++; tick();
        assertEquals("piece track changed during attempt",receiver.fault()); assertEquals(Demand.neutral(),io.last);
    }
    @Test void withdrawalRevocationStopsMotion() {
        reviewedTrial(); awaitExit(); beforeWholeFrame(); tick(); assertEquals(WITHDRAW,command.state());
        io.gap=.5; tick(); assertEquals("whole-robot clearance revoked",receiver.fault());
        assertEquals(Demand.neutral(),io.last);
    }
    @Test void disablingDuringHoldClearsAllDemands() {
        awaitExit(); io.exit=true; tick(); assertEquals(WAIT_WITHDRAW,command.state());
        enabled(false); tick(); assertEquals(Demand.neutral(),io.last);
        enabled(true); tick(); assertFalse(command.isScheduled()); assertEquals(Demand.neutral(),io.last);
    }
    @Test void sensorPortGeometryRejectsTheOldSeatLocation() {
        double center=80, radius=57.15;
        assertTrue(Math.abs(40-center)<radius); assertTrue(Math.abs(100-center)<radius);
        assertFalse(Math.abs(-60-center)<=radius);
        // At +160 mm, both projection points clear, while the tail is still over the +165 mm shelf.
        assertTrue(160-radius>100 && 160-radius<165);
    }
}
