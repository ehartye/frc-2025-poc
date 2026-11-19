package org.reefscape.controls;

import edu.wpi.first.wpilibj2.command.Command;
import org.reefscape.controls.ReceiverIO.*;
import static org.reefscape.controls.ReceiverSubsystem.*;

/** One station-fed attempt. Cancellation/disable requires explicit recovery and a new schedule. */
public final class StationCycle extends Command {
    public enum State {
        DEPLOY, WAIT_ENTRY, WAIT_SEAT, CONFIRM, RETRACT, LOWER_CARRY, CARRY,
        LOWER_PLACE, EXTEND_PLACE, READY, RELEASE_SEAT, RELEASE_ENTRY, WAIT_EXIT, WAIT_WITHDRAW, WITHDRAW, COMPLETE, FAULT
    }
    private final ReceiverSubsystem receiver;
    private State state;
    private long enteredMs, firstSeatMs, lastTickMs, lastMechanismMs, lastChassisMs;
    private final WithdrawalTrial trial;
    private String acquiredPiece;
    private int currentSector;
    private long acquiredOriginMs;
    static final double WITHDRAW_MM_PER_S = 150;
    static final long LATEST_TESTED_WITHDRAW_MS = 150;
    private long attempt, releaseStartedMs, withdrawStartedMs;
    private Sensors previousSensors;
    private Approach previousApproach;
    private boolean previousRelease;

    public StationCycle(ReceiverSubsystem receiver) { this(receiver,null); }

    StationCycle(ReceiverSubsystem receiver, WithdrawalTrial desktopTrial) {
        this.trial=desktopTrial;
        this.receiver = receiver;
        addRequirements(receiver);
    }

    public State state() { return state; }
    private void move(State next, long now) { state = next; enteredMs = now; }
    private void fail(String reason) { state = State.FAULT; receiver.trip(reason); }

    @Override public void initialize() {
        long now = receiver.now();
        Inputs input = receiver.read();
        receiver.neutral();
        if (!receiver.fault().isEmpty()) { state = State.FAULT; return; }
        String why = invalid(input,now);
        if (!why.isEmpty()) { fail(why); return; }
        Sensors s = input.mechanism();
        if (!input.chassis().stationDocked() || !input.chassis().stopped()
            || !near(s.liftMm(),STATION_MM) || !s.slideRetracted() || s.entry() || s.seat() || s.clearOfShelf()) {
            fail("start requires docked empty receiver at stow"); return;
        }
        attempt = receiver.beginAttempt();
        acquiredPiece="";
        acquiredOriginMs=-1;
        previousSensors = s;
        previousApproach = input.chassis();
        previousRelease = input.requestRelease();
        lastTickMs = now;
        lastMechanismMs = s.timeMs();
        lastChassisMs = input.chassis().timeMs();
        move(State.DEPLOY,now);
    }

    private static boolean at(Sensors s, double lift, double slide) {
        return near(s.liftMm(),lift) && near(s.slideMm(),slide);
    }
    private static boolean atTarget(Approach a) {
        return a.targetDocked() && a.targetClear() && a.stopped();
    }
    private static boolean carrying(Sensors s) { return s.entry() && s.seat(); }
    private static long timeout(State state) {
        return switch (state) {
            case CARRY -> 30_000;
            case READY, WAIT_ENTRY -> 5_000;
            case WITHDRAW -> 3_500;
            default -> 2_500;
        };
    }

    @Override public void execute() {
        if (isFinished()) return;
        long now = receiver.now();
        Inputs input = receiver.read();
        String why = invalid(input,now);
        if (!why.isEmpty()) { fail(why); return; }
        Sensors s = input.mechanism();
        Approach a = input.chassis();
        currentSector=a.targetSectorMm();
        boolean releaseEdge = input.requestRelease() && !previousRelease;
        previousRelease = input.requestRelease();
        if (now < lastTickMs || s.timeMs() < lastMechanismMs || a.timeMs() < lastChassisMs) {
            fail("timestamp regression"); return;
        }
        if ((s.timeMs() == lastMechanismMs && !s.equals(previousSensors))
            || (a.timeMs() == lastChassisMs && !a.equals(previousApproach))) {
            fail("changed values at frozen timestamp"); return;
        }
        previousSensors = s; previousApproach = a;
        lastTickMs = now; lastMechanismMs = s.timeMs(); lastChassisMs = a.timeMs();
        String observedPiece=input.trackedPieceId();
        if (!acquiredPiece.isEmpty() &&
            (!acquiredPiece.equals(observedPiece) || acquiredOriginMs!=input.trackOriginMs())) {
            fail("piece track changed during attempt"); return;
        }
        if (s.clearOfShelf() && s.exit().attempt() != attempt) { fail("exit belongs to another attempt"); return; }
        if (now - enteredMs >= timeout(state)) { fail("timeout in " + state); return; }
        if (state != State.WAIT_EXIT && state != State.WAIT_WITHDRAW && state != State.WITHDRAW && state != State.RELEASE_ENTRY
            && s.clearOfShelf()) { fail("unexpected exit witness"); return; }
        if ((state == State.WAIT_ENTRY || state == State.WAIT_SEAT || state == State.CONFIRM)
            && !at(s,STATION_MM,RECEIVE_MM)) { fail("receiving pose lost"); return; }
        if (state == State.LOWER_CARRY && (!a.stopped() || !s.slideRetracted())) {
            fail("moving chassis or slide while lowering to carry"); return;
        }
        switch (state) {
            case DEPLOY, WAIT_ENTRY, WAIT_SEAT, CONFIRM, RETRACT -> {
                if (!a.stationDocked() || !a.stopped()) { fail("station dock lost"); return; }
            }
            case LOWER_PLACE, EXTEND_PLACE, READY, RELEASE_SEAT, RELEASE_ENTRY, WAIT_EXIT, WAIT_WITHDRAW, WITHDRAW -> {
                if (!atTarget(a)) { fail("target permit lost"); return; }
            }
            default -> {}
        }
        switch (state) {
            case RETRACT, LOWER_CARRY, CARRY, LOWER_PLACE, EXTEND_PLACE, READY -> {
                if (!carrying(s)) { fail("custody witnesses lost"); return; }
            }
            default -> {}
        }
        switch (state) {
            case DEPLOY -> {
                if (s.entry() || s.seat()) { fail("piece seen before receiver ready"); return; }
                if (at(s,STATION_MM,RECEIVE_MM)) move(State.WAIT_ENTRY,now);
            }
            case WAIT_ENTRY -> {
                if (s.seat()) { fail("seat before observed entry"); return; }
                if (s.entry()) move(State.WAIT_SEAT,now);
            }
            case WAIT_SEAT -> {
                if (!s.entry()) { fail("entry lost before seat"); return; }
                if (s.seat()) {
                    acquiredPiece=observedPiece==null ? "" : observedPiece;
                    acquiredOriginMs=input.trackOriginMs();
                    firstSeatMs = s.timeMs(); move(State.CONFIRM,now);
                }
            }
            case CONFIRM -> {
                if (!carrying(s)) { fail("seat not sustained"); return; }
                if (s.timeMs() - firstSeatMs >= 60) move(State.RETRACT,now);
            }
            case RETRACT -> {
                if (s.slideRetracted() && at(s,STATION_MM,0)) move(State.LOWER_CARRY,now);
            }
            case LOWER_CARRY -> {
                if (s.slideRetracted() && at(s,CARRY_MM,0)) move(State.CARRY,now);
            }
            case CARRY -> {
                if (!s.slideRetracted() || !at(s,CARRY_MM,0)) { fail("carry pose lost"); return; }
                if (atTarget(a)) move(State.LOWER_PLACE,now);
            }
            case LOWER_PLACE -> {
                if (!s.slideRetracted()) { fail("slide left retracted position"); return; }
                if (at(s,PLACE_MM,0)) move(State.EXTEND_PLACE,now);
            }
            case EXTEND_PLACE -> {
                if (s.slideExtended() && at(s,PLACE_MM,DEPLOY_MM)) move(State.READY,now);
            }
            case READY, RELEASE_SEAT, RELEASE_ENTRY -> {
                if (!s.slideExtended() || !at(s,PLACE_MM,DEPLOY_MM)) { fail("release pose lost"); return; }
                if (state == State.READY && releaseEdge) {
                    releaseStartedMs = now;
                    move(State.RELEASE_SEAT,now);
                }
                else if (state == State.RELEASE_SEAT) {
                    if (!s.entry()) { fail("exit before observed seat clearance"); return; }
                    if (!s.seat()) move(State.RELEASE_ENTRY,now);
                } else if (state == State.RELEASE_ENTRY) {
                    if (s.seat()) { fail("seat reoccupied during release"); return; }
                    if (s.clearOfShelf() && s.entry()) { fail("exit witness disagrees with entry"); return; }
                    if (!s.entry()) {
                        if (s.clearOfShelf()) assessDeparture(s.exit(),now);
                        else move(State.WAIT_EXIT,now);
                    }
                }
            }
            case WAIT_EXIT -> {
                if (s.entry() || s.seat() || !at(s,PLACE_MM,DEPLOY_MM) || !s.slideExtended()) {
                    fail("unexpected contact or pose while awaiting exit"); return;
                }
                if (s.clearOfShelf()) assessDeparture(s.exit(),now);
            }
            case WAIT_WITHDRAW -> {
                if (s.entry() || s.seat() || !s.slideExtended() || !at(s,PLACE_MM,DEPLOY_MM)) {
                    fail("unexpected contact or pose after exit"); return;
                }
                if (s.clearOfShelf() && s.exit().wholeFront()!=null) assessDeparture(s.exit(),now);
            }
            case WITHDRAW -> {
                WholeFront observation=s.exit()==null ? null : s.exit().wholeFront();
                if (currentSector!=trial.sectorMm() || (observation!=null &&
                    (!s.exit().clear() || !s.exit().calibrated() || !trial.sameTrack(observation,acquiredPiece)
                     || !Double.isFinite(observation.lowerGapMm()) || observation.lowerGapMm()<1))) {
                    fail("whole-robot clearance revoked"); return;
                }
                if (!near(s.liftMm(),PLACE_MM) || Math.abs(s.slideMm()-withdrawGoal(now)) > 15) {
                    fail("withdrawal tracking disagreement"); return;
                }
                if (s.entry() || s.seat()) { fail("piece seen during withdrawal"); return; }
                if (withdrawGoal(now) == 0 && s.slideRetracted() && at(s,PLACE_MM,0)) move(State.COMPLETE,now);
            }
            default -> {}
        }
        Demand demand = switch (state) {
            case DEPLOY, CONFIRM -> new Demand(true,STATION_MM,RECEIVE_MM,0,false);
            case WAIT_ENTRY, WAIT_SEAT -> new Demand(true,STATION_MM,RECEIVE_MM,-0.2,false);
            case RETRACT -> new Demand(true,STATION_MM,0,0,false);
            case LOWER_CARRY, CARRY -> new Demand(true,CARRY_MM,0,0,state == State.CARRY);
            case LOWER_PLACE -> new Demand(true,PLACE_MM,0,0,false);
            case WAIT_WITHDRAW -> new Demand(true,PLACE_MM,DEPLOY_MM,0,false);
            case WITHDRAW -> new Demand(true,PLACE_MM,withdrawGoal(now),0,false,
                    withdrawGoal(now)>0 ? -WITHDRAW_MM_PER_S : 0);
            case EXTEND_PLACE, READY -> new Demand(true,PLACE_MM,DEPLOY_MM,0,false);
            case RELEASE_SEAT, RELEASE_ENTRY, WAIT_EXIT -> new Demand(true,PLACE_MM,DEPLOY_MM,1.05,false);
            case COMPLETE, FAULT -> Demand.neutral();
        };
        if (demand.active() && ((s.liftLower() && demand.liftMm() < s.liftMm()-TOLERANCE_MM)
            || (s.liftUpper() && demand.liftMm() > s.liftMm()+TOLERANCE_MM))) {
            fail("lift limit blocks requested travel"); return;
        }
        receiver.output(demand);
    }

    private void assessDeparture(Exit exit,long now) {
        if (!exit.calibrated() || !ReceiverIO.pieceKey(attempt).equals(exit.pieceKey())
            || exit.earliestCrossingMs() < releaseStartedMs || exit.latestCrossingMs() < exit.earliestCrossingMs()
            || exit.latestCrossingMs() > now || now-exit.earliestCrossingMs() > LATEST_TESTED_WITHDRAW_MS) {
            fail("uncertain or late independent clearance"); return;
        }
        if (trial==null || exit.wholeFront()==null) {
            if (state!=State.WAIT_WITHDRAW) move(State.WAIT_WITHDRAW,now);
            return;
        }
        if (!trial.permits(exit,acquiredPiece,acquiredOriginMs,currentSector,now)) { fail("unqualified whole-robot observation"); return; }
        withdrawStartedMs=now;
        move(State.WITHDRAW,now);
    }

    private double withdrawGoal(long now) {
        double elapsedSeconds = Math.max(0,now-withdrawStartedMs)/1000.0;
        return Math.max(0,DEPLOY_MM-WITHDRAW_MM_PER_S*elapsedSeconds);
    }

    @Override public boolean isFinished() { return state == State.COMPLETE || state == State.FAULT; }
    @Override public void end(boolean interrupted) {
        if (interrupted) fail("interrupted or disabled");
        receiver.neutral();
    }
}
