package org.reefscape.controls;

import edu.wpi.first.wpilibj.RobotBase;
import org.reefscape.controls.ReceiverIO.Exit;
import org.reefscape.controls.ReceiverIO.WholeFront;

/** A single reviewed simulated image phase on one verified whole-robot trajectory.
 * The ordinary StationCycle constructor has no trial. No vehicle adapter constructs one. */
record WithdrawalTrial(String assemblySha256,String studySha256,String pieceId,int sectorMm,
                       int phase,long captureSincePriorBoundMs,double lowerGapMm,long captureFromLipMs,
                       double coastFriction,double pixelError,String fieldSha256) {
    static final long SYNC_MS=1, PROCESS_MS=8, SCHEDULER_MS=20, ACTUATION_MS=10;
    static final String FRAME="assembled-robot-front-537mm";
    static final String CLOCK="controller-monotonic-ms";

    WithdrawalTrial {
        if (assemblySha256==null || !assemblySha256.matches("[a-f0-9]{64}") ||
            studySha256==null || !studySha256.matches("[a-f0-9]{64}") ||
            fieldSha256==null || !fieldSha256.matches("[a-f0-9]{64}") ||
            !Double.isFinite(coastFriction) || coastFriction<0 || !Double.isFinite(pixelError) || pixelError<=0 ||
            pieceId==null || pieceId.isBlank() || phase<0 || phase>=17 || captureSincePriorBoundMs<=0 ||
            !Double.isFinite(lowerGapMm) || lowerGapMm<1 || captureFromLipMs<=0 || captureFromLipMs%17!=phase) {
            throw new IllegalArgumentException("incomplete reviewed trial");
        }
    }

    boolean sameTrack(WholeFront w,String acquiredPiece) {
        return pieceId.equals(acquiredPiece) && pieceId.equals(w.pieceId()) &&
            assemblySha256.equals(w.assemblySha256()) && studySha256.equals(w.studySha256()) &&
            FRAME.equals(w.frame()) && CLOCK.equals(w.clock()) && sectorMm==w.sectorMm() &&
            Double.compare(coastFriction,w.coastFriction())==0 && Double.compare(pixelError,w.pixelError())==0 &&
            fieldSha256.equals(w.fieldSha256());
    }

    boolean permits(Exit exit,String acquiredPiece,long trackOriginMs,int dockSector,long now) {
        WholeFront w=exit.wholeFront();
        if (!RobotBase.isSimulation() || w==null || !sameTrack(w,acquiredPiece) ||
            dockSector!=sectorMm || Double.compare(w.lowerGapMm(),lowerGapMm)!=0 || phase!=w.exposurePhase() ||
            w.captureMs()<exit.earliestCrossingMs() || w.captureMs()>=now) return false;
        long elapsed=now-w.captureMs();
        return trackOriginMs>=0 && trackOriginMs<=w.captureMs() && w.captureMs()-trackOriginMs==captureFromLipMs &&
            elapsed>=PROCESS_MS-SYNC_MS && elapsed<=PROCESS_MS+SYNC_MS+SCHEDULER_MS &&
            exit.latestCrossingMs()-w.captureMs()==SYNC_MS &&
            w.captureMs()-exit.earliestCrossingMs()==captureSincePriorBoundMs &&
            now-exit.earliestCrossingMs()<=StationCycle.LATEST_TESTED_WITHDRAW_MS-ACTUATION_MS;
    }
}
