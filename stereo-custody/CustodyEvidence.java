package org.reefscape.controls;

import java.util.ArrayDeque;
import java.util.List;

/** Independent desktop position witness. No actuator, drive or lift authority. */
public final class CustodyEvidence {
    public record Frame(long sequence, long captureMs, long arrivedMs, String epoch,
                        String fit, double minU, double maxU, double minY, double maxY,
                        double minBottom, double maxBottom, String poseSha256) {
        boolean fits() {
            return fit.equals("position-witness") && 55 <= minU && minU <= maxU && maxU <= 100
                && -8 <= minY && minY <= maxY && maxY <= 8
                && -12 <= minBottom && minBottom <= maxBottom && maxBottom <= 12;
        }
    }
    private static final String RECEIVE_SHA = "6389d766ad3d4d65b885bccb615a23807cadd8ef4f14d846214cea8a4dd96f73";
    private final ArrayDeque<Long> exposures = new ArrayDeque<>();
    private long lastTick = -1, sequence = -1, lastCapture = -1, bandStart = -1;

    public boolean update(long now, boolean inBand, boolean calibrated, boolean stationary,
                          String epoch, List<Frame> received) {
        if (lastTick >= 0 && now - lastTick != 20) { exposures.clear(); bandStart = -1; }
        lastTick = now;
        if (!calibrated || !stationary) { exposures.clear(); bandStart = -1; return false; }
        if (!inBand) bandStart = -1;
        else if (bandStart < 0) bandStart = now;
        for (Frame frame : received) {
            if (!RECEIVE_SHA.equals(frame.poseSha256()) || !epoch.equals(frame.epoch())
                || frame.sequence() <= sequence || frame.captureMs() <= lastCapture
                || frame.captureMs() > frame.arrivedMs() || frame.arrivedMs() > now || now - frame.captureMs() > 40) {
                exposures.clear();
                return false;
            }
            if (lastCapture >= 0 && frame.captureMs() - lastCapture > 21) exposures.clear();
            lastCapture = frame.captureMs();
            sequence = frame.sequence();
            if (inBand && frame.captureMs() >= bandStart && frame.fits()) exposures.addLast(lastCapture);
            else exposures.clear();
        }
        if (!inBand || now - lastCapture > 40) exposures.clear();
        while (!exposures.isEmpty() && exposures.getFirst() < now - 160) exposures.removeFirst();
        return exposures.size() >= 5 && exposures.getLast() - exposures.getFirst() >= 68;
    }
}
