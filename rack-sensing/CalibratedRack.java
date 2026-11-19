package org.reefscape.controls;

/** Desktop conversion for the selected 370 mm rack and its separate direct sensor. */
final class CalibratedRack {
    static final double TRAVEL_M = 0.370;
    static final double SENSOR_RANGE_M = 0.635;
    static final int ADC_STEPS = 4096;
    private final int extended, stowed;

    CalibratedRack(int extended, int stowed) {
        this.extended = extended;
        this.stowed = stowed;
        double fullSpan = Math.abs(stowed - extended) * SENSOR_RANGE_M / TRAVEL_M;
        double rounding = SENSOR_RANGE_M / TRAVEL_M;
        if (!usable(extended) || !usable(stowed) || fullSpan < 0.90 * ADC_STEPS - rounding
                || fullSpan > 0.98 * ADC_STEPS + rounding)
            throw new IllegalArgumentException("Unusable endpoint calibration");
    }

    private static boolean usable(int count) { return count > 0 && count < ADC_STEPS - 1; }

    double metres(int rawCount) {
        if (!usable(rawCount)) return Double.NaN;
        return (rawCount - extended) * TRAVEL_M / (stowed - extended);
    }

    double motionThreshold(double repeatabilityFraction) {
        if (!Double.isFinite(repeatabilityFraction) || repeatabilityFraction < 0)
            throw new IllegalArgumentException("Invalid repeatability");
        int span = Math.abs(stowed - extended);
        // One count covers the two rounded readings. An extra count on the
        // calibrated span covers opposite rounding errors at its reference stops.
        double smallestQuantum = TRAVEL_M / (span + 1);
        double differentialError = 2 * repeatabilityFraction * SENSOR_RANGE_M;
        int counts = 1 + (int)Math.ceil((0.00020 + differentialError) / smallestQuantum);
        return (counts - 0.5) * TRAVEL_M / span; // integer-code comparison without a boundary tie
    }
}
