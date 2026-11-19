package org.reefscape.controls;

/** Quantized, ratiometric source and explicit faults for the desktop plant. */
final class RackSignalTrial {
    private final double span, repeat, repeatShift, nonlinearity, offset;
    private final boolean reversed, unscaled, stuck, rail, legacyGate;
    final CalibratedRack calibration;

    RackSignalTrial(String mode) {
        span = mode.contains("90") ? .90 : mode.contains("98") ? .98 : .94;
        offset = mode.contains("phase") ? .0100 : .0101;
        repeat = mode.contains("modern") ? .0002 : .0005;
        repeatShift = mode.contains("adverse") ? -1 : mode.contains("assist") || mode.contains("slip") ? 1 : 0;
        reversed = mode.contains("reverse");
        unscaled = mode.contains("unscaled");
        stuck = mode.contains("slip");
        rail = mode.contains("open");
        legacyGate = mode.contains("legacy");
        nonlinearity = mode.contains("curvature") ? .0025 * CalibratedRack.SENSOR_RANGE_M : 0;
        calibration = new CalibratedRack(code(0), code(CalibratedRack.TRAVEL_M));
    }

    private int code(double displacement) {
        // The bracketed drawing puts the resting eye center 23.9 mm past the guide.
        double ratio = offset + span * (.419 - .0239 - displacement) / CalibratedRack.SENSOR_RANGE_M;
        if (reversed) ratio = 1 - ratio;
        return Math.max(0, Math.min(4095, (int)Math.round(ratio * CalibratedRack.ADC_STEPS)));
    }

    double sensor(double travel, long elapsed) {
        double x = stuck ? 0 : travel;
        // At the counterexample endpoints both readings fall within one repeatability band.
        double shift = repeatShift * repeat * CalibratedRack.SENSOR_RANGE_M * (elapsed > 0 ? 1 : -1);
        double curvature = nonlinearity * Math.sin(Math.PI * x / CalibratedRack.TRAVEL_M);
        int raw = rail && elapsed >= 5 ? 0 : code(x + shift + curvature);
        if (unscaled) return (code(0) - raw) * CalibratedRack.SENSOR_RANGE_M / CalibratedRack.ADC_STEPS;
        return calibration.metres(raw);
    }

    double threshold() {
        return legacyGate ? 4 * .635 / 4096 : calibration.motionThreshold(repeat);
    }

    boolean decoupled() { return stuck; }
}
