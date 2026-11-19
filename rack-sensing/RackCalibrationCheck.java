package org.reefscape.controls;

import static org.junit.jupiter.api.Assertions.*;

final class RackCalibrationCheck {
    private static int code(double gain, double offset, double displacement, int direction) {
        return (int)Math.round(offset + direction * gain * 4096 * displacement / .635);
    }

    static void verify() {
        assertThrows(IllegalArgumentException.class, () -> new CalibratedRack(0, 2450));
        assertThrows(IllegalArgumentException.class, () -> new CalibratedRack(4095, 2000));
        assertThrows(IllegalArgumentException.class, () -> new CalibratedRack(800, 800));
        assertThrows(IllegalArgumentException.class, () -> new CalibratedRack(800, 2500));
        for (int direction : new int[]{-1, 1}) {
            // Keep both ADC endpoints strictly interior for either electrical polarity.
            int start = direction < 0 ? 2700 : 1000;
            for (int gainStep = 90; gainStep <= 98; gainStep++)
                for (int phase = 0; phase < 64; phase++) {
                    double gain = gainStep / 100.0, offset = start + phase / 64.0;
                    int zero = code(gain, offset, 0, direction);
                    int full = code(gain, offset, .370, direction);
                    var cal = new CalibratedRack(zero, full);
                    assertEquals(0, cal.metres(zero), 0);
                    assertEquals(.370, cal.metres(full), 1e-12);
                    assertTrue(Double.isNaN(cal.metres(0)));
                    assertTrue(Double.isNaN(cal.metres(4095)));
                    for (double fraction : new double[]{.0002, .0005}) {
                        double repeat = fraction * .635;
                        double prior = cal.metres(code(gain, offset, -repeat, direction));
                        // Greatest apparent positive motion with the true slide just 0.20 mm forward.
                        double apparent = cal.metres(code(gain, offset, .00020 + repeat, direction)) - prior;
                        assertTrue(apparent < cal.motionThreshold(fraction));
                    }
                }
        }
    }
}
