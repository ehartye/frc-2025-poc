package org.reefscape.controls;

import java.io.BufferedReader;
import java.io.InputStreamReader;

/** Pipe only public sensor samples into the same Java state machine used by the scheduler. */
public final class PortConsole {
    public static void main(String[] args) throws Exception {
        var input = new BufferedReader(new InputStreamReader(System.in));
        var controller = new PortAcquisition(true);
        String line;
        while ((line = input.readLine()) != null) {
            if (line.equals("COUNTERFEED") || line.equals("STATIONARY")) {
                controller = new PortAcquisition(line.equals("COUNTERFEED"));
                continue;
            }
            String[] v = line.split(",");
            var out = controller.update(Long.parseLong(v[0]), Long.parseLong(v[1]), bit(v[2]), bit(v[3]),
                    bit(v[4]), bit(v[5]), bit(v[6]), bit(v[7]));
            System.out.println(out.state() + "," + out.active() + "," + out.beltMps() + "," + out.reason());
        }
    }
    private static boolean bit(String value) { return value.equals("1"); }
}
