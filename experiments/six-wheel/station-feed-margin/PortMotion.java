package org.reefscape.controls;

import java.io.BufferedReader;
import java.io.InputStreamReader;

/** Desktop-only counterfactual following a measured entry, then two dark ports. */
public final class PortMotion {
    public static void main(String[] args) throws Exception {
        double challengedDemand = Double.parseDouble(args[0]);
        var input = new BufferedReader(new InputStreamReader(System.in));
        var controller = new PortAcquisition(true);
        boolean firstEntry = false;
        boolean latched = false;
        String line;
        while ((line = input.readLine()) != null) {
            if (line.equals("COUNTERFEED")) {
                controller = new PortAcquisition(true);
                firstEntry = false;
                latched = false;
                continue;
            }
            String[] v = line.split(",");
            boolean entry = v[6].equals("1");
            boolean seat = v[7].equals("1");
            var out = controller.update(Long.parseLong(v[0]), Long.parseLong(v[1]), v[2].equals("1"),
                    v[3].equals("1"), v[4].equals("1"), v[5].equals("1"), entry, seat);
            if (firstEntry && !entry && !seat) latched = true;
            firstEntry = entry && !seat;
            boolean challenge = latched && out.active();
            double demand = challenge ? challengedDemand : out.beltMps();
            System.out.println(out.state() + "," + out.active() + "," + demand + "," + out.reason());
        }
    }
}
