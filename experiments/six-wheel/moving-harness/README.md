# Route the moving camera cables

<!-- cad-still -->
[![Four-module swerve frame and mounting hardware](../../../docs/robot/cad/assembly.png)](../../../docs/robot/cad/index.html)

**Project CAD · Four-module swerve frame and mounting hardware.** Original robot structure for the swerve candidate. The project-local Fusion assembly adds four manufacturer module occurrences. [Source STEP · f8b75f2df4](../../swerve-drive/packaging/swerve-frame.step) · [Rotate the model](../../../docs/robot/cad/index.html)
<!-- /cad-still -->

The existing anchor sequence needs at least **1,482.27 mm** at full extension even with zero cable thickness, arbitrary entry points within every anchor and perfectly sharp turns. Only 17.73 mm of the prior 1.50 m estimate remains for real bends, plugs and service slack. A right-side rolling return provides a different route. [Move the shelf through its stroke](explore.html) · [Existing anchor calculation](anchors.json)

| From the fixed equipment to the moving handoff | Power | USB 3 data |
|---|---:|---:|
| Fixed feed including formed corners | 620.19 mm | 839.16 mm |
| Managed length through the carrier | 701.50 mm | 701.50 mm |
| Available tail, transitions and terminations within the proposed cut budget | 578.31 mm | 559.34 mm |
| Conservative fixed-feed clearance to neighboring robot parts | 8.97 mm | 6.50 mm |
| One-way cut budget | 1.90 m | 2.10 m |

The handoff stays at X = −72.5 mm plus shelf extension, Z = 800 mm, with strands departing forward. Y is −329 mm for power and −317 mm for data. The fixed-side start points represent allocations on the converter top and host envelope. Actual USB exit direction, plug-to-datum transitions, retention, mating and service slack must fit within the quoted tails. Disconnect both services and independently support the loom before extracting the camera bridge. [Removable bridge drawings and extraction path](../camera-datum-shop/README.md)

## Continuous travel

The igus Series 10 carrier (width code 025, radius code 100, standard color 0) has a 25 mm internal width, 36 mm external width and 30.5 mm pitch. The 370 mm stroke, 117.5 mm off-center fixed end, and 380 mm catalog bend allowance require 682.5 mm before rounding up to **23 links**. The ideal return moves at half the shelf speed and shares the extra length between the two straight runs. A 20 mm upper camber allowance and the catalog overlength go into the carrier clearance box. A loaded carrier or mount that leaves the modeled envelope needs another check. The 0.281 kg catalog chain mass excludes brackets, separator and cables. [Series 10 dimensions and load chart](https://us.rs-online.com/m/d/9b62d5ba055cd52e1fd1983cbf74209d.pdf) · [Off-center calculation](https://www.igus.com/cable-carriers/resources/e-chain-designing-calculation)

The CFBUS.PUR.068 cable has a 7 mm maximum OD and a rated 87.5 mm moving bend radius. The fixed feed uses 50 mm bends. The LAPP 1028952 two-core 2.5 mm² power cable has a nominal 8.61 mm OD and a 64.5 mm dynamic minimum radius. Its fixed feed uses 36 mm bends. The screen reserves an 8.70 mm jacket. Check actual diameter and install a separator and end retention compatible with the two cables. [USB construction and dimensions](https://media.digikey.com/pdf/Data%20Sheets/Igus%20PDFs/CFBUS.PUR.pdf) · [Power cable specification](https://api-shop.lapp.com/occ/v2/us/products/export/specification-sheet-file/1028952?curr=USD&lang=en_US)

The 603-body shop STEP and the registered frame motion bound the entire 370 mm travel, including rotating gears and the variable-length draw wire. The carrier box clears the backup return line by **11.41 mm** or more. The fixed-feed calculation also checks the three added converter envelopes. Straight and curved centerlines use at most 2 mm arclength per sample, then subtract the jacket radius and another 1 mm to obtain continuous lower bounds. The two fixed feeds retain at least 2.15 mm between jackets under this screen. A real 8.7 mm cylinder on the inboard route at Y = −282 mm intersects the native service-pin rail by **713.36 mm³** at pickup. [CAD results](geometry.json)

At the actual station and both reef sectors, the additions retain positive clearance against the registered field. The bounds include a 35 mm chassis translation disk, ±2° heading, ±15 mm rack endpoint error and 3 mm fixture registration. The station wall leaves **7.36 mm**. The closest occupied reef ball leaves **19.21 mm at sector B**. These are bounds on the proposed carrier and fixed feeds at the docks. They do not establish that the receiver can capture or score a pipe at those errors. [Native field results](field.json)

## Electrical and mass acceptance

The installed Agent FRC 0.22.0 review gives **5.00677 V** at 1.20 A for the screened lower bound, using the power study's 10 mV instrumentation allowance, hot copper rate and 55 mΩ connector/fuse allowance. Only **5.64 mΩ** extra loop resistance remains. Adding 50 mΩ drops the guarantee to **4.94677 V**. Actual camera reset or uninterrupted operation requires a functional observation. The adapter's short narrower leads, power injection, connections, completed hot copper loop and the actual 5.13 V minimum regulator setting must satisfy this same budget. [Installed verdicts](power/review.json) · [Inputs and resistance budget](power/budget.json) · [Power panel](../fixed-low-power/dual-camera.md)

The shop estimate left 1.577 kg of represented inspection margin. A conservative gross screen carries the complete 0.750 kg converter retrofit, 0.281 kg carrier and 0.134 kg for 2.10 m of USB cable. That leaves roughly **0.412 kg** for outstanding articles before reconciling the old allowances. Weigh and reconcile power cable, brackets, guide, connectors and slack once in the selected assembly. Separately qualify the fixed carrier support, moving end attachment, full-load flex and drag, shelf withdrawal time, and failure behavior with a missing camera stream. Clearance and this static electrical budget alone do not qualify the mechanism for a match. [Shop mass assumptions](../camera-datum-shop/shop-mass.json)

## Reproduce

Use the project's local artifact directory containing the previously exported whole-robot STEP exchanges. The final command takes a staged or installed Agent FRC plugin root as its first argument. The study runs offline.

```text
python moving-harness/route_screen.py artifacts moving-harness/anchors.json
python moving-harness/continuous_screen.py artifacts moving-harness/geometry.json
python moving-harness/field_screen.py <registered-field-step> moving-harness/field.json
python moving-harness/power_budget.py <agent-frc-root> moving-harness/power
```
