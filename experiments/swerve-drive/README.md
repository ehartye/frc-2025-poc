# Four-module swerve for the 2025 robot

<!-- cad-still -->
[![Four-module swerve frame and mounting hardware](../../docs/robot/cad/assembly.png)](../../docs/robot/cad/index.html)

**Project CAD · Four-module swerve frame and mounting hardware.** Original robot structure for the swerve candidate. The project-local Fusion assembly adds four manufacturer module occurrences. [Source STEP · f8b75f2df4](packaging/swerve-frame.step) · [Rotate the model](../../docs/robot/cad/index.html)
<!-- /cad-still -->

[Replay the opening match drive](motion/index.html) or [open the student Fusion design](../../cad/FUSION.md). The same fixed-height receiver approaches the red station using independently steered wheels and the two upper holes in its station panel. Across four simulated combinations of calibration, operating mass and yaw inertia, the robot reaches the 21.10 s feed slot with 0.770–0.905 s to spare.

| What students can investigate | Evidence |
| --- | --- |
| Watch the four wheels rotate and reverse while the robot holds its heading; try a jammed corner or a delayed camera. | [Interactive field replay](motion/index.html) and [eight controller trials](motion/README.md) |
| Inspect wheel centers, rear-module interference, steering sweep, frame cuts and attachment hardware. | [Native packaging](packaging/) and [Fusion model and supplier revision](../../cad/FUSION.md) |
| Compare a fast turn with a gradual turn on the same four tires. Review motor demand, loaded battery voltage, and wiring allocations. | [Power, traction and mass](power/README.md) |
| Follow the first identified pipe from its timed release into the retained two-stage receiver. | [Source-bound feed receipts](motion/first-feed.json) and [desktop port-controller replay](motion/feed.py) |

The earlier abrupt turn required a tire coefficient of 0.836 in one supported mass and stiffness case. Slowing for the waypoint and the final 35 mm gives a physical 65 kg opening with peak coefficient near 0.171. Both calibration signs and a higher 72 kg / 8 kg·m² comparison passed the independently varied pack and drivetrain demand checks.

The 65 kg feed trials reach the station program’s seated range at wall 22.912 s. Both signs survive the 31.25 and 15.625 microsecond contact refinements through 27.032 s. That result uses a stationary chassis and retained guide and shelf boundaries. Mobile transport, independently imaged pipe custody, reef placement and match points still require evidence on the swerve assembly.

For the first chassis and mount package the conservative inspection upper is 55.813 kg for long wiring or 54.792 kg for shorter wiring. They exceed the source-pinned 52.163 kg limit by 3.650 and 2.629 kg. The longer field-equipped mass upper with bumpers, battery and one pipe is 70.300 kg. The design is an offline candidate with an open inspection-weight decision. [Installed mass reconciliation](power/installed.json)

The first chassis fit exposed two rear-motor intersections in the retained electronics plates. Machined edge pockets and an elevated host bracket clear the rear motors by at least 1.477 mm on the left and 1.292 mm on the right in the revised geometry. The 30 revised rear hardware and connector envelopes stay at least 594.5 mm below the modeled station sightlines and 79 mm inside the bumper. With 0.1571 kg charged for the revision and no weight credit for removed material, the longer-wire inspection upper is 55.970 kg and the field-equipped one-pipe upper is 70.457 kg. [Rear hardware](packaging/rear-electronics.json) · [Revised camera and bumper check](motion/rear-visibility.json)
