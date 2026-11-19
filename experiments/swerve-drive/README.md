# Four-module swerve for the 2025 robot

<!-- cad-still -->
[![Team 1780 · stowed swerve robot](../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · stowed swerve robot.** Red alliance R4 with numbered bumper covers. Purchased parts use project-created representations in this public assembly. [Source STEP · abbd8ab838](../../cad/full-swerve-r4/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

[Replay the opening match drive](motion/index.html) or [open the student Fusion design](../../cad/FUSION.md). The same fixed-height receiver approaches the red station using independently steered wheels and the two upper holes in its station panel. Across four simulated combinations of calibration, operating mass and yaw inertia, the robot reaches the 21.10 s feed slot with 0.770–0.905 s to spare.

| What students can investigate | Evidence |
| --- | --- |
| Watch the four wheels rotate and reverse while the robot holds its heading; try a jammed corner or a delayed camera. | [Interactive field replay](motion/index.html) and [eight controller trials](motion/README.md) |
| Inspect wheel centers, rear-module interference, steering sweep, frame cuts and attachment hardware. | [Native packaging](packaging/) and [Fusion model and supplier revision](../../cad/FUSION.md) |
| Compare a fast turn with a gradual turn on the same four tires. Review motor demand, loaded battery voltage, and wiring allocations. | [Power, traction and mass](power/README.md) |
| Build a four-corner conductor cut list and see which retired parts change inspection weight. The repaired upper is 51.993 kg, leaving 170 g under R103. | [Drawing, stock and inspection mass](mass-closure/README.md) |
| Follow the first identified pipe from its timed release into the retained two-stage receiver. | [Source-bound feed receipts](motion/first-feed.json) and [desktop port-controller replay](motion/feed.py) |

The earlier abrupt turn required a tire coefficient of 0.836 in one supported mass and stiffness case. Slowing for the waypoint and the final 35 mm gives a physical 65 kg opening with peak coefficient near 0.171. Both calibration signs and a higher 72 kg / 8 kg·m² comparison passed the independently varied pack and drivetrain demand checks.

The 65 kg first-feed trials reach the station program’s seated range at wall 22.912 s. Both signs survive the 31.25 and 15.625 microsecond contact refinements through 27.032 s. At the first wheel update at 27.035 s, five timely observations from both camera heads confirm grip on the guarded finish. Matching the shelf and pipe finishes hides the rear edge. A 45 ms delay leaves only three observations in time. Both cases hold the robot at the station. These trials use the earlier opening and stationary chassis. The corrected drive needs fresh pickup and loaded travel evidence on its attained poses. [Camera timing chart](../../docs/robot/station-witness.png) · [Signed drive comparison](motion/energy.json)

For the first chassis and mount package the conservative inspection upper is 55.813 kg for long wiring or 54.792 kg for shorter wiring. They exceed the source-pinned 52.163 kg limit by 3.650 and 2.629 kg. The longer field-equipped mass upper with bumpers, battery and one pipe is 70.300 kg. The design is an offline candidate with an open inspection-weight decision. [Installed mass reconciliation](power/installed.json)

The first chassis fit exposed two rear-motor intersections in the retained electronics plates. Machined edge pockets and an elevated host bracket clear the rear motors by at least 1.477 mm on the left and 1.292 mm on the right in the revised geometry. The 30 revised rear hardware and connector envelopes stay at least 594.5 mm below the modeled station sightlines and 79 mm inside the bumper. With 0.1571 kg charged for the revision and no weight credit for removed material, the longer-wire inspection upper is 55.970 kg and the field-equipped one-pipe upper is 70.457 kg. [Rear hardware](packaging/rear-electronics.json) · [Revised camera and bumper check](motion/rear-visibility.json)
