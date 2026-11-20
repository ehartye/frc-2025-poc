# Can the two station-fed pipes still score?

<!-- cad-still -->
[![Team 1780 · captive swerve receiver](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · captive swerve receiver.** The public Fusion capture candidate at rest, with 1,001 robot solids and the hollow practice CORAL. Project-drawn representations occupy the supplier interfaces. [Source STEP · 0dad9e862e](../../../cad/swerve-captive/public-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

[Scrub the source-to-target timeline](replay.html) to see where each pipe is at a given TELEOP time and try alternate stock, partner and spin conditions. The nominal simulation retains the nine-point opportunity. The first pipe stays supported at 32.06 s, the second at 82.15 s, and the robot parks at 94.02 s. Both stay in the trough through the 138 s assessment. Three points for LEAVE, four for the two TELEOP trough placements, and two for PARK depend on the assumed robot, partners, source and controls. The official points field is empty. [Full-field map and staging](../full-match/README.md) · [Edition-specific scoring observations](../scoring/README.md)

| Red 2 program | Two-second trough support in TELEOP | Finish | Conditional opportunity |
| --- | --- | --- | --- |
| Partner clears and red 2 parks | A 32.06 s, B 82.15 s | 94.02 s | 9 |
| Partner holds its starting spot | Individual pipe traces are unchanged | Collision at 89.39 s | Whole match withheld |
| Return to station after B | A 32.06 s, B 82.15 s | 100.10 s | 7 |
| B stock first available at 111 s | A 32.06 s | 52.40 s | 5 with no park on this program |

The [coupled spin and belt study](../axial-spin/README.md) now carries the inbound angular momentum to the reef. It identifies a candidate 0.90 m/s belt limit and failures at both 0.75 and 1.05 m/s.

## What carries from handoff to target

The centered, transverse station release uses the existing 10 mm bumper stand-off and 240 mm receiver stroke. On the 545-body robot the acquired pipe leaves the chute at 1.697 m/s, meets the receiver 49.81 ms later, and reaches the first retract command 1.10 s after that. A and B keep separate stock identities and absolute lip times of 3.672 and 54.073 s. Each seated center enters the existing 1 ms mission integrator at station-frame (0.124580, 0, 0.937150) m, with horizontal velocity (−0.000496, 0) m/s and yaw π. Its vertical velocity and pitch are zero under the shelf support model. Neither seating nor removal is inferred from a fresh object at the target. [Feeder and first contact](../station-supply/README.md) · [545-body pickup replay](source-545.json) · [545-body complete mission](shop-545.json) · [401-body comparison](reference-401.json)

At the target, eight sightlines from the two modeled eyes cover the center and top of each end rim. Both robot revisions retain all 17 tight zero-added-wait phases on the issued path. The worst tight-pixel withdrawal onset is 110 ms after shelf clearance. In 116 trials, both pipes sustained two seconds of support at every integer-millisecond onset from 62 through 119 ms in the combined pixel budgets. The smallest gap to the retracting shelf in the post-contact runs was 41.39 mm. [Onset sensitivity](onset-sweep.json)

Actual XCAF solids were checked against pipe-wall material from the shelf witness to the end of the possible contact interval, at nominal pose and the forward stroke / inward cartridge extreme. The later swept envelopes include timing and travel error. Adding the old 50 ms hold leaves zero eligible phases. A 600 ms delayed withdrawal brings the pipe back into contact with the receiver. Synthetic image extraction, motor response and the new source identities still need an independent controls qualification. [Source-bound controls](../competition-controls/README.md)

During each 1 ms seated-to-first-target-contact trajectory, the finite pipe is enclosed by a 163.86 mm radius sphere, including a 2 mm allowance at each axial end and in radius. This bounds every orientation. Across the 24 fixed pieces in the January field, the closest lower bounds are 33.45 mm for A and 52.66 mm for B, both to reef ALGAE NAUO201. During trip B the lower bound to previously supported A is 78.16 mm. The roster and loaded trajectories are checked separately from the two-second trough contact dynamics. [Carried-pipe clearance](carried-stock.json) · [Installed field geometry](../full-match/installed-stock.json)

## What can change the outcome

A centered, perfectly inelastic initial landing and enough friction to roll supplied the two seats. In the station study, restitution of 0.20, 0.40 or 0.60 produced native guard contact within 35, 20 or 15 ms. Those failures have no downstream placement. Center tracking, human introduction and the loaded 67 kg drivetrain schedule remain assumptions. The 10 mm docking clearance has little tolerance for angular error.

The shelf integrator resolves translation and yaw. Axial spin after the feeder seats the pipe is unmeasured. The trough contact model therefore compares signed axial speeds at first contact with the target. Both pipes sustain two seconds of support at 0, +5, +10 and +15 rad/s. At −5 rad/s both reach the inner-riser bound. At +20 rad/s both reach the receiver. The nominal chute exit implies a rolling speed magnitude of 29.69 rad/s. Its sign and magnitude at the reef remain open. These are tested discrete values; a physical scoring claim needs measured spin or a contact design that tolerates its measured range. [Spin sweep](spin-screen.json)

## Reproduce

Run from the project root with its existing OpenCascade and NumPy environment. The 545 red poses ship with the [shop and duplex-lift assembly](../design/competition-package/shop-duplex/README.md). The original 401 red exports are reconstructed from the preserved blue STEP and patches. The optional first command needs your separately obtained, exact January field assembly. The checked-in numerical pickup receipts suffice for the downstream replay.

```powershell
$field = 'path/to/FE-2025.STEP'
$poses = 'design/competition-package/shop-duplex/red'
python station-supply/transfer.py --field $field --poses $poses --expected-receive-sha256 6389d766ad3d4d65b885bccb615a23807cadd8ef4f14d846214cea8a4dd96f73 --body-count 545 --output sourced-match/source-545.json
python sourced-match/replay_source_match.py --model shop-545 --poses $poses --source sourced-match/source-545.json --output sourced-match/shop-545.json
python full-match/materialize_red.py --output artifacts/red-401
python sourced-match/replay_source_match.py --model reference-401 --poses artifacts/red-401 --source station-supply/transfer.json --output sourced-match/reference-401.json
python sourced-match/carried_stock.py
python sourced-match/spin_screen.py
python sourced-match/onset_sweep.py
python sourced-match/view_data.py
```
