# Fixed-height robot mass at inspection

<!-- cad-still -->
[![Team 1780 · stowed swerve robot](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · stowed swerve robot.** Red alliance R4 with numbered bumper covers. Purchased parts use project-created representations in this public assembly. [Source STEP · abbd8ab838](../../../cad/full-swerve-r4/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

The selected straight lower guide and existing camera gyro leave **369 g of modeled inspection headroom** even when the shelf receives no credit for removed stock and every possible shared hoist attachment is charged at the upper envelope below. The corresponding R103 estimate is **51.794 kg against 52.163 kg**. The issued fabrication drawings omit the old winch and all seven legacy brackets. Building exactly those drawings leaves 2.182 kg on the same shelf basis. These are prebuild results. [Selected bill](straight-guide.json) · [Inspection allowance](../design/assembly-accounting/inspection-limit.json)

| R103 margin on the straight guide | Matched original shelf stock | No original shelf credit |
|---|---:|---:|
| Historical zero credit for the grouped winch | −0.798 kg | −1.718 kg |
| Historical independent group-removal interval | +1.002 kg | +0.082 kg |
| Issued drawings with the original mechanism omitted | +3.102 kg | +2.182 kg |
| All seven original attachment blanks charged at their stock upper limits | +2.301 kg | +1.381 kg |
| All seven blanks and fourteen upper-envelope M6 through-sets | **+1.290 kg** | **+0.369 kg** |
| The preceding case plus the full gearbox/ratchet space filled with solid steel | −1.976 kg | −2.897 kg |

The last row deliberately exceeds plausible gearbox density as assembled. It shows why an unexplained object in a reserved space cannot simply receive a zero weight. Completely unidentified material has no bounded result. The alternative returned lower guide costs another 30.234 g on its independent-stock basis, leaving **339 g** under the pessimistic bounded case. Drive, custody, field clearance and scoring still require evidence on their respective configurations. [Returned-arm comparison](returned-guide.json) · [Primary straight guide](../guide-setting/README.md)

## What did the old allowance actually cover?

The original stability calculation assigns exactly one 1.8–3.9 kg load to the 34 shapes returned by the winch routing generator. The corresponding shop map has 24 distinct physical identities after grouping both flanges with each drum/sheave and the two segments of each rope. The original project supplies a single allowance, so its individual purchase prices or inferred component weights cannot be subtracted separately. [Historical routing and design](../design/receiver-lift/README.md) · [Complete identity register](straight-guide.json)

| Item on the earlier robot | Fixed-height fabrication decision |
|---|---|
| NEO, Sport reducer, ratchet, pneumatic release cylinder/linkage, pulleys, belt, guards specific to this drive | Omit the complete mechanism, including the unmodeled release articles. No additional motor, cap, enclosed gearbox or pneumatic credit. An externally retained cylinder, guard or unrelated part requires its own roster and bounds. |
| Shaft, bearings, drums, ropes, sheaves, upper pins and terminations | Omit primary winding and routing. The previously retired secondary haul has its own change and stays counted once. |
| Bearing and sheave arms, crossbar anchor tabs, deck riser | Seven legacy interfaces are inspected separately below. The issued CAD leaves the four structural parents in place. |
| Controller, isolated lift leads, PH, harness, low locks, service rails, bushings, carriage, deck and mast | Retain their selected project rows. Electrical wiring remains separately booked at 0.65–1.65 kg. The original 2–5 kg unmodeled allowance remains. The no-air change and all subsequent welded, machined, optical and guide changes keep their own deltas. |
| Chassis gyro | The selected fixed-focus production OAK-D Lite already contains a BMI270. Its installed article is `observer-camera-proxy`. Only its existing data stream is needed. Kickstarter units lack the sensor. A different module would require a pin, mass, mount and data-path revision. [Camera option and field source](../station-landmark/README.md) |

The source sum reopens the original robot and reproduces both wheel-adjusted baseline endpoints from **331 original loads**, the independent shop/duplex additions, the selected battery and bumpers and the original/selected wheel quantities. Omitting only the lift row gives 34.690780–45.870960 kg before subsequent changes. The earlier method subtracted a separate −3.9…−1.8 kg range after charging the lift at 1.8…3.9 kg. Its upper endpoint retained 2.1 kg from a group already removed in full. This comparison pairs each original endpoint with itself and then charges the residual explicitly. The installed Agent FRC checker verifies the same 1,189 known physical identities, zero second motor credit and the original limit. [Source sum](original.json) · [Prior interval comparison](../winchless/supported-budget.json)

## How much shared material can the mounts contain?

The reopened historic placement STEP contains the original bracket shapes at the fixed 560 mm carriage height. The separately reopened guide mast pair, machined crossbar and slotted ribbed deck share **0 mm³** of material with the seven corresponding legacy solids. Surface contact used to fasten an arm does not make its material part of the mast. For a conservative fabrication comparison, the bill nevertheless charges a solid bounding blank at every site, including space formerly occupied by holes. Each full dimension gains 1 mm over its source stock bounds. The maximum permitted material density is 8,100 kg/m³, above the project's nominal steel estimate of 7,850 kg/m³. All sizes below are robot X × Y × Z in millimeters. The paired pieces occur on opposite sides. [Native locator coordinates, STEP hashes and volumes](stock.json)

| Possible residue | Qty | Maximum blank | Upper mass each |
|---|---:|---:|---:|
| Crossbar anchor tab | 2 | 39 × 19 × 7 | 42.0 g |
| Lower bearing arm | 2 | 41 × 9 × 37 | 110.6 g |
| Upper sheave support | 2 | 80 × 11 × 19 | 135.4 g |
| Deck riser | 1 | 21 × 9 × 147 | 225.0 g |

A through-set includes one screw, sleeve, washer and locknut. The dimensional references come from the native M6 carriage mount. The proposed upper screw gage uses a Ø13 × 5 mm head and Ø6.5 × 44 mm shank. A native comparison found zero original screw material outside this union. For the other three items the calculation fills the enlarged cylinder bounding each complete part, including bores. A through-set is at most **72.264 g** under these bounds. Two per old mounting site add at most 1.012 kg to 0.801 kg of bracket material. These are uncertainty allowances. The planned robot adds no unneeded fasteners. Existing installed support screws remain in their own bill. Any actual extra loose hardware needs an identity, location and clearance check before adopting it.

The no-credit shelf case can absorb at most **2.182006 kg** of all residual material before reaching the modeled limit. Any larger block, extra fastener, denser stock or retained drive component requires a new calculation. The calculated margins say nothing about the weight of a particular fabricated robot. Future acceptance includes verifying the camera option, stock and counts, identifying integral material and weighing the complete selected robot under the inspection rule.

## Reproduce or compare a proposed build

Use the Python/OCP environment and installed Agent FRC assembly-accounting CLI. The straight guide input directory contains the two shelf material directories from guide setting, each with its `unknownGroupDeltaKg/selected.json`. The optional returned-arm inputs come from the independent-stock arm variant.

```text
python winchless-bill/stock.py
python winchless-bill/original.py
python winchless-bill/account.py PLUGIN STRAIGHT_GUIDE_MAPS WORK/straight
python winchless-bill/account.py PLUGIN RETURNED_ARM_MAPS WORK/returned --receipt winchless-bill/returned-guide.json
python winchless-bill/screen.py proposed.json --example
python winchless-bill/screen.py proposed.json
python -m unittest discover -s tests -p test_winchless_bill.py
```

The optional schedule starts with all seven sites explicitly omitted. Edit the X/Y/Z stock dimensions, upper density and mounting-set count at each site to study a bounded fabrication alternative. It rejects incomplete inventories and out-of-gage parts. The generated installed maps retain the independent estimate omissions and leave the physical scale reading empty.
