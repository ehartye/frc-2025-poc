# Where the straight-guide robot carries its weight

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

The selected robot fits both modeled weight limits. The no-shelf-credit upper estimate is **51.794 kg** under R103 and **59.294 kg** with bumpers under R408. Adding the battery gives **65.464 kg empty** on the field. One pipe takes that to **66.280 kg**. The historical drive model used 65.833 kg empty and 66.650 kg loaded. Its mass assumption therefore covers the selected upper bound by 369 g. It still needs independent assumptions about where the weight lands. [Fabrication bill](../winchless-bill/README.md) · [Mass conversion and contact cases](contact.json)

## How much load reaches the center wheels?

[Open the interactive axle-load experiment](index.html). On level carpet with six identical linear springs and centered weight, each axle carries one third. The registered contact coordinates are X−247.65, 0 and +247.65 mm, at Y±290 mm. Each wheel just reaches the ground in the native model. Under load, a short or soft center pair can transfer additional weight onto the end axles. That increases the torque needed to turn with skid steering.

| Center pair, equal end-wheel rates | Load on four end wheels |
|---|---:|
| Same height and stiffness as ends | 66.67% |
| 0.305 mm shorter, equal stiffness | 73.44% |
| 0.305 mm shorter, 80% of end stiffness | 77.23% |
| Same height, half the stiffness | 80.00% |
| 2 mm shorter, equal stiffness | 100.00% |

The examples use 1 mm reference compression, the settling distance if six equal springs shared the empty weight. Their equivalent vertical rate is 107.0 N/mm per wheel. Tire and chassis compliance determine the actual effective rate. The 0.305 mm sensitivity puts opposite 0.2% radius errors on the center and ends. In the signed drive traces, the radius error is correlated by side. Those traces supply no measured center-height difference.

For a centered row with equal end rates, let *r* be center stiffness divided by either end stiffness. Let *g* be the center gap and *d* the compression three equal wheels would have under that row's weight. Requiring at most 75% on the ends gives **g/d ≤ 1.125 − 0.75/r**. Identical springs tolerate a center gap up to 37.5% of reference compression. A center at 80% stiffness tolerates only 18.75%. Both centers can lift completely. The six-wheel solver allows different rates and gaps at every wheel and balances vertical force, pitch and roll. [Contact solver](contact.py)

## What happens during the station approach?

Both revised pre-feed trajectories use the upper station holes and production-camera gyro while the robot is empty. They attain stationary confirmation at 20.35 and 20.59 s against a 21.10 s feed. The inverse drive replay takes all 4,221 pose records in each source. It imposes 44% weight on the light side, varies the share on the ends and uses 11.8 V open circuit.

| Common resistance | Highest end share passing both signed clocks in the grid | First higher tested failure | First inhibit, left / right |
|---|---:|---|---:|
| 27 mΩ | 92.5% | 95%, phase demand exceeds 38 A | 10.160 / 10.165 s |
| 32 mΩ | 77.5% | 80%, terminal voltage falls below 9.5 V | 3.885 / 3.885 s |

At the selected 75% share and 27 mΩ the two peaks are 34.47 and 34.33 A, and the terminal minima are 9.954 and 9.952 V. With both center wheels unloaded that model inhibits at 3.295 s in both directions. The tabulated boundaries apply to the 13 sampled shares. The sampled wheel poses after an inhibit describe the original request. This replay earns no subsequent achieved motion or scoring credit. [Signed 5 ms results](opening.json) · [Registered station source and regeneration instructions](https://github.com/ehartye/frc-2025-poc/tree/bb2c04d4b24b6e9e2ac74511e9e0bd9ead7e4204/straight-guide-match)

A separate historical full-path replay restores the old empty mass and reproduces both published current, voltage and friction results to 1e−7. At the revised mass it includes that run’s existing pipe and mechanism demand schedule. The current corrected-station experiment stops at the feed. [Historical drive controls](power.json) · [Issued schedule](../conditioned-match/slow-power.json)

## How close to the centerline must the mass stay?

A 580 mm track permits a static lateral load-line offset of **34.8 mm** before the light side drops below 44%. An offset of 40 mm gives 43.10%. A turning robot has an additional load-line shift bounded by CG height times lateral acceleration divided by gravity. For these two station clocks, a horizontal CG offset within 50 mm implies a peak lateral acceleration bound of 0.167 m/s². At 450 mm CG height, the remaining lateral CG allowance is **27.14 mm**.

The project can presently place a guaranteed **18.284 kg across 48 parcels** by matching the bought drive parts, battery and removable bumper parcels to the installed scene and reopening the slotted ribbed deck. Additional material above each known lower mass remains in a 47.180 kg remainder at the upper empty mass. A distribution placing its centroid within **−31.80…+35.32 mm** guarantees the static 44% side target across the known parcel ranges. For the 450 mm height opening case, the sufficient range narrows to **−21.18…+24.70 mm**. Centering the remainder gives ≥46.63% on the lighter side. Putting it 40 mm left can reduce that bound to 42.10%. The 50 mm horizontal-CG hypothesis and the height still require a complete design moment budget. [Source-matched parcel ledger](moments.json) · [Residual placement cases](balance.json)

Future commissioning can compare the simulated spring and placement cases to six simultaneous wheel loads, tire rolling radii and loaded compression, using the competition bumper, battery and payload configuration. All results here are prebuild simulations.

## Reproduce the calculations

Use the project's Python CAD/vision environment. Generate the selected mapping using the [winch bill procedure](../winchless-bill/README.md), the registered stowed scene using the [fixed-camera route](../fixed-camera-route/README.md), and both corrected opening paths using the linked station source. The historical check uses the two issued slow-field pose files and mechanism traces in the shared project artifact tree.

```text
python wheel-loads/contact.py
python wheel-loads/moments.py WORK/straight/noCutCreditKg/allStocksAndFourteenSetsKg.json ARTIFACTS/fixed-camera-route/scene.json
python wheel-loads/drive_replay.py ARTIFACTS
python wheel-loads/opening.py ARTIFACTS STRAIGHT_GUIDE_MATCH/revised-opening.json
python wheel-loads/balance.py
python -m unittest discover -s tests -p test_wheel_loads.py
```
