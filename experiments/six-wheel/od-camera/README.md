# Two cameras on the outside-diameter receiver

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 83b3b9c7fe](../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

**Retain the end-belt receiver for the low-role robot.** The outside-diameter (OD) design accommodates variation in pipe length and conditionally places the centered and long, yawed pipe. Installing the three proposed camera power converters puts its conservative inspection estimate **748 g over** the R103 limit. The end-belt robot retains **1.061 kg** under the same accounting. A new camera bridge solves the mechanical interference and improves the view at the trough. The remaining mass, power and clearance-observation limits make this a fixture candidate. [Inspection and power comparison](budget.json) · [Original receiver and 52/64 disturbance grid](../od-gate/README.md)

## What physically fits

The original red no-air robot has 593 bodies. Its moving stereo camera was bolted through a drilled shelf that the OD retrofit removes. Keeping that bridge in place creates 16 positive-volume interference pairs with the closed receiver and 35 with both gates open. A native identity comparison retires 104 receiver parts and 18 old camera supports, keeps 468 parts in their registered positions, moves the same camera and two bolts, and adds 147 parts for a total of **618**. Two longer rear anchors replace the short OD anchors. The fixed stereo unit, signal light and previously budgeted moving-camera cable retain their original allocations. [Direct substitution](unmodified.json) · [Body identities and reopened volumes](native.json)

The two 2.5 mm feet share the outer slide's rear anchors at X −80 and Y ±180 mm. Welded 15 mm square posts and 12 mm booms carry a horizontal camera plate. The moving lens is at X +75, Y 0, Z 895 mm relative to the unextended stage, pointing straight down. Both short-range heads stay on the hood at Y ±110 mm. The other stereo unit stays on the chassis at X −315, Z 970 mm. The original [15-solid camera bridge STEP](revised-bridge.step) is editable through [bridge.py](bridge.py). Stock and joints require dimensional inspection.

The continuous 370 mm slide sweep leaves at least 1.249 mm from the long anchor to the outer slide under the declared frame motion. A wider foot had zero qualified clearance against both masts over the first 101 mm. A symmetric 2 g cantilever screen predicts 1.09 mm post deflection before joint or boom flexibility. Measure mount deflection, optical alignment, fastening and cable sweep under load. [Travel sweep](travel.json) · [Failed wide foot](foot-width-control.json) · [Station approach and adverse contact](station.json)

The candidate camera plate is the closest part to the occupied reef ball. At the centered target it clears by 31.86 mm in sector A and 54.92 mm in sector B. The long delivery's +15 mm approach error reduces sector A clearance to 19.19 mm. A further 25 mm of forward chassis error at the 370 mm slide stop produces **4.5 mm³ of native interference**. Sector B retains 25.15 mm in the same control. These twelve samples cover two gate positions and the stated chassis errors. The end-belt robot has a separately qualified [aft camera mount](../bridge-tolerance/README.md). [Occupied stock evidence](occupied.json)

## What students can actually observe

The register includes the red station’s 47 occurrences, 184 red reef occurrences and twelve separately installed reef balls. Synthetic 640 × 480 stereo images use the actual rotating gate and the finite hollow pipe. Each exposure compares six alternate poses with a two-pixel edge allowance. These are enumerated disturbances. A continuous pose error bound still needs calibration on the real cameras. [Rendered observations](images.json)

| Left/right rendered pipe pixels | Center at station | Long, yawed at station | Center settled | Long, yawed settled |
|---|---:|---:|---:|---:|
| Chassis stereo | 12,271 / 12,314 | 12,283 / 12,441 | 226 / 260 | 195 / 359 |
| Moving stereo | 68,498 / 68,498 | 73,502 / 66,021 | 44,684 / 44,684 | 45,328 / 45,463 |

Both station images from the roof clip the theoretical pipe outline. Both final pipe silhouettes fit within the roof images. An earlier rearward aim rendered zero centered pipe pixels at rest and only 39/45 in the long case. The paired hood range cones return 9.26 mm on centered pipe and 9.31–10.33 mm on long pipe at the three seated poses. They both report empty at 4.100 s in each release. A real sensor still needs an operating-range, surface, crosstalk and freshness check. [Aim control](rearward-aim-control.json) · [Independent head rays and empty controls](ranges.json)

| Complete robot and finite pipe | Center | 307.975 mm pipe, 4° yaw |
|---|---:|---:|
| Start of continuously bounded >1 mm separation through rest | 3.678 s | 4.308 s |
| Sampled interval wholly beyond front plane by >1 mm | 3.842–4.062 s | 3.860–4.106 s |
| Final nearest-body separation | 19.19 mm | 3.64 mm |

A positive front-plane reading establishes instantaneous separation in the modeled pose. In the yawed run it expires before the pipe attains bounded clearance through rest. Both empty hood readings also precede that clearance. Keep withdrawal inhibited until an independent observation bounds the identified pipe against this complete moving assembly for the intended withdrawal. A future fixture needs the long pipe, 10 mm lateral offset, gate lag, target error and a failed release control. [Whole-body trajectories](departure.json)

## Electrical and inspection limits

The new material and hardware add a conservative 1.809 kg after subtracting the displaced articles once. Only 2.49 g remains before adding the three fixed-base converters. The alternative correlated mass calculation leaves 52.49 g after the 0.750 kg addition. Removing the entire moving camera still leaves the single-camera OD variant 182 g over its conservative estimate with those supplies. Weigh the finished assembly and individual old/new parts before relying on correlated savings. The converter housings are an uninstalled fixed-base allowance.

The three independent converter branches feed the host and two cameras on PDH 6/8/9. The model prices both additional range heads and their interface, the gate and hood drive, common drive and added slide inertia. At 11.8 V with 27 mΩ source resistance and 85% converter efficiency the parked transfer retains 0.976 V above the 9.5 V design floor. Loaded carry misses by 0.072 V and simultaneous transfer and drive miss by 0.913 V. At 75% efficiency the latter two miss by 0.089 V and 0.935 V. These cases reuse a historical drive-load background. Measure installed branch resistance, driver outputs, current, motion latency and minimum terminal voltage with the actual scheduling and protection. [Three-supply source](../fixed-low-power/dual-camera.md) · [Reconciliation](budget.py)

## Reproduce

Run the Python entry points in this directory from a project environment with Open CASCADE, VTK, NumPy and Pillow. Supply the locally sourced lamp STEP to `assembly_review.py`, `bridge.py`, `travel.py`, `probes.py` and `departure_review.py` via `--lamp`. Also give `vision.py`, `station.py` and `occupied.py` the registered January field via `--field`. Run `budget.py` following native export. The rendering command writes a four-view comparison under `artifacts/od-camera/`. The project publishes the original bridge and numeric measurements. [Source and evidence digests](register.json)
