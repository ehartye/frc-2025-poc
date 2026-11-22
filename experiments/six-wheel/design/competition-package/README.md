# Station receiver assembly review

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 83b3b9c7fe](../../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

The [545-body shop and duplex revision](shop-duplex/README.md) is the latest combined engineering model. This page preserves the 401-body revision used in earlier field and controls evidence.

Open [the robot viewer](robot.html) in a current browser. It works offline. Drag to rotate or use the arrow keys with the drawing focused. Switch between Stowed, Receive, Carry and Place. Select a subsystem or a single part to see its position, or remove the bumpers and deck to inspect the drives. The scale under the view controls follows the current zoom. The [full match rehearsal](../../full-match/replay.html) shows the initial stacks, route and alternative ending positions.

## What moves

| Pose | Lift height | Slide extension | Review question |
|---|---:|---:|---|
| [Stowed STEP](poses/stowed.step) | 940 mm | 0 mm | Can the receiver wait for a supply with the slide fully home? |
| [Receive STEP](poses/receive.step) | 940 mm | 240 mm | Where do the belts meet the pipe as it drops from the station? |
| [Carry STEP](poses/transport.step) | 670 mm | 0 mm | What stays within the bumper footprint during the transfer? |
| [Place STEP](poses/place.step) | 560 mm | 370 mm | Which surfaces can still touch the pipe as the shelf withdraws? |

Coordinates are in millimetres from the chassis frame, with X forward, Y left and Z up. The middle carriage moves half the outer slide distance. Three pinions rotate as the racks translate. The return lines change length with lift height and the string potentiometer pays out line with slide extension. The field adapter can call `pose_shapes(height_mm, stroke_mm)` for nominal intermediate geometry or read the [per-part frames](part-frames.json) and [pinion bounds](rotation-bounds.json). The default package uses the blue kit. The [red assembly receipt](red-alliance-review.json) records the separately generated and reopened red full robot. Pass `alliance="red"` to either pose API or add `--alliance red` to the builder for the matching red CAD/viewer. The nominal CAD shows centered grip cartridges. The [receiver-drive model](../receiver-drives/) supplies their independent lateral motion.

The [drive chassis](../drivetrain/README.md), [lift](../receiver-lift/), [receiver drives](../receiver-drives/), [removable bumper kit](../bumper-kit/), [electrical packaging](../electrical-inspection/) and [release observer](../../release-observer/README.md) own their dimensions and verification. Combined checks remove each superseded equipment reservation and reject duplicate or missing body IDs.

## Reproduce the checks

The selected assembly combines 401 named solids. Each pose checks 59,132 cross-subsystem pairs. The shorter, 420 mm grip belts replace the earlier 537 mm belts. An inward-displaced long belt could overlap the pipe by 143.4 mm³ even after the shelf cleared. The shorter belt allows a short unpowered coast on the shelf. The [receiver-drive study](../receiver-drives/README.md) records the resulting acquisition and handoff limits. The [source register](source-register.json) identifies the inputs that produced these four exports.

Run from the repository root with Python and the project's OpenCascade bindings.

```sh
python design/competition-package/build.py artifacts/combined-review --step
python design/competition-package/travel.py artifacts/combined-review/travel.json
```

The builder verifies source STEP identities, checks interference between subsystems, tests three service volumes and exports four named STEP assemblies. Each export is reopened and checked for valid solids, body count and material volume. The [pose receipt](review.json) lists the number of inter-subsystem comparisons and the actual interference in each pose. The viewer uses triangulated copies rounded to 0.1 mm. All fit checks use native solids.

The [travel receipt](travel.json) covers nominal continuous station extension, target extension and lift travel with the slide retracted. For each pair of subsystems the checker rules out disjoint swept bounds, verifies the common reference geometry of rigidly co-moving bodies, or reconstructs a native prismatic end face and sweeps it over the full relative displacement. A remaining unclassified pair fails the check. Both travel directions traverse the same occupied positions.

An early harness anchor passed all four named poses. At 100 mm of slide extension it intersected the synchronizer bracket by 320 mm³. A second position passed that bracket but crossed the slide motor. The revised anchor sits on the shelf underside at (75, −45, −90) mm. Removing the battery hold-down is part of battery service. The other two service checks exercise fuse and breaker extraction and access to the main disconnect.

## Design status

The [whole-robot balance study](../stability/README.md) checks the completed native poses against the turn schedule. It includes six-wheel weighing inputs, the clearance to the tipping edges and the grip required from the lighter drive side.

The field and robot models share a station-fed, low-level placement role. The team has yet to select this role and its shop constraints. Game-piece contact, observer latency, upper-mass match timing, spring and belt deflection, pneumatic loss, cable flex life and as-built tolerances each need their own evidence. A clear harness route alone does not establish when the robot can release the pipe. Consult the [release observer](../../release-observer/README.md) and the [field rehearsal](../../mission/MOVING-TRAFFIC.md) for the assumptions behind a timed delivery.

## Earlier field and observer evidence

The original 361-body assembly and its four STEP hashes are retained at [PR 25 geometry](https://github.com/ehartye/frc-2025-poc/tree/40c486d4ba53b227b8cf720b20b6d6bddac88b29/design/competition-package). [PR 26 field and optical evidence](https://github.com/ehartye/frc-2025-poc/tree/aa0b5d368918db56618fd84b28ecb65c4776a023/mission) qualified that exact geometry. The [historical receipt register](historical-receipts.json) pins both source editions. The 401-body robot preserves the registered 850 mm bounding square while replacing four envelopes with removable bumper parts and drilling four frame members. The new CAD motion receipt covers installed fit. Field release, occlusion and contact timing require a new downstream qualification against these four new native STEP hashes.

## Shop handoff

The [fabrication review](../shop-package/README.md) supplies nine dimensioned drawing families, metric DXFs, a purchasing table and an inventory keyed to the 401-body revision. Two proposed positive-retention details have their own native review and mass delta.
