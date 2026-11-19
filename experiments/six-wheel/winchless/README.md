# A service fixture for the winchless carriage

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

[Open the interactive CAD view](service.html). The four tooling-table stands reach the stowed carriage with both permanent height locks retained. The front pair approaches from the nose and contacts the machined bridge. The rear pair enters from the sides under the carriage plates. Its fingers and removable keeper pins capture fore/aft translation, lateral translation and uplift. The fixed robot gives up powered and manual lift travel.

## Where the mass goes

The original allowance of 1.8–3.9 kg covers the lift drive, ratchet, drums, lines, hardware and release. The [accounting record](supported-budget.json) traces all 34 original occurrences through the selected shaft, drums, lines and earlier motor omission. The newer stationary hardware and the previously omitted motor earn zero separate credit. Both low locks, service rails and bushings, lift branch protection, controller and loose allowances stay on the robot. The tooling stands and table fasteners are off-robot equipment.

| Mass basis with direct carriage brackets installed | Entire primary group verified absent | Unresolved shared or retained group material |
| --- | ---: | ---: |
| Matched shelf material | +1.593 kg | −0.207 kg |
| No credit for original shelf material | +0.673 kg | −1.127 kg |

The margins compare the upper modeled mass with the inspection limit. Each column has the same 1,133 installed identities. Claiming the group requires tracing the whole booked allowance, including shared or integral material, against the as-built inventory. Finished guide faces and mountings still need their own bill. A physical inspection-scale margin requires weighing the complete robot. [Before direct brackets](budget.json) · [Original group and selected robot](../fixed-hoist/README.md)

## What the stands can reach

Sixteen reopened native solids were checked against 1,598 registered obstacles. At 0.25 mm test penetration all four pads engage the full 254.47 mm² circular land. The insertion envelopes use 2 mm pad retraction. An extra 2 mm of screw advance intersects each loaded surface by 508.94 mm³. Rear keeper pins withdraw toward the front. The reversed head sweep intersects each retained service rail by 134.19 mm³. The separate bracket-tool and 535 mm extraction checks give at least 8 mm separation from the stands in all 54 comparisons. [Nominal STEP geometry](service-stands.step) · [Fit and adverse cases](fit.json) · [Bracket access](brace-access.json)

## What the physical fixture must establish

The 18 kg receiving limit is an assumption for this study. Assigning a 2 g vertical reaction from the whole reference load to one stand gives 353 N. The front stand sees the larger demand, with a 114.4 N·m arm-root moment, 179.7 MPa illustrative weld bending stress and 1.26 kN of total tension in the outer anchor row. The head, thread and fillet models use nominal geometry. The bench, sixteen anchor positions, welds, threads, motion under load and keeper retention need physical qualification. The current design has no lifting or service rating. [Load cases](service-loads.json)

A receiving test needs the complete moving assembly independently supported and weighed under controlled conditions. Reactions on the pads alone cannot establish its mass or center of gravity while the permanent locks or another restraint carry an unknown share. With ±0.10 kg per indication, the four example readings of 4.25 kg give a 17.4 kg upper mass and 104.0 mm minimum support-polygon clearance. Loading the rear pair alone fails both the minimum pad reaction and the 20 mm edge reserve. Actual measurements remain open. [Fixed lift control states](../fixed-lift-omission/controls.json)

## Reproduction

Use the project-generated registered stowed scene and both shelf-material maps from the [fixed hoist](../fixed-hoist/README.md) and [direct carriage support](../carriage-support/README.md). The selected budget uses the installed Agent FRC assembly-accounting entry point.

```text
python winchless/fixture.py <registered-scene.json> --carriage carriage-support
python winchless/brace_access.py carriage-support
python winchless/service_loads.py
python winchless/accounting.py <installed-plugin> <stationary-shelf-maps> <output-folder>
python winchless/accounting.py <installed-plugin> <carriage-support-maps> <second-output-folder> --carriage-support
python winchless/view.py
```
