# Fabrication review · 401-body station receiver

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

A shop can review and quote **27 existing chassis and receiver instances from nine drawing families**. The [dimensioned drawing book](drawings.html) opens offline and prints nine A3 landscape sheets. [Cut list](cuts.csv) · [drill/counterbore coordinates](operations.csv) · [1:1 metric DXF operation patterns](patterns/) · [native comparison receipt](native-review.json)

The authoritative input is the blue [401-body stowed STEP at 3fc3f27](https://github.com/ehartye/frc-2025-poc/blob/3fc3f27b716546dc92a34c9c6dda8c99fd5b8ff4/design/competition-package/poses/stowed.step) with SHA-256 `6ab8b98d5f5a196809a87f299279b8f8c01a06384d6c39dcda1f51423f865990`. Separate proposed hardware in [the joint drawings and assembly notes](JOINTS.md) closes the outer carrier clamp geometry and the receiver screw-to-channel load path for design review. The [401-row inventory](inventory.csv) routes every installed ID to a single original design source. The [purchasing table](purchasing.csv) groups recurring candidate components and missing hardware by actual fitted quantity. Quantities exclude spares and refer to this robot, before any owner-selected substitutions.

## Reading a sheet

All coordinates are finished millimetres. B and C are edges of the finished blank. U/V increase as shown on the face, with the third dimension normal to it. The rail holes use rear-end and lower-edge datums so the paired bearing centers register across the frame. The shelf uses rear and robot-right edges. Existing fastener envelopes and stock selections are retained in the 401-body CAD.

The DXF STOCK layer shows nominal blank edges. THRU contains finished bore circles and horizontal slot profiles. CB_TOP is a distinct top counterbore diameter; obtain depth from the sheet or operation schedule. For edge-open reliefs, machine only the material remaining within the blank. DXFs describe nominal operations. Kerf, cutter compensation, lead-ins, fixture overtravel, tool selection and stock allowance belong to the shop. The R6 shelf edge needs the side-profile dimension on its drawing after the flat pattern is cut.

## Proposed inspection targets

The following are **first-article targets** awaiting team and capable-shop agreement. They make coupon and fixture requirements explicit.

| Interface | Target to review and inspect |
|---|---|
| General blanks | Deburr without rounding locating shoulders. Saw tube/plate oversize and square the datum ends. Typical finished blank length/width ±0.20 mm, plate thickness by purchased measured stock. Parallel opposing tube faces within 0.15 mm over each individual carrier span. Use a 700 mm maximum assembled frame and the separate 850 mm installed bumper gauge. |
| Axle mounting | Proposed true position Ø0.10 for M5 patterns on paired parts relative to the finished B/C datums. Transfer the opposing rail locations in a common jig. Keep the two center-bearing axes collinear within Ø0.10 over the 90 mm rail/carrier region. Rotate a gauge shaft by hand and check both ±4 mm adjustments without binding. The largest matched pair and actual running clearance govern tolerance stack acceptance. |
| Slots and mounting holes | For explicit clearance holes and slot widths, nominal +0.10/−0.00 mm; slot center travel 8.00 ±0.10. Gauge simultaneous installation with the actual screw/washer and adjustment at both stops. Keep holes coaxial through both tube walls. A masking plug or burr can use up the entire fit allowance. |
| Bearing and sleeve bores | Ø28.62 is the drawing's nominal carrier opening. The known shell is nominal Ø28.58. Choose retained slip, transition or press fit from supplier OD tolerance, measured lot, material and axial retainer. Record measured bore/OD and retention force before boring all carriers. Fit and inspect bumper crush sleeves to the accepted welding/retention process, then recheck rail flatness and bearing alignment. |
| Shelf and sensor datums | R6 upper nose proposed ±0.25 mm; optical through holes nominal +0.15/−0 mm. Propose pocket depth 3.00 ±0.10 and mounting true position Ø0.10 after matching to actual spacer and crossblock. Check the candidate polymer's temperature, creep and IR response. Verify a clear view through both ports and positive clearance at full slide travel. |

The modeled 100 × 25 × 2 and 100 × 20 × 2 hollow rails use square internal corners. Real extrusion or fabrication can carry corner radii, seams and wall variation. Obtain or measure actual stock and refit the saddle/web, jig and bumper sleeves. Cutting substitutes from nominal US stock directly would change the component and previously verified interfaces.

## Assembly traveler

1. Mark front, left/right, broad face and bottom datums on each blank. Pair the rail and carrier patterns before drilling. Record the bearing lot, tube wall/radius, raw lengths and hole gauges. Hold center-inner gear-access relief and attachment selection at the gearbox interface.
2. Locate outer tube center planes at |Y|337.5, inner plate inner faces at |Y|229, end crossrail exterior faces at |X|350. Hold the square outside frame at 700 mm and compare its design diagonals (989.95 mm). Use the [four-corner joint, fixture drawing and qualification traveler](../../frame-corners/README.md). Recheck opposing axle alignment after frame joining.
3. Fit the outer nut saddles, opposing bearings, shafts, wheel hubs, two sprocket lanes and chain per the [drive assembly sequence](../drivetrain/README.md). Fit the [selected axle retention and bearing keepers](../../axle-retention/README.md). Measure shaft endplay and verify wheel service access, bearing drag and dynamic chain clearance on both sides. Record dial readings for aligned axes; the narrow nominal tooth/chain side clearance makes alignment consequential.
4. Fixture and gauge the two receiver inner stages on the bench. Fit the four screw-retaining blocks and their transverse dowels before nesting. Match the four shelf holes and two bottom-up optical ports, then assemble the stack in [joint section detail](JOINTS.md). Exercise full travel and measure prototype contact under guarded bench conditions before accepting a mechanism configuration.
5. Fit both [removable bumper sets](../bumper-kit/README.md), record right-angle driver access, weigh each complete configuration, then check the electrical and service corridors in the [combined package](../competition-package/README.md). Consult the independently owned lift, controls and field qualification receipts for their approved editions.

## Build-readiness disposition

| Area | Available engineering input | What still stops shop or purchasing release |
|---|---|---|
| Chassis and wheel carriers | Native-matched rail/carrier/adapter families and stock/datum schedules; separately proposed outer nut saddles and clamp stacks | Source actual 700 mm frame stock and bearing/axle lot. Qualify the [issued corner welds and squaring fixture](../../frame-corners/README.md). Qualify the selected axle end hardware and keeper installation under axial load. Verify gearbox-adapter attachments, inner carrier hardware and torque/preload. The eight inner-center clamp positions land against a solid adapter plate. The 16 inner-end positions need positive retained hardware. |
| Receiver shelf | Native-matched R6 profile, Ø12 ports, shelf spacers, counterbores and a revised pinned crossblock detail | Confirm polymer grade, actual long-screw head/grip, material and press fit, thread holding strength, pin tear-out, station contact behavior and removal sequence on a first article. A Ø5.3 clearance bore in the original block cannot tighten the screw. |
| Receiver transmission | 144 registered solids, original belt/shaft layouts and [receiver BOM](../receiver-drives/bom.csv) | Pitch-disk pulleys and several gearbox/bearing proxies lack tooth form, keys/retainers, final pulley and cheek fits; finalize actual HTD pulleys, guide clearances, sealing and fastening. The damping housing has a fluid/shear model and still needs seals, wipers and containment. |
| Core electrical and pneumatics | A protected [branch and channel map](../electrical-inspection/README.md), named control/power candidates and service clearances | Actual harness cut lengths and connectors, service loops, terminal boots, acquisition-board design, compressed-air plumbing, fuse/breaker inventory and cable/pad strain relief need a purchased BOM and bench evidence. A clear service volume is an access reservation rather than a purchased item. |
| Observer, lift, bumpers and inspection | Optical mounting design, lift route, 44-part bumper kit, rail gauges and reusable review artifacts | Source optical clamps/adhesives and qualify visibility; use the separately controlled lift-catch design and load tests. Two physical bumper builds, installed mass, torque, fastener witness, team numbers, service-time demonstration and event inspection remain open. |

## Rebuild

```sh
python design/shop-package/drawings.py
python design/shop-package/inventory.py
python design/shop-package/native_check.py artifacts/shop-review --export
```

The native checker independently reconstructs the planar machining operations in local shop coordinates, including the hollow sections, eight millimetre slot travel, edge-open cuts, 3 mm counterbores and R6 side geometry. It compares both material differences with each individually named source body after transforming the exact archived STEP into the same datum frame. It also evaluates the proposed joints against the 401-body installation and reopens the exported parts. The receipt records positive and rejected tool checks. The 1:1 patterns and dimensioned drawings support quoting, machining review and first articles. Final tolerances, materials, preload, joining and hardware choices belong to the team and shop performing the build.
