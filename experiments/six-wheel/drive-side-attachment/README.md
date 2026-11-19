# Fasten the two inner drive rails to the welded frame

<!-- cad-still -->
[![Team 1780 · stowed swerve robot](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · stowed swerve robot.** Red alliance R4 with numbered bumper covers. Purchased parts use project-created representations in this public assembly. [Source STEP · abbd8ab838](../../../cad/full-swerve-r4/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

The [rail flex study](RAIL-FLEX.md) examines lateral travel at the wheel bearings between these joints.

Each gearbox already reacts torque through two #10-32 screws into its steel adapter and four M5 screws into the inner rail. In the issued assembly the 660 mm rail merely meets the inside walls of the front and rear crossrails. A planar butt contact can transmit compression in X. The geometry supplies no positive connection for separation in Y or Z or a reversed longitudinal load. Four new ears close that load path and leave both rails removable. The upper estimate for the selected robot rises by **480 g**, leaving **273 g represented inspection headroom**. The assembled robot still requires weighing. [Shop sheet](shop-sheet.svg) · [50-body STEP](inner-rail-joints.step) · [Predecessor adapter and carrier](../design/installed-procurement/CLAMPS.md) · [Axles and keepers](../axle-retention/README.md)

## Machine, locate and weld

1. Retain both original 6 mm inner rails and every original axle/adapter datum. Trim each end by **4.00 ±0.10 mm**, to a finished nominal 652 mm. Preserve the two permanently attached end take-up tabs on each rail. Use robot center X0 and the three original axle centers, with Z76.2 at the shaft axes. Bolt rows are at |X|312 and Z90/130. Drill Ø9.0 +0.15/0 with centers ±0.10 mm. Keep both faces flat to 0.10 mm over each joint and deburr.
2. Machine four matched ears from documented weldable aluminum, proposed 6061 to match the prior frame trial. Blank 30 × 20 × 64. From |X|300 to 322, leave the 8 mm web at |Y|235–243. Widen its outboard edge on a straight ramp to |Y|255 at |X|327. Leave the last 3 mm to |X|330 full-width. The ear occupies Z78–142. Mirror X and Y about the robot centerlines. The flat at each weld toe makes the 2.5 mm leg measurable.
3. Fixture with the welded frame square, the ears at their matched |Y|235 datum, representative 6 mm rails and the three supported shafts or gauging mandrels aligned. Maintain 0.15 mm rail-face parallelism over the carrier span and Ø0.10 mm paired bearing-axis registration over 90 mm from the original shop drawing. Tack, remove the detachable stock and electronics, and join the ear to the **2 mm crossrail wall** with two 64 mm vertical fillets on opposite sides, 2.5 mm equal legs. Verify the existing tube really matches the material, wall, temper and corner joint reviewed. Control distortion and test the actual welding process on an ear-to-tube coupon. [Tube material and weld fixture](../frame-corners/README.md)
4. Transfer-finish each pilot on the welded frame with the bearing alignment fixture installed. Use a matched Ø6 × 12 ground steel dowel, 8 mm embedded, 4.00 ±0.05 projecting inboard. Proposed pin diameter 5.997–6.000 mm. Qualify a 0.006–0.015 mm interference fit from the actual pin lot; press from the accessible side after welding and before final rail installation and witness its end. Plan replacement from the released rail side with a supported press fixture; actual drift and anvil access for pin replacement remain to be proven. The front rail hole is round, Ø6.030–6.045. At the rear relieve only X, with two end centers 1.20 mm apart and the same width. This permits ±0.60 mm differential longitudinal registration without releasing the vertical datum. Keep minimum axial engagement 3.95 mm; verify with the measured faces. Test pushout both ways and recheck after cycling. A pin that moves in the ear fails acceptance.
5. Reinstall the rail over its two dowels, from inboard, and seat squarely against both ears. One M8 screw, one washer under its head, one washer under the prevailing-torque nut, **two positions per ear**. A 30 mm screw from |Y|227.4 ends at 257.4; the nominal nut ends at 252.6, leaving 4.8 mm beyond it. Inspect the actual stack, complete locking-feature engagement, washer seating, edges and thread height. Measure the fastener lot, torque-to-preload and residual clamp after the proof trial, and mark hardware. Obtain the supplier’s allowable torque/reuse for the paired coatings and strength classes. A witness mark alone cannot qualify preload.

## Buy and make

| Installed on one robot | Supply and receiving requirement |
|---|---|
| 4 ears, 8 weld beads, 2 machined existing rails | Cut four blanks, two welds per ear; order/document parent tube and filler by the approved coupon. The two rails stay in the BOM once. |
| 8 M8 × 1.25 × 30 screws | Accu **SSC-M8-30-HK-12.9**, full thread, ISO 4762, high-tensile steel; maximum Ø13.27 × 8 head, 6 mm hex. Catalog mass 16.9 g each. [Accu](https://www.accu.co.uk/metric-cap-head-screws/642509-SSC-M8-30-HK-12-9) |
| 8 M8 prevailing-torque nuts | Ferrometal **03698**, DIN 980 all-metal, class 10, M8 × 1.25, 13 mm across flats, 8 mm high. Aspen lists a 4.8 g nominal nut of this size. Use the class-10 source and verify lot/finish and prevailing torque. [Ferrometal catalog, item 03698](https://www.ferrometal.fi/media/downloads/catalogues/ferrometal_fasteners_for_industry.pdf) · [Aspen dimension table](https://www.aspenfasteners.com/content/pdf/Metric_DIN_980_spec.pdf) |
| 16 hardened washers | Keller & Kalmbach **200125684**, M8 ISO 7089, Ø8.4 × Ø16 × 1.6, 300 HV. One washer on each side. [Supplier specification](https://www.keller-kalmbach.de/produkte/verbindungselemente/scheiben/iso-7089---din-125-a-scheibe/p/200125684) |
| 4 ground Ø6 × 12 dowels | Select certified stock to the measured diameter/press/pushout requirement above. The source does not establish a specific lot, retention force or catalog SKU. |

Stock on hand, pack sizes, quote, compatible corrosion protection, actual weld deposition and spares still need owner input. These quantities are installed requirements. The mass map adds **40 loose/made articles**. It includes the weld metal in four finished ears. The two reworked rails retain their identities and the upper mass case takes zero credit for removed rail metal. [Mass allocations](mass-input.json) · [Installed 0.22.0 checker](accounting.json)

## Access and reversibility

The 795-source-body/gauge scene includes the shop robot, selected clamps, four welded frame corners, radial axle kit, bearing keepers and rotating hub screws. The issued rail still intersects a proposed inside fillet by **200 mm³** at every end. The 4 mm cut gives **1.50 mm nominal clearance**. A 0.1 mm overlength, 0.3 mm oversized weld toe and 0.2 mm weld/location error leave 0.90 mm in the stated one-sided stack. Fit with feelers at all four ends. [Native checks](fit.json)

Both bumper assemblies remain fitted for access to the eight rail bolts. Use the 6 mm key from inboard and a shallow 13 mm reaction socket from the tire side. Gauge the socket to **19 mm OD**, at most |Y|269 at the rear of the working tool. The 30 mm screw requires up to 12.8 mm internal clearance from the nut face facing the ear, including its exposed threads. The modeled working envelope includes an 84 mm downward handle on both tools with a short stroke toward robot center in X, approximately 5° for a 12 mm wide handle. A hex key extension and fine-tooth or repositionable drive must fit those dimensions. The full fitting, handle coupling and hand clearance need checking on the actual robot. The outside socket approaches the ear buttress to 1.08 mm. A long socket extending to |Y|315 intersects the front wheel by **7,202 mm³** at nominal axle setting. The selected maximum |Y|269 leaves at least 8.3 mm to the tire sidewall irrespective of the ±4 mm longitudinal take-up. Check real sidewall bulge, guards, chain and harness. [All four stations and both bolt rows](fit.json)

To remove a **rail**, support the deck, gearbox, wheels, chains and any rail-mounted equipment. Isolate drive power. Use the [existing wheel-service sequence](../axle-retention/wheel-service/README.md) to strip the relevant wheel bay, including both bumper sections when withdrawing the wheel stacks. Remove center face screws, adapter, attached drive components, three shafts, the inner bearing carriers/clamps and the removable inner jack screws; retain the fixed jack tabs on the rail. Hold the rail, remove the four attachment bolts and loose washers/nuts for that side. Pull the rail **8 mm inboard**, then lower **180 mm** with both fixed tabs. The issued STEP proves the bare rail and the tabs clear the retained native parts throughout that translation. The broad swept lower envelope touches the deck face at its starting boundary. Dropping 3 mm before releasing the pilots intersects their steel by 68.50 mm³ front and 62.53 mm³ rear. Reverse the route on its locators and re-gauge the shafts after tightening. A populated 88-body side/module swing, unmodeled deck fixings, a loaded deck, external pit stand and weld-bead intrusion at the jack tabs require separate first-article review. [Named rail, retained tabs, staging and controls](rail-service.json)

## Load path and proof boundary

The [route receipt](../design/drivetrain/route-receipt.json) supplies 19.147 N·m geared-side torque and a conservative single inner-bearing reaction of 747.27 N at the 38 A drive limit. Treating all three inner bearings on that side as simultaneously bounded by this reaction gives a deliberately severe rail screen. The 624 mm simply supported vertical span resolves axle reactions and the gearbox moment. One attachment receives the entire longitudinal reaction. The adverse case doubles all loads and adds a separately assumed 500 N transverse side load before doubling. The design takes no friction credit for bolt shear. The separate 10 mm minimum compression lever for the opening joint and 17 mm outboard eccentricity require actual contact verification. [Equations and cases](loads.py) · [Calculated demands](demand.json)

| Quantity | Factored adverse demand |
|---|---:|
| Each opposed gearbox screw, torque only | 754 N tangential |
| Each of four M5 adapter screws, torque only | 229 N tangential |
| M8 screw, combined nominal equivalent stress on 36.6 mm² | 473 MPa |
| Earlier M6 at the same final lever arms (20.1 mm²) | 861 MPa |
| 6 mm rail / 8 mm ear bolt-hole bearing | 153 / 115 MPa |
| Isolated pilot carrying the full resultant in 4 mm rail engagement | 210 MPa rail bearing |
| Two 64 mm seams, verified 1.75 mm effective throat | 156 MPa equivalent |
| Same load at only 1.0 mm effective throat | 273 MPa equivalent |

The project uses a **600 MPa maximum nominal bolt demand** as a preliminary design screen. The selected M8 geometry satisfies it; the M6 does not. This threshold is a design input. Verify certified fastener properties, actual tensile preload plus applied load, fatigue, thread engagement, contact patch, bolt-group prying and the all-metal nut pairing. The adverse case assigns 7.35 kN maximum nominal shear and 11.75 kN maximum separating demand to one bolt. Use these to size and instrument the fixture; no production torque follows from these numbers. A plain 8 mm web with narrow weld spacing needed 181 MPa under its own root eccentricity. The widened 20 mm root reduces that demand while increasing material. A computed 156 MPa weld demand and relatively thin 2 mm tube wall still require destructive joint coupons and the actual as-welded HAZ strength. A vendor screw strength marking cannot establish the strength of the welded tube. The pilot-only 210 MPa case makes a loosened clamp a stop-and-inspect condition.

For first article, measure both pilot axes and both rails, cycle an unloaded removal/reinstallation and record the three paired bearing runouts. Record clamped contact out to the modeled moment arm and prevailing torque. Load a representative joint at the stated directions and moment arms, monitor fixture compliance and slip, then inspect weld toes, backside wall, hole ovalisation, residual alignment, thread condition and pin migration after reversals. Include both crossrail directions and a measured side-contact case. Withhold use until the materials, actual loads, fixture, coupon failure mode, proof sequence and acceptance limits have responsible shop approval. The CAD is sufficient to fabricate a representative trial.

## Reproduce

Use the project's Python/OpenCascade environment and the pinned 603-body red place exchange. From the repository root:

```sh
python drive-side-attachment/joint.py artifacts
python drive-side-attachment/release_path.py artifacts
python drive-side-attachment/loads.py
python drive-side-attachment/mass.py "$AGENT_FRC_PLUGIN_ROOT" artifacts/axle-retention/maps/selected-axles.json artifacts/drive-side-attachment/maps
python -m unittest discover -s tests -p test_drive_side_attachment.py
```

The native run reopens every named body, checks material intersections, exercises tool failure controls and pins the applicable source hashes. [Generating the shop STEP](../camera-datum-shop/README.md) · [Producing the baseline accounting map](../axle-retention/README.md)
