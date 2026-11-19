# Review the station-fed robot

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

Open [the editable OpenSCAD assembly](station-l1.scad), select `stowed`, `receive`, `transport` or `place` in the Customizer, and press F5. Enter a part ID from [the bill of materials](bom.csv) to isolate it. The four named, colored STEP assemblies open in ordinary mechanical CAD. The right-hand view below removes the deck and bumpers to expose the two independent drive sides.

![Placing pose and cutaway of the six-wheel drivetrain](overview.svg)

| Review pose | Carriage height / slide stroke | Shelf front from chassis center | Loaded midpoint CG, X / Z |
| --- | --- | --- | --- |
| [Stowed](stowed.step) | 940 / 0 mm | 165 mm | −28.6 / 299.3 mm |
| [Receiving](receive.step) | 940 / 240 mm | 405 mm | −3.6 / 299.3 mm |
| [Transport](transport.step) | 670 / 0 mm | 165 mm | −28.6 / 244.5 mm |
| [Placing](place.step) | 560 / 370 mm | 535 mm | +10.0 / 222.1 mm |

Robot X points forward, Y left and Z up from the floor at chassis center. Each CG example places a 0.8165 kg pipe at its hypothetical seat. The stowed example is a sensitivity case. The empty robot has a 700 × 700 mm outer frame, an 850 × 850 mm bumper envelope and a 1,020 mm mast height. The placing shelf projects 110 mm beyond the front bumper.

## Lineage and selection

The project first investigated station-fed L1 on a differential chassis. The preserved [B4 contract](b4-source.json) fixes the original 220 mm receiving trial and 370 mm placing stroke. Its 200 mm raised retainer was an optional clearance reserve. The [bumper-aware docking study](../../STATION-DOCK.md) supports testing a 240 mm receiving stroke on the R6 shelf. This package uses opposed end-rim belt cartridges for experimental retention. The team still owns competition-role and purchased-hardware selection. Compare the release-day V0 and subsequent updated scoring clauses in the [edition-specific scoring record](../../scoring/README.md) before interpreting a placement.

## Trace the physical interfaces

| Interface | Modeled geometry | Manufacturing and verification work |
| --- | --- | --- |
| Drive | Six 152 mm wheels on 580 mm track. A motor/gearbox allocation drives each central axle. Two closed pitch bands per side connect it to the front and rear axles. Twelve sleeves register six shafts in bored rails. | Size drive ratio, torque, belt engagement, tensioners and bearings. Add keyed hubs, shaft retention, selected mounting patterns and rated motors. The two 5 mm axial lanes are a packaging limit. |
| Frame and bumpers | Hollow outer rails, inner plates, end crossrails, load deck and four bumper volumes. All four independent end wheels clear their corresponding crossrail. | Detail joints and service fasteners, removable bumper mounting, backing, covers and inspection geometry. Check stiffness and wheel alignment under impact. |
| Receiver and lift | Captured mast shoes and a crossbar support two nested three-stage channels 360 mm apart. Their 320 mm stages advance at half and full stroke. Overlap is 135 mm at maximum reach. | Design slides, low-friction elements, guides, lifting transmission, independent stops and power-off holding. Nominal channel clearance supplies geometry for this study. |
| Shelf attachment | The reopened R6 source receives four Ø5.3 mm bores, four Ø9.2 mm × 3 mm head recesses and two Ø12 mm optical ports. Four simplified screw bodies run through the shelf, 6 mm spacers and crossblocks. | Verify material grade, creep, fastener engagement, recess edge distance and block retention. Current PE, aluminum and steel densities are sizing inputs. The [structural study](../../structural/README.md) places its loads at the same four X = −80/+140, Y = ±180 mm centers. |
| Retention and state sensing | Two active cartridge reserves contact the pipe ends. Two underside sensor envelopes face upward at local X=40 and 100 mm through the shelf. Entry and seat may both read occupied. | Select belt compound, preload, motors, sensor range, optical protection and brackets. A transverse beam at that height crosses the continuous cartridge envelopes. The old X=−60 mm seat sensor missed the nominal seated pipe. |
| Service | Lift the battery vertically after detaching its restraint. The full nominal extraction sweep intersects only that removable bar. The electrical cover's nominal lift envelope cleared all registered parts. | Add hand and tool allowances, latches, terminals, strain relief and moving cable loop. Verify an isolated removal procedure on fitted hardware. |

The 1,000 N illustrative side load in the structural study produces up to 360 N of resultant shear per proposed shelf fastener and 535 N·m about the robot datum when placing. The hook-up and material allowables for the shelf, captive blocks, slide rails, carriage and chassis still require load testing. A static fit does not qualify an impact rating.

## Power, mass and procurement

The [BOM](bom.csv) identifies every solid, its frame, material or equipment classification, mass method, midpoint location and budget allowance. For fabricated shapes the mass estimate is actual reopened solid volume × assumed density. Other entries use explicit mass ranges. Including bumpers, battery and a 2–5 kg unmodeled allowance, the package spans **40.30–59.13 kg**. The CAD centroid approximates the CG of each equipment envelope. The cost range of **$1,975–$7,228** is a design allowance before tax, shipping, tooling and spares. The package does not identify any purchased hardware as already ordered or weighed. Establish an inspection-weight ledger with the applicable rule's exclusions and weigh the final machine.

The [electrical interface study](electrical.json) maps protected distribution to the drive sides, lift, slide, grip cartridges and controls. Its independent bus-load illustration uses 12.4 V open circuit and 25 mΩ combined battery/trunk resistance. The assumed receive, loaded travel and place loads are 38, 75 and 45 A, giving 11.45, 10.53 and 11.28 V. Simultaneously demanding the high case from every axis draws 162 A and falls to 8.35 V, below the study's 9.5 V floor. Battery history, conductor losses, protection, controller limits and motor torque curves must be measured or selected before this can set operating limits. Use [the desktop controller and I/O contract](../../competition-controls/README.md) for the travel permit, state ownership, independent exit measurement and unresolved physical channel assignments.

## What survived reopening

Each STEP reopened as **96 uniquely named valid solids**. The review considered all 4,560 part pairs per pose. Separated bounds discharged most pairs. OCP evaluated intersections for the remaining candidates. No pair exceeded 0.001 mm³ in these four poses. This checks registered robot parts at those instants. Motion between poses, the field, a moving CORAL, flex, cable routing and unmodeled hardware still require independent checks.

The [first-fit receipt](first-fit.json) records four wheel/crossrail clashes at 656.238 mm³ each, two tray/anchor overlaps at 100.0 mm³ each and slide noses projecting past the shelf. The revision narrows the end tubes from 25 to 20 mm, raises the anchors by 1 mm and moves the slide stages rearward by 15 mm. [Final STEP receipts](review.json) · [Independent checks](checks.json)

OpenSCAD 2021.01 compiled four distinct pose selections. Its isolated shelf render produced 3,276 triangles and 1,661,011.634 mm³. OCP measured 1,661,005.619 mm³ on the same named STEP part. Bounds agreed at X = −100…165, Y = −210…210 and Z = 865…880 mm in stow. Mesh tessellation accounts for the small volume difference. Agent FRC 0.14.0 matched the independently expected body IDs on all four reopens and rejected a control with a removed body.

## Rebuild

The project-owned [builder](build_assembly.py) reads the [assembly parameters](assembly.json), [fabricated components](mechanical.py), preserved B4 source and pinned R6 STEP. Change the source dimensions there to regenerate both the OpenSCAD and STEP forms. The standalone OpenSCAD file is editable independently. Preserve deliberate hand edits before regenerating it.

```text
python design/station-l1/build_assembly.py artifacts/robot-review
python design/station-l1/render_review.py artifacts/robot-review artifacts/robot-review/overview.svg
python design/station-l1/check_package.py artifacts/robot-review --plugin <installed-agent-frc> --openscad <openscad-executable>
```

The CAD builder and independent reopen use OCP 8.0.1. The plugin check uses Node.js. Use the actual OpenSCAD CLI executable for the last option. Keep changed outputs alongside the prior run for review. B-Rep geometry and identity, rather than timestamped STEP byte equality across separate exports, are the intended reproduction checks.

The [receiver lift design](../receiver-lift/README.md) replaces the lift-drive reserve in a separate four-pose assembly. It checks a synchronized twin-drum route, available motor voltage and service clearance.
