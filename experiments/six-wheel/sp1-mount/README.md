# Attach the outer-slide measurement cable

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

The fixed-low receiver uses a TE/Celesco SP1-25 to measure shelf travel independently of the rack motor. The #89 rail kit retained a box for the sensor and a solid tab for the cable end. Placing the eye pin through that tab produces 25.13 mm³ of solid interference. The new kit bolts the supplied L bracket to the bridge and captures the eye in double shear on the moving shelf. The [dimensioned shop drawing](shop-drawing.svg) and [seven-piece STEP](sp1-mount.step) use robot coordinates at zero stroke.

## Set the two datums

Robot X points forward, Y left and Z up. This design applies to the fixed 560 mm receiver. The common guide and eye axis is Y−127, Z415. The nominal guide exit is X26. The pin center moves from X75 to X445. The shelf reference plane is Z485, 70 mm above the wire.

| Dimension or source feature | Installation consequence |
|---|---|
| Included reversible L bracket, two Ø3.7 holes 35.6 apart | Invert the bracket under the bridge. Rotate the instrument per the manual's **horizontal right-facing, 12.2 mm** configuration. The mating face is Z427.2. Fit the mounting screws at X−31.3 and +4.3, Y−107.1. |
| Housing reference center about X−13.5, guide exit 39.5 along the pull axis | Register the guide center to the actual shelf-pin axis, including its transverse offset. The plan view puts the mounting screw row 19.9 mm from the cable axis. Check solder lugs, insulated leads and clocking on the received part. |
| Eye center 63.4–63.5 from the housing center at full retraction | The 49–419 mm guide-to-pin span includes about 23.9–24.0 mm of retracted guide and eye geometry. The modeled spool payout is approximately **25–395 mm**, with about 240 mm of full-range reserve at the extended end. Treat a measured change of 370 mm as the travel calibration. |
| Eye OD 9.52, width 4.19, conflicting printed bore Ø.191 in / [4.19 mm] | .191 in is 4.8514 mm. Use the received eye as a gauge. The shoulder is Ø4.00, the ears are bored together Ø4.10 and the clear fork gap is 5.50. Free pivoting and measured backlash govern acceptance. |
| Same bracket drilled for the opposite, 41.4 mm cable height | That choice displaces the wire vertically by 29.2 mm. At home a 49 mm horizontal span would become about 57.05 mm with a 30.8° exit angle. Its calibrated slope across the stroke also changes. |

The December 2015 manual documents the open solder-lug model, included bracket, mounting configurations, eye geometry and part-specific ordering data. The current catalog lists a revision-1 exact-part model. The available native check uses independently drawn supplier reservations, without verified supplier B-rep surfaces. [2015 manual, pp. 1–3](https://intertechnology.com/Celesco/pdfs/SP1.pdf) · [TE SP1-25 catalog](https://www.te.com/en/product-SP1-25.html)

## Machine and assemble

1. Mill one 6061-T6 adapter with coplanar 2 mm lands around the four original bridge fasteners. Machine the pad and flange as one piece. Hold the mounting face flat within 0.10 mm and drill the two bracket holes from a common datum. Tap M3 × 0.5 and verify at least 4 mm of usable thread below the bridge. Trial-seat the actual L foot and both M3 × 6 socket screws. The modeled foot thickness is 2 mm. Reject screws that bottom before clamping.
2. Mill the shelf flange, 6 × 7 mm post, lower bridge and two 2.50 mm cheeks as a single fork. Keep the original two flush M3 heads through the shelf and their locknuts underneath. Machine a 5.50 mm slot and line-ream the cheeks after roughing. Locate the bore relative to the shelf holes, within ±0.10 mm. Dress the front edge without reducing the nominal 1.20 mm ligament beyond the drawing allowance. An appropriately sized end mill, corner relief and the wire-end gauge must clear the internal transition.
3. Turn a stainless shouldered pin, Ø4.00 +0/−0.03 × 10.80, with M3 × 0.5 × 5.70 beyond the shoulder. Machine a 1.1 mm wide, 0.9 mm deep slot across the head. Install from −Y. The 3.2 × 7 × 0.5 mm washer and locking nut seat against the shoulder. The fork has 0.30 mm nominal axial freedom under the head. Confirm that the washer bears on the shoulder, the locking element engages, two full thread pitches project, and the eye pivots freely without pinching the ferrule.
4. With the instrument supported and the slide mechanically restrained, fit the factory eye between the cheeks. Keep the cable in controlled tension while inserting or removing its pin. For service, hold the pin from −Y with a slotted blade and operate the nut from +Y. The swept extraction check includes the head, shoulder and thread. Support the released cable while the two accessible foot screws are removed. The computer checks 37.75 mm of counterhold-blade approach, a Ø8.6 mm socket reserve, and a Ø5.6 × 33 mm driver approach below each foot screw at both endpoints. Check chosen tool length, hand access, cable restraint and the route for the complete sensor before disassembly. [Native service and adverse controls](fit.json)

At the catalog 1.9 N ±25% cable tension, use 2.375 N for the upper steady pull. The 70 mm lever produces 166.25 N·mm at the shelf flange or 10.39 N in a 16 mm fastening couple. A 6 × 7 mm gross post section, with 6 mm in the bending direction, gives about 3.96 MPa at the mounting face. The smooth pin carries about 0.095 MPa in nominal double shear. These are static screens. Snag loads, post-root stress concentration, socket retention, locknut prevailing torque, bearing life, vibration and actual assembled stiffness require qualification. The sensor's own 15 g cable-acceleration rating and 500,000-cycle catalog figure do not rate the fabricated attachment. [2015 ordering table](https://intertechnology.com/Celesco/pdfs/SP1.pdf)

## What the native fit covers

The continuous check compares all seven new parts and three supplier or cable reservations with the #89 receiver, phase-bounded rotating bodies and registered harness envelopes. The nominal run screened **964 bodies and reservations**, including tests between the travel endpoints. No positive material hit or unresolved pair remained. The source-sized exterior housing box with 1 mm free-side padding also cleared the swept third-body reservations. Individual bracket openings, eye shape, neck length, pad seating, cable diameter and the tool envelopes have separate checks. Actual terminal boots, dress, undercuts, supplier thickness and hand access still need the physical article. [Audit implementation](check.py) · [Fit receipt](fit.json)

The changed parts alone retain at least **10.24 mm** to fixed reef structure at A and **7.10 mm** at B under the #89 allowances. The unaffected raised rail still governs the combined clearance at 2.908 mm. This selection reuses the attained poses and stock registration. Physical force, field alignment, optical geometry and scoring permissions remain subject to their existing reviews. [Governing rail study](../selected-match/README.md)

## Measure before using the signal

| Check | Record on the first article |
|---|---|
| Mount and cable axis | Measure guide and free-eye centers relative to the bridge and shelf at 0, 185 and 370 mm. Target ≤0.50 mm offset in each transverse direction. Inspect the cable jacket and guide through the stroke. The triangle-length error from a 0.71 mm resultant offset at 49 mm is only 0.006 mm, but the horizontal force can load a rubbing guide. |
| Eye and retained pin | Record pin, bore, neck OD, eye width, cheek gap, available angular swing and nut end float. Check the source's conflicting nominal bore callouts. A 4.98 mm bore and 3.97 mm pin permit 1.01 mm diametral play on reversal or slack. Spring preload usually draws the eye to one side. Measure it under reversals and expected vibration. |
| Electrical position | Compare the physical shelf datum with the full voltage-to-position reading at home, midtravel and full extension, in both directions. The SP1-25 catalog accuracy alone is ±1.5875 mm over 635 mm. The desired ±2 mm relies on the remaining error budget, calibration and independent evidence. Verify slack, guide obstruction and open-wiper responses separately. |
| Weight and parts | Retain the single supplied SP1 article, included bracket, cable and eye, plus the two shelf and four bridge fastening pairs. Weigh the two removed adapter parts, both replacements, pin, washer, locknut and two foot screws. Reconcile installed routing and any terminals, sleeves or clamps added at assembly. [Selected configuration mass](mass.json) |

## Reproduce

Use the registered project artifact directory, the matching signal-light STEP and the same owner-supplied field STEP. The output contains original project metal and standard hardware representations.

```sh
python sp1-mount/check.py artifacts "$SIGNAL_LIGHT_STEP" "$FIELD_STEP" sp1-mount
python sp1-mount/account.py "$AGENT_FRC_PLUGIN_ROOT" artifacts/selected-match-clearance/maps/shop-receiver.json artifacts/sp1-mount/maps
python sp1-mount/account.py "$AGENT_FRC_PLUGIN_ROOT" artifacts/selected-optics/issued/maps/registered-optics.json artifacts/sp1-mount/optics-maps
python -m unittest discover -s tests -p test_sp1_mount.py
```

The mass program verifies the #89 source and applies the final selected physical instances once. The included factory bracket and cable remain in the existing 0.12–0.19 kg sensor budget. The source plugin accepts a flat alternative from the original assembly baseline, so the map retains the other #89 additions and its signed mass interval when replacing the two machined mounting parts. The incremental modeled range is +23.24 to +61.57 g. The upper robot estimate is 51.97932 kg, with 183.80 g of represented headroom. The physical scale margin is unknown. The generated map is `artifacts/sp1-mount/maps/sp1-install.json`. [Accounting adapter](account.py) · [Source mass calculation](mass-input.json)

Against the independently registered camera kit, the second map verifies that the two fixed-camera shoes and both M4 screws remain in the selected installed inventory. It reproduces the same SP1 delta, an upper estimate of 52.002139 kg, and 160.984 g of represented headroom. Physical mass and combined optical behavior remain open. [Camera-inclusive mass receipt](optics-mass.json) · [Issued optical scene](../selected-optics/README.md)

The combined optical and actuator study renders the SP1 hardware and both camera marks, then replays the rack at its supported moving-mass upper bound. [Same-configuration camera and rack results](../integrated-slide/README.md)
