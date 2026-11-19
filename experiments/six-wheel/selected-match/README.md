# Fit the receiver to the reef wall

<!-- cad-still -->
[![Four-module swerve frame and mounting hardware](../../../docs/robot/cad/assembly.png)](../../../docs/robot/cad/index.html)

**Project CAD · Four-module swerve frame and mounting hardware.** Original robot structure for the swerve candidate. The project-local Fusion assembly adds four manufacturer module occurrences. [Source STEP · f8b75f2df4](../../swerve-drive/packaging/swerve-frame.step) · [Rotate the model](../../../docs/robot/cad/index.html)
<!-- /cad-still -->

The original scoring pose put both inner slide rails through the trough wall. A raised channel floor and a relocated position sensor give the reviewed shop robot **2.908 mm minimum clearance** to fixed reef structure at both scoring sectors through the modeled stroke and error allowance. The result supports a fabrication trial. It does not assign either pipe a score. [Explore stroke and sensor travel](review.html) · [18-body machining STEP](receiver-clearance-kit.step) · [Field clearances](corrected-clearance.json)

| At the two attained docks | Original receiver | Revised shop receiver |
|---|---|---|
| Inner rails, 370 mm stroke | Each rail occupies 1,386.84 mm³ of the trough wall, at both docks | Swept sections clear fixed reef structure by ≥2.908 mm at A (−180 mm) and B (+220 mm) |
| Position feedback | Original flag occupies 331.83 mm³ of wall at A. At B it reaches both a panel and a fastener | Sensor and flag move together to a new Y datum. The direct lead retains 49 mm stowed and 419 mm extended length |
| Stowed station | The registered original poses clear the station | 10.00 mm and 10.54 mm to the front bumper. Another 18 mm of station approach gives 65,780 mm³ positive bumper overlap |
| Other field occupants | The nominal ball gap did not reveal the wall contact | Reef-ball lower bounds are 16.48 and 24.89 mm. The earlier placed pipe has 16.09 mm clearance under the documented assumptions |

The original material check uses the attained pose, actual native material and registered field occurrence. The revised sweep encloses the occupied sections through intermediate stroke and heading, then compares them with the field structures. These are different observations. The viewer illustrates the original intersection section and the engineered profile. It displays the computed full-sweep bound without treating a screen pixel as a measurement. [Original native counterexample](reef-material.json) · [Earlier stock, station and route bounds](clearance.json)

The follow-on installation registers the real SP1-25 bracket pattern, captures the cable eye on a removable smooth pin and preserves the original shelf travel. [Draw-wire mounting and calibration](../sp1-mount/README.md)

## Make and inspect the kit

Use robot X forward, Y left and Z up. The carriage height for this configuration is 560 mm. Source dimensions below are in millimetres. The STEP at zero stroke contains 18 manufactured solids. The existing SP1-25 and changing-length wire belong to the complete robot scene; their original purchasing identities carry forward. [Editable geometry](rail_profile.py) · [Material, round-trip and placement controls](kit.json)

| Work | Dimensions and assembly check |
|---|---|
| Two continuous inner U channels | Rail length 320, width 20, top Z479, side and floor thickness 3. Floor underside stays at Z446 through local X58, rises 19 by X98, and remains at Z465 through the front X160. Machine the smooth-transition profile from the source’s 16 straight segments. Keep original pin stations at X−85, −75, +135 and +145 through both sidewalls at Z475.5. Cross-drill/gauge against the assembled blocks, deburr and keep pin axes paired. |
| Slot the existing fixed bridge | Open the 12 mm-wide channel forward, centered at Y−127, with R6 rear end centered at X55. Preserve bearing seats, motor, saddle interfaces and both side strips. At stow the relocated flag must pass freely without rubbing. |
| Reposition the measurement hardware | Shift both endpoints 42 mm aft and 81 mm toward center from Y−208 to **Y−127**. Keep the transducer on the fixed lift/saddle and the flag on the outer shelf. The middle rack moves half the outer stroke. The independent wire reads the actual outer-stage displacement. Gauge routing, orientation, wire exit alignment, return tension and full-stroke calibration on the actual SP1-25. |
| Drill the flag and shelf | Moving flange 44 × 12 × 4, attached with two M3 × 25 countersunk screws and two M3 locknuts. In the retracted position drill at X96/112, Y−127. Heads sit flush in the piece-facing surface at Z500. Verify head angle/depth, thread projection, edge finish and actual socket/locking access. |
| Mount the saddle | Four M3 × 12 countersunk screws and four locknuts. Drill at X−48/+34 and Y−117/−92 through bridge and saddle. Reopen at full stroke, check screw retention, and weigh actual stock, hardware, coatings and any service loops. |

A flag at Y−142 and 16 mm stroke intersected the stop by **672 mm³**. Shifting the sensor 42 mm aft while keeping the old Y coordinate intersected the height pin by **1,658.76 mm³**. The accepted added-material check spans the full stroke against 884 registered bodies and rotational envelopes. The field screen includes another 74 service/cable envelopes. The shop check found no positive or unresolved interference for the changed material. [Shop fit algorithm](shop_fit.py) · [Source-pinned fit result](kit.json)

At a full 1 kN applied to one front block, the gross U-section calculation gives **169.48 MPa** at local X95.5. A 500 N case gives 84.74 MPa. The front hole leaves 5.95 mm below and **1.95 mm above**. Material certificate, actual root radii, machined edge distances, pin bearing and tear-out, fastener pull-through, surface condition, elastic deflection and cyclic loading need a representative fixture. Measure the front tip under load and repeat the wall-clearance gauge with measured docking, rail and field tolerances before adopting the design. [Load calculation and explicit assumptions](shop_fit.py)

## Evidence that can carry forward

The selected map uses the #88 frame attachment baseline plus the relieved receiver and the new clearance kit. Eight grip parts are replaced at existing locations, four shims are added, six spring/stop articles are retained, and the clearance kit books 12 fasteners. The bridge and shelf are machined in place. The installed 0.22.1 checker reports full mapping coverage and **51.917745 kg upper represented mass**, **0.245377 kg represented margin**. Physical inspection margin remains unknown. [Kit mass delta](clearance-mass.json) · [Combined map result](mass.json) · [Upstream frame joints](../drive-side-attachment/README.md)

The stowed selected robot fits the registered 850 × 850 × 1059 mm envelope. The lower bound to occupied stock on the stowed route is only 1.034 mm. The proposed fixed camera power tail reaches Z1064.735, **5.735 mm above** that envelope, so it needs its own route and clearance check. The fixed camera head, power/data fitting, cable shape, pressure, tracking, wheel calibration and attained docking pose have separate evidence boundaries. Retain the original detector observations as software evidence until the same robot and clock are qualified. The 2.908 mm clearance cannot absorb an arbitrary additional docking or fabrication error. [Match software experiment](../relieved-match/README.md) · [Drive, traction and dock review](../selected-drive/README.md)

## Reproduce

Use the same registered 2025 field STEP, signal-light STEP and project artifact root. The owner-supplied files stay outside Git. The scripts verify source identities. From the repository root run

```sh
python selected-match/reef_material.py artifacts "$SIGNAL_LIGHT_STEP" "$FIELD_STEP" artifacts/original-reef-material.json
python selected-match/shop_fit.py artifacts "$SIGNAL_LIGHT_STEP" selected-match
python selected-match/clearance.py artifacts "$SIGNAL_LIGHT_STEP" "$FIELD_STEP" artifacts/baseline-clearance.json
python selected-match/clearance.py artifacts "$SIGNAL_LIGHT_STEP" "$FIELD_STEP" artifacts/corrected-clearance.json --corrected-shop
python selected-match/account_receiver.py "$AGENT_FRC_PLUGIN_ROOT" artifacts/drive-side-attachment/maps/selected-rail-joints.json artifacts/selected-match-clearance/maps
python -m unittest discover -s tests -p test_selected_match.py
```

Compare the replayed original with [reef-material.json](reef-material.json), the baseline with [clearance.json](clearance.json) and the corrected case with [corrected-clearance.json](corrected-clearance.json). The corrected mass map is written as `shop-receiver.json` in the selected output directory.
