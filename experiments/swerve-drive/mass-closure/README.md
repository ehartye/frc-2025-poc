# Inspection mass for the fixed-height swerve build

The drawing build reaches **51.993 kg under R103** with the repaired rear electronics installed, leaving **170 g** of calculated headroom. Including the 7.5 kg bumper upper gives **59.493 kg under R408**, 1.742 kg below the project limit. Both keep the full original 2–5 kg unitemized allowance, all eight controllers, the isolated lift wiring, the longer motor phase extensions, purchased-module receiving ranges and every frame/weld allowance. The baseline four-module design with long supply wiring remains 3.650 kg overweight. [Source bill](../power/installed.json) · [Reopened CAD measurements and eleven comparisons](recipe.json)

## How much room does the numbered R4 actually have?

The R4 STEP contains the issued polymer shelf face and aluminum insert at **665,612 and 302,118 mm³**. It contains the five rear-repair replacement bodies, the retained crossbar and both masts. All 54 retired outer-clamp identities and seven obsolete winch blanks are absent by name. The student F3D matched its release digest, the native reopen placed one 274-body supplier module at each of the four registered wheel datums, and the assembly checker reproduced the complete selected change list and its 944 physical IDs. The public exchange has 975 solids, including supplier envelopes and 16 printed digits on the removable covers. Those visual representations add no separate R103 article. [Numbered CAD](../../../cad/full-swerve-r4/design.md) · [R4 receiving and geometry receipt](r4-receiving.json)

The largest identified receiving sensitivity in this **module-and-harness comparison** is the four populated modules together. REV lists **1.720 kg with NEO**. The project books up to 1.806 kg per corner, an 86 g difference that repeats four times. The published module weight is nominal. Compare the same installed population on every corner: module core, one installed V2 wheel, selected gears, encoder, NEO and NEO 550. Keep spare wheels and unused kit articles off the robot. The eight external motor controllers, fabricated mounts, separate extension harnesses, protection and finish retain their existing charges. Two reused drive motors already leave their original mass row before the populated assemblies enter. [REV specifications and supplied articles](https://www.revrobotics.com/rev-21-3005/) · [Original matched bill](../power/mass.json)

| Four populated module masses | Corner-specific supply cuts | Uniform 1.00 m supply cuts |
| --- | ---: | ---: |
| 1.720 kg each, supplier nominal | 514 g below R103 | 339 g below |
| 1.806 kg each, project receiving upper | **170 g below R103** | 5 g over |

A **500 g project working reserve** gives the inspection team a useful receiving target. With the issued cuts, the four modules must total at most **6.894 kg** to preserve it, or 1.7235 kg on average. They may differ individually. The supplier nominals total 6.880 kg, leaving only **14 g** beyond that target. The formal R103 ceiling permits 7.394 kg in total when all other assumptions hold. Every additional gram beyond a separately booked group reduces both allowances by one gram. These cases retain the full 5 kg miscellaneous upper. The selected R408 upper still includes the 7.5 kg removable bumper set. Physical inspection mass remains unknown.

The R4 controller and PDH placements also narrow where the wire budget is most exposed. For example, the 900 mm rear-right cut contains 438.8 mm of screened common route. The largest sum of straight tail distances from the two handoffs to points in the registered PDH and drive-controller boxes is 328.0 mm. At those box extremes, **133.2 mm** remain for terminal offset, bends, service and routing around hardware. Front-right drive has 134.7 mm under the same test. These are endpoint sensitivities. The CAD boxes do not establish connector pin locations, pigtail lengths or a collision-free routed tail. Preserve those separately when assigning the finished lengths. [All eight circuit screens](r4-receiving.json)

Reproduce the public CAD check with `python experiments/swerve-drive/mass-closure/r4_receiving.py`. For the full replay, pass `--map PATH_TO_SELECTED_JSON --plugin AGENT_FRC_ROOT --archive PATH_TO_R4_F3D`. The private map and native archive are the same project-owned inputs used for the release receipt. The script reports whether each was supplied.

## Build the selected drawing

1. Use the fixed-height mast and support drawings. Omit both old winding-shaft bearing arms, both upper sheave supports, both crossbar anchor tabs and the old riser. Leave their fourteen optional mounting sets out of the robot. The earlier study checked each legacy solid against its structural parent and found zero shared material. Retain the mast, crossbar, rail supports and their actual fastening hardware. A student retrofitting an existing robot should inventory both mast faces, both ends of the crossbar and the former riser on the deck. The possible seven blanks and fourteen sets accounted for 1.812814 kg in the previous pessimistic estimate. [Seven locations and dimensions](../../six-wheel/winchless-bill/stock.json)
2. Make the machined R6 shelf with its polymer face and aluminum insert. The old drilled PE shelf occupied 1,655,363 mm³. Its remaining face occupies 665,612 mm³. The removed 989,751 mm³ earns 920.469 g at the lower registered PE density of 930 kg/m³. The complete replacement insert still receives the 2,850 kg/m³ upper, and both shock absorbers, four nuts and two witness allowances retain their full 44 g. A second complete original shelf or unknown/changed stock invalidates this credit. [Installed machining and material](../../six-wheel/shelf-structure/installed.json)
3. Omit all six original six-wheel outer bearing clamps, each consisting of one nut saddle, four washers and four screws. The former six-wheel shop added **560.769 g** specifically for those 54 items after its original drivetrain bill. The four-module roster retires all 54, but its earlier mass comparison still charged that separate shop row. Reopening the original STEP gives 71,435.585 mm³ at the booked 7,850 kg/m³. The 14 receiver-joint bodies in that STEP and their separate 31.338 g slide change stay in the historical accounts. The implementation rejects any overlap with a previously credited group. [Original joint STEP](../../six-wheel/design/shop-package/proposed-joints.step) · [Physical retirement register](../packaging/retirement.json)
4. Make two independent positive/negative power pairs at each corner. Use 10 AWG for the drive extension and 14 AWG for the steer extension, under the original wire OD and insulation density limits. Use the following maximum finished lengths per conductor. They are also routing budgets. Leave the motor-side extensions, CAN and sensor allocations unchanged. The registered common spans clear the prior routing envelopes. The unmodeled portions must accommodate both connector tails, service slack, fixings and breakouts in the stated length. Check the path with cord from the actual PDH terminal to its assigned controller before cutting. Keep the drive and steer protections and supplier pigtails assigned in the electrical plan. [Registered routes](../power/paths.json) · [Branch assignments](../power/branches.json)

| Corner | Each of four supply conductors | Screened common span | Available beyond that span |
|---|---:|---:|---:|
| Front left | 800 mm | 325 mm | 475 mm |
| Front right | 950 mm | 478 mm | 472 mm |
| Rear left | 550 mm | 96 mm | 454 mm |
| Rear right | 900 mm | 439 mm | 461 mm |

The wire assembly upper is **1.492 kg** including the 460 g connector, sensor adapter and restraint allocation. The inspection break-even upper for all this drivetrain wiring together is 1.662 kg after installing the elevated rear host. The four finished power paths use the same conductor gauges and fall below the corresponding supply-extension lengths in the preceding electrical screen. A point-in-section fit places two 10 AWG, two 14 AWG and two CAN conductors inside the existing 8 mm route radius. Connector breakout geometry, service reach and bundle behavior around bends still need qualification. The 80 g rear host lead and finish allocation is separate from the power harness.

## Fit the rear camera host

The rear steering motors intruded into both earlier backplates. Install the 23 mm elevated host bridge and machined backplates. The independent material check charges the full new plate, four aluminum risers, four screws, four washers and four nuts, a 10% positive material allowance and 80 g for the two longer data leads, terminations and finish. The 157.101 g upper gives zero credit for displaced aluminum. Five original pieces move, including the guide foot and four board posts. Both replacement struts retain the old aluminum stock and upper lug positions. Both camera data leads retain their functions with new lengths. Rear cabling must still clear the structure and support both cameras. [Repair STEP](../packaging/rear-electronics.step) · [Repair measurements](../packaging/rear-electronics.json)

The accounting map exchanges seven earlier items for 24 revised or added items, leaving 944 modeled identities. Its independent calculation starts with the issued fixed-height baseline and subtracts the original wheel rows and outer-clamp charge there. It leaves all unrelated rows intact, removes the 36 hub screws from their own shop row, zeroes the three fully surrendered drive groups, and charges the complete module, frame, controller, harness, and host changes once. It separately emits the old independent-interval parent plus the rear repair as a control. Reuse the final matched map only with its entire frozen change list. [Physical exchange](rear_exchange.py) · [Matched material accounting](final_map.py)

## What changes the result?

| Configuration | Before rear repair | With rear repair |
|---|---:|---:|
| Selected four corner-specific cuts | +327 g | **+170 g** |
| Uniform 1.00 m supply cuts on all corners | +152 g | −5 g |
| Uniform 1.20 m cuts | −23 g | −180 g |
| Prior long wiring with all three exchanges complete | −356 g | −513 g |
| Prior shorter 12/14 AWG target with all three exchanges complete | +665 g | +508 g |
| Selected cuts, all seven possible winch blanks retained without their mounting sets | −474 g | −631 g |
| Selected cuts, all seven blanks and fourteen sets retained | −1,486 g | −1,643 g |
| Selected cuts, no original shelf stock credit | −593 g | −751 g |
| Selected cuts, previous duplicated charge for the outer clamps | −234 g | −391 g |

The uniform break-even length is **0.9944 m** with the elevated host, under the same local motor and accessory budgets. The previous shorter wiring target uses a different drive gauge and shorter phase extensions. The prior longer and shorter bills remain available for comparison. Gross field mass with the repaired host at the selected upper is 65.663 kg empty and 66.480 kg with one pipe. The existing 65 and 72 kg opening trials retain their own CG, yaw inertia, contact, routing and empty-robot conditions. The calculated inspection margins are prebuild estimates. A scale reading and the actual four-pad load distribution remain open. [Prior four-module power cases](../power/routes.json)

## Reproduce

Use the selected 927-item swerve map from the original [four-module accounting procedure](../power/README.md). Run from the repository root with Python/OCP and the installed Agent FRC assembly checker. The program verifies the original source map, checks the four-module inventory, reopens four project STEP files and calculates each wiring and stock variant from the registered inputs. The replay writes the matched 944-item selected.json to the specified output directory. The cuts command needs only the public receipt and supplied lengths in metres (FL, FR, RL, RR).

```text
python experiments/swerve-drive/mass-closure/inspection_recipe.py replay . PLUGIN_DIR SELECTED_SWERVE_MAP OUTPUT_MAP_DIRECTORY OUTPUT.json
python experiments/swerve-drive/mass-closure/inspection_recipe.py cuts 0.80 0.95 0.55 0.90 --extra-kg 0.0
python -m unittest discover -s tests -p test_swerve_mass_closure.py
```
