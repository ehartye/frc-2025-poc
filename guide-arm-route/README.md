# Routing the lower-left guide arm around the docking camera

[Rotate the three guide assemblies](compare.html). The straight arm blocks a surveyed field fastener. Lifting its guide attachment restores the view but leaves too much unsupported liner. The orange arm in the third assembly rises above the sightline, then returns to the original guide fasteners. Both signed docking approaches reach sector A with this arrangement.

| Lower-left mount | Smaller left wheel | Smaller right wheel | Guide support |
| --- | --- | --- | --- |
| Original straight arm | Stops 75.0 mm short | Stops 27.5 mm short | Face attachments at Z687 and Z787 mm |
| Raised attachment | Dock permitted | Dock permitted | Raising the lower attachment to Z760 triples the lower overhang. The static screen predicts 1.24–1.30 mm movement under the two original pickup loads. |
| Closed return | Dock permitted at 2.940 s | Dock permitted at 2.905 s | Retains both original attachment heights. Under the same loads the positive-Y face moves 0.088–0.104 mm in the static screen. |

Times start at sector entry on the stowed, fixed-height robot. For the closed return the latest qualified position bounds were 8.53 and 7.88 mm, respectively, at 0.74° heading. Wrong ID, stale frames, missing features and duplicate features all inhibited both wheel cases. With a common 14 mm field-survey error, the right controller admitted a pose 13.66 mm off laterally. Registration between the field and the camera requires independent measurement. These trials exercise the docking controller; neither loaded travel nor a scored placement follows from them. [Rendered field trials](field.json) · [Predecessor and fastener identity](../mounted-guide-vision/README.md)

## What changed on the robot

The existing match-drilled lower root retains its socket access and two-axis reference. From X4 to X75 the closed aluminum tube rises 73 mm and continues forward at Z760. A rectangular return with a 16 mm X width, 10.6 mm lateral depth and nominal 0.9 mm walls follows the back of the guide toward the original lower pocket. It sits 10 mm ahead of the guide center in X. The folded backing, lower wedge, tapped bar and two screws retain their geometry. The six-body exchange puts those five mating articles beside the revised arm for inspection. Exactly one installed article changes on the bill. [Reopen the STEP](route.step) · [Construction](return_arm.py) · [Dimensions and source pins](revision.json)

The native study checked the arm and mating articles against 1,243 registered shapes and 93 harness edges. It also checked the retained mast and wedge contact surfaces, the four setting-pin withdrawals, all eight socket approaches, and the 34 compensated survey probes. The stowed module still has a clear removal path, first 65 mm inward, then 670 mm forward. The new material was divided into 196 conservative cells for this motion. [Installed fit](clearance.json) · [Inspection paths](inspection.json) · [Service sweep](removal.json)

The finished assembly needs closed torsion paths at the rising tube and return. The ideal closed front arm yields 0.0853 mm/N at its lower attachment, compared with 0.0311 mm/N for the straight arm. Slitting the torsion walls puts the linear model far outside its useful range. Measure the tab, top joint and lower nose under reversed loading, record joint slip, and survey the liner after unloading. Check lateral travel near the low overhang under the load heights recorded in the simulation. The modeled lower and upper reactions in the original comparison reach approximately 0.75 N and 2.25 N. Fixture measurements must establish the installed stiffness, damping and joint integrity.

The CAD adds 6,344 mm³ of aluminum. The budget includes up to 5 g of additional joint and finish allowance. Matched aluminum stock adds an estimated 16.49–23.08 g. Independent material ranges allow an upper addition of 30.23 g. With the complete old hoist charge removed and zero shelf credit, the latter leaves **51.77 g** in the modeled upper budget. Keeping the old hoist charge produces a deficit. All eight installed-plugin checks retain 1,189 identities. The completed robot still needs to be weighed. [Mass comparisons](accounting.json)

## Why a narrower tube fell short

Eighteen shapes were compared on the same approaches. A 36 mm downward shift intersects the left gripper motor. Several narrow plates restored the right approach while the left still stopped. On the shallow plate the right-eye view of a partially hidden fastener changed during the left approach; at 1.820 s the observer lost the two usable left-eye features under the ambiguity check. The shallow plate also overlaps the folded backing. The deeper closed return clears the mechanical envelope but still loses the left docking view. Rechecking both cameras in the closed-loop drive mattered even after the dominant blocker was identified. [All layouts](layouts.json)

## Reproduce the checks

Use the project-pinned CAD, NumPy, and OpenCV environment. Keep generated field assets, complete rasters, selected maps and replay paths in the project’s external artifact directory. The field STEP is the project-supplied registered input. These commands use the default canonical optical observer in this checkout.

```sh
python guide-arm-route/arm_route.py <artifacts>/guide-arm-route rise-+0 rise-+73 front-return
python guide-arm-route/clearance.py <registered-scene.json> <artifacts>/guide-arm-route rise-+0 rise-+73 front-return
python guide-arm-route/field_view.py render <artifacts> rise-+0 rise-+73 front-return
python guide-arm-route/field_view.py trace <artifacts> rise-+0 rise-+73 front-return --modes nominal wrong-face dropout stale ambiguous shared-survey-shift
python guide-arm-route/service_path.py <registered-scene.json> <artifacts>/guide-arm-route/front-return
python guide-arm-route/inspection.py <artifacts>/guide-arm-route/front-return
python guide-arm-route/accounting.py <installed-plugin> <match-drilled-map-directory> <artifacts>/guide-arm-route/front-return
python guide-arm-route/student_view.py <artifacts>/guide-arm-route
```

For the complete comparison, use the layout identifiers in `compare_layouts.py` when generating geometry, checking fit and rendering both signed routes. Its collector checks the exchanged geometry against the optics and clearance receipts before extracting the nominal results. Pass `--installation outward` to both optical commands to challenge the same arm with the earlier loose-bolt fit. The installed lower pilot is intended to prevent this shift after the guide has been correctly match-drilled.
