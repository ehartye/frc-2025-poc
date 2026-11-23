# Routing the lower-left guide arm around the docking camera

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

[Rotate the three guide assemblies](compare.html). The straight arm blocks a surveyed field fastener. Lifting its guide attachment restores the view but leaves too much unsupported liner. The orange arm in the third assembly rises above the sightline, then returns to the original guide fasteners. Both signed docking approaches reach sector A with this arrangement.

| Lower-left mount | Smaller left wheel | Smaller right wheel | Guide support |
| --- | --- | --- | --- |
| Original straight arm | Stops 75.0 mm short | Stops 27.5 mm short | Face attachments at Z687 and Z787 mm |
| Raised attachment | Dock permitted | Dock permitted | Raising the lower attachment to Z760 increases the lower overhang from 62 to 135 mm. The static screen predicts 1.24–1.30 mm movement under the two original pickup loads. |
| Closed return | Dock permitted at 2.940 s | Dock permitted at 2.905 s | Retains both original attachment heights. Under the same loads the positive-Y face moves 0.088–0.104 mm in the static screen. |

Times start at sector entry on the stowed, fixed-height robot. For the closed return the latest qualified position bounds were 8.53 and 7.88 mm, respectively, at 0.74° heading. Wrong ID, stale frames, missing features and duplicate features all inhibited both wheel cases. With a common 14 mm field-survey error, the right controller admitted a pose 13.66 mm off laterally. Registration between the field and the camera requires independent measurement. These trials exercise the docking controller; neither loaded travel nor a scored placement follows from them. [Rendered field trials](field.json) · [Predecessor and fastener identity](../mounted-guide-vision/README.md)

## What changed on the robot

The existing match-drilled lower root retains its socket access and two-axis reference. From X4 to X75 the closed aluminum tube rises 73 mm and continues forward at Z760. A rectangular return with a 16 mm X width, 10.6 mm lateral depth and nominal 0.9 mm walls follows the back of the guide toward the original lower pocket. It sits 10 mm ahead of the guide center in X. The folded backing, lower wedge, tapped bar and two screws retain their geometry. The six-body exchange puts those five mating articles beside the revised arm for inspection. Exactly one installed article changes on the bill. [Reopen the STEP](route.step) · [Construction](return_arm.py) · [Dimensions and source pins](revision.json)

The native study checked the arm and mating articles against 1,243 registered shapes and 93 harness edges. It also checked the retained mast and wedge contact surfaces, the four setting-pin withdrawals, all eight socket approaches, and the 34 compensated survey probes. The stowed module still has a clear removal path, first 65 mm inward, then 670 mm forward. The new material was divided into 196 conservative cells for this motion. [Installed fit](clearance.json) · [Inspection paths](inspection.json) · [Service sweep](removal.json)

The 2025 project is entirely prebuild simulation. The straight arm with the existing gyro is the lead for the next complete match replay. The closed return is a fallback for the original camera-only observer. The following fixture and scale checks are for a future build.

The finished assembly needs closed torsion paths at the rising tube and return. The ideal closed front arm yields 0.0853 mm/N at its lower attachment, compared with 0.0311 mm/N for the straight arm. Slitting the torsion walls puts the linear model far outside its useful range. Measure the tab, top joint and lower nose under reversed loading, record joint slip, and survey the liner after unloading. Check lateral travel near the low overhang under the load heights recorded in the simulation. The modeled lower and upper reactions in the original comparison reach approximately 0.75 N and 2.25 N. Fixture measurements must establish the installed stiffness, damping and joint integrity.

The CAD adds 6,344 mm³ of aluminum. The budget includes up to 5 g of additional joint and finish allowance. Matched aluminum stock adds an estimated 16.49–23.08 g. Independent material ranges allow an upper addition of 30.23 g. With the complete old hoist charge removed and zero shelf credit, the latter leaves **51.77 g** in the modeled upper budget. Keeping the old hoist charge produces a deficit. All eight installed-plugin checks retain 1,189 identities. The completed robot still needs to be weighed. [Mass comparisons](accounting.json)

## Pickup in the returned-arm configuration

The asymmetric model uses the closed return on the positive-Y side and the straight tube on the negative-Y side. Replaying the previous station trajectory against the new stiffness would skip the feedback from the moving guide into the pipe. Instead four runs advance both guides, the two carriage stages, the pipe, plungers and original Java recovery controller together. At 31.25 and 15.625 µs, the left approach reaches the seated state at Java 1,340 ms and the right at 1,360 ms. Both stay seated through 5,460 ms. The largest modeled guide travel is 0.1211 mm. The altered arm clears the pipe envelope by at least 6.64 mm under the stored-pose interpolation. [Model, controls and reproduction](mechanics/README.md) · [Terminal states](mechanics/joined.json) · [Pipe and arm separation](pipe-gap.json)

| Station evidence | Smaller left wheel | Smaller right wheel |
| --- | --- | --- |
| Fixed camera, both timesteps | 5 of 5 position observations | 5 of 5 position observations |
| Roof camera, 0.05 mm guard, both timesteps | 6 of 6 inside the custody window | 0 of 6 inside the custody window |
| With rear edge and background at matching brightness | Rear edge missing | Rear edge missing |

Each of the four optical trials uses the actual pipe and guide states from its own timestep. The last exposure at 5,457 ms arrives after the 5,460 ms departure decision. A controller can therefore use at most the first five exposures at that decision. The right case must hold even though the physical model seats the pipe. These results keep the mechanical route available as a fallback; the straight arm and gyro lead the full-match simulation. [Registered exposures](mechanics/station-frames.json) · [Fixed and roof images evaluated at each exposure](station.json)

## Why a narrower tube fell short

Eighteen shapes were compared on the same approaches. A 36 mm downward shift intersects the left gripper motor. Several narrow plates restored the right approach while the left still stopped. At 1.820 s on the left approach, substituting the shallow plate’s left image alone reproduced the ambiguity. The right-image-only substitution retained both usable features. The competing left-image detection survived with both field fasteners hidden in the render. [Same-pose pupil substitutions](pair-audit.json). The shallow plate also overlaps the folded backing. The deeper closed return clears the mechanical envelope but still loses the left docking view. Rechecking both cameras in the closed-loop drive mattered even after the dominant blocker was identified. [All layouts](layouts.json)

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

To reproduce the joined contact and camera checks, run the four mechanical trials and the handoff in [the mechanics instructions](mechanics/README.md). Then run the following from the project root, substituting the SHA-256 of the registered packet reported by the handoff.

```sh
python guide-arm-route/station_view.py <artifacts> <field.step> <packets.json> <artifacts>/guide-arm-route/front-return <packet-sha256>
python guide-arm-route/pipe_gap.py <raw-joined-trials>
```
