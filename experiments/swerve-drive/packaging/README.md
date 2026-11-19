# Four MAXSwerve corners for the fixed-height robot

<!-- cad-still -->
[![Four-module swerve frame and mounting hardware](../../../docs/robot/cad/assembly.png)](../../../docs/robot/cad/index.html)

**Project CAD · Four-module swerve frame and mounting hardware.** Original robot structure for the swerve candidate. The project-local Fusion assembly adds four manufacturer module occurrences. [Source STEP · f8b75f2df4](swerve-frame.step) · [Rotate the model](../../../docs/robot/cad/index.html)
<!-- /cad-still -->

Students can open the [editable Fusion saddle](tab-saddle.f3d), change a stock or bore dimension and inspect the four machining features. The [79-body chassis STEP](swerve-frame.step) contains the original chassis, cranked inner rails and made mounting articles. The local Fusion robot assembles those parts with four manufacturer modules. Its [reopening receipt](student-archive.json) preserves the four placements and the measured centers of the molded tires. The project retains the commercial archive locally. [Student CAD guide](../../../cad/FUSION.md)

## Module and coordinates

| Corner | X forward, mm | Y left, mm |
| --- | ---: | ---: |
| Front left | 282 | 277 |
| Front right | 282 | −277 |
| Rear left | −282 | 277 |
| Rear right | −282 | −277 |

The 564 mm wheelbase and 554 mm track are steer-axis distances. Each free tire has a 38.1 mm modeled axle height. The low 12-tooth pinion drives the 22-tooth intermediate and 15-to-45 bevel pair, giving 5.50 motor turns per wheel revolution. A stationary wheel therefore contributes one drive motor turn per full azimuth turn. Validate motor and absolute-sensor signs per corner. The module has a full-size NEO drive, NEO 550 steer, two SPARK MAX controllers and a Through Bore Encoder. The [shared geometry contract](module.json) defines the right-handed supplier-to-robot transform for each corner. [Motion](../motion/) · [Power, weight and controller mounting](../power/)

REV's current product record identifies the V1.1 shaft adapter, key and encoder support update and the V2 molded wheel. Its assembled downloadable STEP is dated November 2022. OpenCascade resolved 72 physical leaf occurrences. Autodesk Fusion imported 74 nested occurrences and 274 bodies per module. Current spacer, short key and bridge source files and the nominal V2 tire describe the revision differences. The clearance check additionally reserves their full outside volumes. Internal fits and a model of the complete revised supplier assembly require the revision-matched assembly. [REV module](https://www.revrobotics.com/rev-21-3005/) · [Assembly drawing](https://www.revrobotics.com/content/docs/REV-21-3005-DR.pdf) · [Wheel](https://www.revrobotics.com/rev-21-3004/) · [Spare-part revisions](https://www.revrobotics.com/maxswerve-spare-parts/)

## Attach a corner

The original perimeter has inside faces at |X|330 and |Y|325. At each corner the locating rails are 48 mm from the steering center. Each original supplier mounting plate contains six matched Ø5.00 mm holes. In its raw X/Z plane, the two rows are at (−31.75, 57.15 / 69.85 / 82.55) and the transposed positions. The joint uses the first and third hole in each row. The lower and upper tab bearing faces are at Z69.85 and Z120.65 above the floor. Measure each real assembly and machine the saddle height to allow assembly and shimmed bearing before applying clamp load.

| Article per corner | Quantity | Finished geometry and interface |
| --- | ---: | --- |
| Machined aluminum saddle | 2 | 24.5 × 50.7 × 35.7 mm. Two Ø5.30 through bores at 25.4 mm pitch, two Ø6.5 cross bores at 21 mm pitch, two Ø10.7 reliefs. The [native feature receipt](saddle-fusion.json) distinguishes Fusion accuracy settings. |
| #10-32 tab screw and locknut | 4 pairs | 66.68 mm modeled grip-side major-diameter envelope. Fit an actual length with positive locking engagement, sufficient smooth grip and clearance above the plate. The outboard and inboard tabs share vertical loading. |
| M6 side screw and threaded boss | 4 pairs | From the inboard face clamp the saddle into the rail. Bore the inner wall Ø13.10, insert a Ø13 × 13 mm threaded boss flush with that face and weld to the wall. Check thread gauge, wall flatness, boss engagement and tool entry before release. |

Preserve the original central six-millimetre mast webs between X−145 and +145. At each end a closed aluminum web turns inboard 75 mm before returning to the original six-millimetre drilled end. Retain the four end-ear, pilot and fastener sets and their existing telescoping/removal construction at the new Y stations. Each original deck tie and its end tube remains. The four deck clearances give the NEO bodies room above the sheet. The [frame source](frame.py), [reopened part volumes](frame.json) and [retired physical inventory](retirement.json) distinguish replaced stocks, retained ear sets, two reused drive motors and two reused controllers. Feed these identities into the [selected electrical comparison](../power/).

The matched material model adds 1.006 kg nominal before the rear electronics changes. It reserves 1.279 kg at the upper matched bound, or 1.496 kg when taking zero stock credit. The manufactured rail, saddle, boss and screw geometry is independent of the chassis-scale estimate. One saddle carrying an entire adverse corner load puts several kilonewtons into its short fastener pair. Qualify the heat-affected rail, threaded bosses, saddle prying, joint slip, fatigue and blocked-wheel load before assigning a structural rating. [Load and mass bounds](mount-budget.json)

## Space and service

Every moving module component stays inside an analytically continuous steering and wheel-spin reservation. The largest accepted radial envelope is 47.4 mm. The finite native part is clipped against that cylinder where an axis-aligned rectangle would overstate its radius. Bounding boxes for each purchased part and all custom articles are in [envelopes.json](envelopes.json). These low bodies fit inside the existing bumper footprint, keeping the original outer plan dimensions available to the field replay. Final reef, cage, traffic and scoring outcomes belong to the registered wheel-motion/field checks for this configuration. The standing 850 mm bumper outline leaves about 51.7 mm total nominal width in a 35.5-inch cage gap; cage height, approach angle, ropes and other robots can consume that space. This dimension alone grants no transit permission.

The first registered assembly hit the original plates, inner rails, and ears at 26 native pairings. Relieving the deck and moving the rail ends eliminated those while leaving two actual steering-motor hits against the aft electronics plates. The left motor entered the plate by 123.676 mm³ and the right entered the host plate by 455.484 mm³. The fixed data guide and one real host-board post also intersected on the right. Preserve these negative controls when importing the [correction source](rear_fit.py). [Original assembly result](fit.json) · [Host and guide controls](backplate-control.json)

Service begins on stands with the robot isolated, the full cartridge supported, and both removable bumper halves away. Secure the deck and remove the nearest outboard deck tie, washer and nut. Its original hook occupies the tab-saddle withdrawal corridor. The other five deck ties stay in place. Disconnect drive, steering and feedback leads. Remove the four horizontal frame screws, leaving the two tab saddles and their tab screws with the module. Allow 195 mm downward withdrawal. The welded bosses remain in the frame. The [continuous service sweep](service.json) checked 85 carried body reservations per corner. Each cleared the remaining fixtures throughout the 195 mm drop. The smallest radial lower bound to a welded corner fillet was 16.95 mm. For all 16 frame screws a straight 6 mm tool envelope cleared 62 mm of insertion and the head cleared a 40 mm withdrawal. Leaving the matching deck anchor fitted produced 61.6 mm³ of hard interference at 20 mm down at each corner. Wrench-handle swing and loose connector slack need a separate shop check. [Fastener and hook controls](driver-access.json) Recover steering zero, fastener engagement, carpet contact and closed-loop wheel behavior after a change.

## Resolve the rear motors and host computer

The [reopened R2 student archive](student-archive-r2.json) records one revised deck, zero original decks and four position-verified commercial tires. The [second exchange](rear-electronics.step) opens a local pocket in each rear electronics plate. At the right motor it retains the existing converter and carries the host board on a new raised plate. Four M4×40 through screws, four 21 mm bored aluminum spacers, washers and nuts connect the bridge through the original backplate and four full-thickness deck lands. Elevating the Pi and its existing posts by 23 mm also moves the two USB host ports. The fixed camera, two independent camera power outlets, motor positions and stationary upper cable lugs keep their original locations. The two lower guide struts connect the raised pad to those lugs. [Geometry and source bounds](rear-electronics.json)

The raised electronics exchanges clear both native steer motors. The left pocket has at least 1.477 mm of clearance around the motor reservation. The right pocket has at least 1.293 mm. A 0.20 mm static compression witness gave 9.543 mm³ of support material at every bored bearing station. The entire correction adds at most 157.1 g under the no-chip estimate. That includes a separately visible 80 g allowance for longer data leads, terminations and finishes. New lead length, ferrite and bend space, USB signal quality, bolt preload and fabricated clearance require their own acceptance measurements. A camera-ray clearance does not test the two modified host leads.

## Reproduce the native checks

Use the project-supplied 2022 REV assembly and unsimplified field/robot assets locally. Run the existing design's source assembly build first; its optical input uses the documented owner-supplied status-light model. The Python fabrication scripts use the project OpenCascade runtime. Fusion supplies the student-authoring and archive-reopen evidence.

```text
python native_module.py <REV-21-3005.STEP> <assets>/swerve-package/original.json
python extract_interfaces.py <REV-21-3005.STEP> <assets>/swerve-package/vendor-native
python scene_inputs.py <assets> <owner-supplied-status-light.step>
python frame.py . <assets>
python envelopes.py <assets>/swerve-package
python fit.py <assets>/swerve-package
python backplates.py <assets>/swerve-package
python rear_fit.py <assets>/swerve-package
python service.py <assets>/swerve-package FL
python ../../../tests/test_swerve_packaging.py
```

The [source placement](module_placement.py) checks the 4×4 coordinate transforms. Native STEP measures material intersections, returned topology, volumes and datums. The Fusion archive independently checks the student's view of editable geometry. Physical inspection, installed weight and field scoring remain separate evidence.
