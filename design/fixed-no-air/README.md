# Fixed-height robot without match pneumatics

The fixed carriage uses two mast locks for vertical support. Wheel drive, slide motion and both grips are electric. The sole modeled air actuator released the former lift pawl. A dedicated fixed-height match build can omit the compressor, reservoir, two tank saddles, valve and regulator/gauge/dump assembly. The Pneumatic Hub stays on its original CAN and power branches. The ratchet, actuator reserve, ropes, drums, motor, wiring allocations and mechanical locks stay in the robot. The CAD and mass results assume a build with the air circuit absent. Disabling the compressor on a robot that still carries the kit saves no weight. [Fixed carriage](../fixed-height/README.md) · [Published power and isolation study](../../fixed-low-power/README.md)

## Match mass

| Single-height mast configuration | Conservative R103 headroom | R103 headroom with the original air allocation cancelled at matching endpoints | Bumper-on R408 headroom at matching endpoints |
|---|---:|---:|---:|
| High parking build | 2.290 kg | 3.090 kg | 4.662 kg |
| Low receiver, original observer | 2.316 kg | 3.116 kg | 4.688 kg |
| Low receiver, two-camera observer | **1.811 kg** | **2.611 kg** | **4.183 kg** |

The issued mass budget contains one 1.8–2.6 kg line for the removed group. Agent FRC 0.21.0 combined the negative interval with the previous positive baseline independently. It also counted all 21 added camera items alongside the six removals in the two-camera build. Reconstructing each original endpoint cancels 1.8 kg at the low endpoint and 2.6 kg at the high endpoint. The latter calculation places the two-camera robot at **49.552 kg** at the upper R103 estimate. The installed checker leaves the finished margin unknown. It includes provisional hardware and cable allowances. Both physical bumper sets, finished wiring, markings and the fully dressed robot require separate weighing. Present any other match mechanisms with the inspection inventory, including an air kit if a pneumatic configuration is also declared. [Five installed accounting cases](inspection.json) · [Original grouped allowance](../electrical-inspection/inspection.json) · [2025 inspection scope and limits](../shared-height/README.md)

The 156-second electrical replay for the earlier low observer uses zero compressor and lift phase current while the PH draws up to 0.137 A. That replay used one camera. The moving camera requires its own protected converter, wiring and verified load at the rail. The original pneumatic controller also responds to missing pressure with a fault that can retain a support-torque request. A fixed-height match configuration requires independently isolated lift actuation. The no-air design provides no powered carriage-height change. Moving between heights needs a separately supported pit rebuild and a means of servicing the retained ratchet and lines. Neither the installed pins nor the pit pins support travel between elevations. [Branch map](../electrical-inspection/circuits.json) · [Lift control states](../electrical-inspection/lift_interlock.py) · [Camera installation](../../moving-observer/README.md)

## Geometry through the match

The low scoring candidate combines the actual deck-mounted light, keyed deck and spacer with the moving-camera build. The previous 598-body optical robot and this 593-body robot have **590 identical native solids** at each registered stroke of 0, 185 and 370 mm. The changed deck reaches X 337 mm. It remains behind the unchanged bumper at X 425 mm throughout the stroke. The same drilled shelf reaches X 535 mm at full extension. The removed or relocated parts are all on the fixed base. The CAD handoff preserves the identity and forward position of the delivered shelf through retraction. Camera calibration, image backgrounds, contact friction, field attitude and the match controller have separate evidence. A full local viewer can be rebuilt from the supplied signal-light STEP using the command below. [Per-body OpenCascade hashes and front surfaces](whole-robot.json) · [Two-camera geometry and field-stock checks](../../moving-observer/README.md)

For the empty high parking build, removing the air group lowers upper running mass from 65.343 to 62.743 kg. The fore/aft load-line margin at the published straight acceleration decreases from **57.5 to 52.7 mm** as the estimated mass distribution changes. Both tested mass endpoints complete the reserved straight move. The no-air screen calls for 11.40 A peak drive phase current and a minimum traction coefficient of 0.235 under its stated assumptions. The balance calculation carries signed machining losses, the revised deck and lamp allowance, and separately evaluates the low robot with the camera at full extension. The parking calculation is for the empty high configuration. [Distributed mass, load transfer and drive demand](balance.json)

## Pit support

[Open the interactive service detail](service.html) to see the left pin withdrawn, retained and carrying the plate after a modeled 1 mm settlement. The sectioned rail exposes the bushing. Hide the carriage plate or isolate a part to inspect the keeper. The [six-solid service STEP](pit-pins.step) contains the two removable pins, two headed keepers and two locknut envelopes. They are pit equipment outside the match model.

The selected 8 mm pin reaches 10 mm under the carriage plate. With a 3 mm rearward shift of the pin and a 1 mm forward plate error, the model retains 6 mm of overlap. A 2 mm downward test produces material contact on both sides in each pose, including the two-camera low build. The 70 mm pin withdrawal and the keeper insertion path clear the model. The original lateral tool sweep intersects the rail and bushing. Moving it forward clears those parts but intersects a backup line. The revised vertical keeper permits an 8 mm diameter axial tool approach from both ends in the tested poses. Positive contact also blocks forward pin motion at the head and rearward motion at the installed keeper. [Native contact, clearance and adverse placement results](native.json)

At 244.8 N on one pin, the short design has **118 MPa** nominal bending stress at the bushing exit and **179 MPa** at the weaker section through the keeper bore. The long control with the same bore reaches 380 MPa at that bore. The camera adds up to 0.440 kg to the moving stage. These static comparisons include a two-gravity load allowance on one pin. The small cross-hole has an unresolved stress concentration. Select and verify actual materials and fasteners, local bearing, pin/rail joint strength, thread retention, deflection and both-sided support in a captive fixture. Keeper head and nut shapes reserve the fitting space. The design still needs glove, tool and installed loom checks plus a qualified independent support for a height change. [Native sections](native.json) · [Original service rail and load path](../receiver-lift/RETENTION.md)

## Reproduce

From the project root, use the OpenCascade-capable Python, Agent FRC 0.21.0 and the source-pinned [manufacturer light model](../signal-light/README.md). The first command reopens the full robot and service hardware. The next two run the installed accounting checks and the distributed-mass comparison. The final commands recreate the section viewer and the complete removable local robot viewer.

```powershell
python design/fixed-no-air/service.py $env:SIGNAL_LIGHT_STEP
python design/fixed-no-air/account.py $env:AGENT_FRC_PLUGIN_ROOT artifacts/fixed-no-air/maps
python design/fixed-no-air/balance.py
python design/fixed-no-air/view.py
python design/fixed-no-air/whole.py $env:SIGNAL_LIGHT_STEP --viewer artifacts/fixed-no-air/robot.html
python design/fixed-no-air/export_robot.py $env:SIGNAL_LIGHT_STEP artifacts/fixed-no-air/placement.step
```

The STEP export contains the complete 593-solid placement pose, including the supplied product geometry. Keep it with that local source. Two readers reopened 593 solids. The exporter checked every part’s bounds, volume and centroid and compared native material for the modified deck, camera bridge and shelf. All 1,002 light faces matched by surface type, bounds, area and centroid. Of these, 987 had interior grid samples, giving 10,458 point-to-trimmed-face checks in both directions. Use `--pose receive` for the pickup configuration. [Round-trip measurements](exchange.json)
