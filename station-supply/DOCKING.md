# How close should the robot park?

Open the [docking lab](dock-lab.html), select a receiver position and turn the robot 1° while moving it 5 mm toward the wall. Compare the square bumper in plan with the pipe in the centered side view. The minimum parking gap and the space available for a pipe to enter the receiver answer different design questions.

| Nominal bumper gap | Receiver stroke | Result at the station | Result for the centered pipe |
| ---: | ---: | --- | --- |
| 10 mm | 240 mm | The foam and cover intersect the station at 1° yaw and 5 mm closer than intended. | The issued centered flight reaches shelf height. |
| 40 mm | 240 mm | At least 8.34 mm of space across the tested parking errors, rounded to two decimals. | The pipe meets all three pressure pads at 49.31 ms, before the assumed shelf contact. |
| 40 mm | 270 mm | The same 8.34 mm lower bound applies across the parking errors. | Moving the shelf forward 30 mm restores the centered landing. The pipe is tangent to the grip belt. |

The bounds cover all combinations of ±10 mm in each horizontal direction and ±3° of chassis heading, with the receiver extended at 940 mm lift height. They enclose 545 red robot bodies and 47 registered station occurrences. In the longer-stroke candidate the full 0–270 mm extension sweep clears other robot bodies at that height. The operating sequence, loads and approach route for the changed dock still need verification. [Issued pose study](dock-tolerance.json) · [Longer-stroke study](compensated-dock.json)

The longer-stroke receiver still needs a guiding or alignment solution. At the modeled 1.697 m/s exit speed, eight curved-flight enclosures leave a minimum 0.249 mm to hardware other than the belt and intended shelf, allowing up to 1.5 mm of grip carriage travel. With 1 mm side error and 0.25° heading error, the first sampled overlap is with a belt at 20.92 ms. With 10 mm and 3° in the same direction, the pipe reaches the rigid angle board at 10.96 ms on either side. Even the smallest combined registration error studied leaves the rigid-hardware clearance unresolved. The belt's response, chute rebound, incoming spin and achievable parking errors need measurements before the team can count a repeatable feed. [Feeder assumptions and contact model](README.md)

## Locating the jaw stops

On the 40 mm dock with 270 mm extension, move each original inboard stop 1.00 mm outward on its front support foot. The 4 × 2 × 16 mm blocks retain their complete 8 mm² footprints and their original volume. The lug touches the relocated stop at 0.50 mm inward. A 0.75 mm inward displacement overlaps it by 7 mm³ on each side. The outward stop stays at 1.50 mm. Allow ±0.10 mm on the installed travel limits for this study. The downloadable pair is in the robot frame at the receiving pose. [Two relocated stops in STEP](receiver-stops.step) · [Survey and native results](entry-window.json)

| Depth error | Side error | Heading error | Minimum clearance to rigid hardware | Maximum outward belt movement required by the envelope |
| ---: | ---: | ---: | ---: | ---: |
| ±5 mm | ±0.35 mm | ±0.15° | 0.477 mm | 0.694 mm |
| ±5 mm | ±0.50 mm | ±0.25° | 0.113 mm | 1.074 mm |
| ±5 mm | ±1.00 mm | ±0.25° | Unresolved at the cheeks | 1.574 mm |

The errors are independent and refer to the center of the incoming pipe at nominal shelf height. Eight convex prisms enclose the entire curved flight of the transverse pipe at the modeled chute exit speed, including its full diameter, length, and every allowed registration error. Each grip can travel inward by up to 0.60 mm. The intended belt and shelf contacts are evaluated separately from rigid hardware. At 0.50 mm side error and zero yaw, the issued fully inward grips intersect the pipe shell at the upper cheek by 0.193 mm³ at 15.44 ms on the assumed ballistic path. The relocated stops clear this path. A real belt can change the path earlier.

The [student lab](dock-lab.html) converts two sideways readings 200 mm apart plus an independent depth reading into bounds at the receiver. Enter the total uncertainty of each reading, including registration of the measuring device. The two simple stops still require an attachment detail, an as-built survey and a check that the belt on the far side can reach a short or off-center pipe. The entry window covers flight through shelf height. The seated state, later slide travel, and subsequent scoring need rechecking with the changed grip limits and the actual pipe spin.

## Reproduce the geometry studies

Supply the January field STEP used in the [source receipt](transfer.json). From the project root, use the CAD Python environment that can import OCP.

```powershell
python station-supply/dock_tolerance.py --field $env:FIRST_FIELD_STEP --poses design/competition-package/shop-duplex/red --output station-supply/dock-tolerance.json
python station-supply/compensated_dock.py --field $env:FIRST_FIELD_STEP --assembly design/competition-package/shop-duplex --output station-supply/compensated-dock.json
python station-supply/entry_window.py --step station-supply/receiver-stops.step --output station-supply/entry-window.json
python station-supply/build_dock_lab.py
```
