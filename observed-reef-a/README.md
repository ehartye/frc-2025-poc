# Can the shop robot see scoring sector A while carrying coral-A?

The existing fixed head can register the A sector when it can see the surveyed face and a complete subset of its fasteners. At the terminal decision the left-small wheel case is inside an 8.53 mm position bound and the right-small case is inside 7.88 mm. Both heading bounds are 0.74°. These desktop results use the signed, powered station starts and the 85% loaded gyro schedule. The earlier [two-pipe match replay](../conditioned-match/replay.html) retains its original observers and stops.

[Open the student camera replay](viewer.html). Select the wheel case and field input, then move the exposure slider. The two panels show the fasteners the camera measured at that instant.

## What the field camera can actually see

The close approach frequently exposes both fasteners to one pupil and only one to the other. The control uses a 36h11 ID10 for coarse registration, keeps each head's native occurrence and pixel identity, and checks a padded, robot-depth-tested window around its measured face before accepting a centroid. Two distinct identified fasteners can transport the moving estimate. The final gate additionally requires two complete features in one eye and a corroborating measurement in the other. The planar pose interval uses the pixel rays of the entire complete eye and adds separate survey and stopping allowances. No image uses an interpolated fastener for an occluded one. [Field trials](field-trials.json) · [January field and source geometry](../sector-alignment/scene.json)

The robot receives 35 ms captures after 15 ms and integrates shaft travel at 5 ms. The three fresh A permits arrive at wall 47.827 s on the left-small path and 47.792 s on the right-small path. Both chassis stop within about 1.5 mm of the intended center in the simulated field frame. Labelled dropout, duplicate candidate, 75 ms stale packet and wrong face each inhibit both signed approaches. A common 14 mm survey displacement admits a false positive on both. Before transferring the 10 mm / 1° gate, survey the installed field and the loaded camera independently.

At the A approach, accounting for 10 mm planar displacement and 1° heading error leaves at least 135.06 mm to the occupied reef ball. The bound uses the 991-body stowed register, two actual guide plates, the finite carried-pipe enclosure and 629 optional fixed-harness boxes. The student robot stays stowed throughout this approach. The common power calculation retains the two-camera host, rack hold and both grip controllers. It allows up to 8 A phase at each grip and 0.032 m/s surface speed throughout carry. [Native stock bounds](stock.json) · [Six-robot, installed stock and bus](field.json)

All twelve signed sector wheel histories repeat byte for byte after substituting the 998-body machined receiver raster with its painted reference marks and fixed harness. The machined stock check reopens the 998 bodies plus both fabricated guides. The guide remains closest to the mounted ball. Filling the complete deformed shelf and pressure-pad bounding box with a dark opaque surface expanded by 2 mm in each direction also repeats all twelve histories. This tests field-landmark occlusion. The separate roof witness depends on the shelf surface behind the pipe. [Opaque shelf challenge](machined-guard-field-trials.json) All results concern the stowed rack at A. [Machined field captures](machined-field-trials.json) · [Machined stock](machined-stock.json)

## Can the mechanism keep this particular pipe?

A field localization permit and a pipe-custody permit come from separate cameras. The roof camera observes the tube ends every 17 ms. The images arrive after 15 ms and a 5 ms control loop commands the left and right belt cartridges individually. A lateral yaw difference changes the relative belt speeds. The common speed corrects longitudinal drift. The actuator retains the registered cartridge preload, finite end-wall footprints and per-cell friction. The physics carries the original pipe, its rates and angular momentum from the signed pickup.

| Desktop challenge | Outcome |
| --- | --- |
| Effective shelf, continuous trim, 125 and 62.5 µs | Both signed starts reach the sector at 48.162 s. Each has 1,244 accepted roof samples and at least 10.71 mm clearance to the guide. |
| Machined shelf, two dampers and 13 retained modes, capped quantized belts | Each independent arrival preserves 1,244 roof witnesses to stowed rest at 48.162 s. Peak plate deflection bounds are 0.02421 and 0.02454 mm. The respective idler errors peak at 0.03313 and 0.03232 mm. [Joined cases, last pipe states and clock](outcomes.json) |
| Machined station, nominal painted roof and fixed stereo | Both starts have five supported fixed-eye frames and six overlapping roof frames before the initial departure. The optical witness depends on contrast and registration. |
| Conservative 0.05 mm displacement mask or a matched-brightness finish | Neither start has a qualified initial roof witness. A position-only classification without a measured yaw interval also inhibits on the first serviced frame. The controller holds the rack stowed at the station. If granted the earlier nominal handoff and then challenged, it inhibits on the first delivered observation at Java 5,475 ms. |
| Both belt surfaces jam | The independent idler following errors latch at wall 28.642 and 27.757 s. The chassis stops at 29.652 and 28.522 s, respectively. Center travel after the inhibit is 0.294 and 83.379 mm on the registered, finite wheel brake. |

Each idler has 4,096 angle counts per 60 mm of belt travel. A 200 Hz pulse-position controller waits for a full count of accumulated error, limits the pulse to 6 mm/s, and latches a 0.25 mm following error. On the upper modeled rotor inertia the old 26.25 m/s² slew needs 13.921 A. Capping slew at 2.4 m/s² requires up to 4.625 A, within the modeled 8 A limit. This bound assumes up to 0.25 m/s of contact approach speed, 1.4 mm spring travel, 0.40 friction and 0.4 N extra drag. The raw trajectories provide 1 ms samples for comparison. At 125 and 62.5 µs, both shelves and both signed routes reach stowed rest with the same 1,244 delivered custody decisions. Across the common trajectory, the largest sampled center difference is 0.074 mm and the largest axis difference is 0.033°. On the machined configuration the maxima fall to 0.060 mm and 0.023°. [Two-step trajectory comparison](refinement.json) Contact patches, body survey and the plate mode basis do not undergo a three-level joint refinement here. Idler traction, breakaway torque, gearbox backlash, roof calibration, shelf/pipe properties and continuous speed under impact still need measurements. [Actuator envelope](effort.json) · [Jam and wheel brake](cargo-stops.json) · [Same-pipe settling after inhibit](jam-settle.json) · [Optical overlap](station-bridge.json)

A separate [four-land elastic support study](https://github.com/ehartye/frc-2025-poc/pull/117) moves the two 1 mm guide clearances independently. On the same signed approaches, the left-side Java trial faults at 1,020 ms. The right-side trial seats at 1,360 ms at both tested timesteps. The two source states used here come from the earlier coupled receiver. The four-land record ends at Java 1,700 ms. A right-side transfer needs the complete 21 modal positions and rates propagated to the departure epoch, together with the pipe, dampers and controller.

The earlier fixed-clock release at 48.010 s precedes the new chassis stop. The two legacy guide faces remain the studied optical geometry. A separate native shop check found the moving cable enters the negative-Y guide at 327.801 mm of slide extension. The replacement supports require their own camera, moving-cable and contact checks. This study authorizes zero placements. The first pipe remains reserved while held, the second human feed stays unissued, and LEAVE supplies the conditional three-point subtotal. The separate south reservation retains a five-point LEAVE/PARK opportunity. The drive uses a 20:1 gear ratio and assumes 75% of wheel load on the outer axles. A legal station start, actual tire loading, traction and battery impedance remain independent acceptance conditions.

## Reproduce

Use the project Python environment, registered field assets, and the original two signed pickup and drive traces produced by the [conditioned match](../conditioned-match/README.md).

```sh
export OPENBLAS_NUM_THREADS=1
python observed-reef-a/station_bridge.py "$ARTIFACTS" elastic-carry "$FIELD_STEP"
python observed-reef-a/trials.py "$ARTIFACTS"
python observed-reef-a/trials.py "$ARTIFACTS" --machined
python observed-reef-a/trials.py "$ARTIFACTS" --shelf-guard
python observed-reef-a/native_bounds.py "$ARTIFACTS" "$LAMP_STEP"
python observed-reef-a/native_bounds.py "$ARTIFACTS" "$LAMP_STEP" --machined
python observed-reef-a/stock.py "$ARTIFACTS"
python observed-reef-a/stock.py "$ARTIFACTS" --machined
python observed-reef-a/field_screen.py "$ARTIFACTS"
python observed-reef-a/retention.py "$ARTIFACTS"
python observed-reef-a/retention.py "$ARTIFACTS" --fine
python observed-reef-a/retention.py "$ARTIFACTS" --quantized --stuck
python observed-reef-a/retention.py "$ARTIFACTS" --quantized --limited
python observed-reef-a/retention.py "$ARTIFACTS" --quantized --limited --elastic elastic-carry
python observed-reef-a/retention.py "$ARTIFACTS" --quantized --limited --fine
python observed-reef-a/retention.py "$ARTIFACTS" --quantized --limited --elastic elastic-carry --fine
python observed-reef-a/refinement.py "$ARTIFACTS"
python observed-reef-a/retention.py "$ARTIFACTS" --quantized --elastic elastic-carry --head roof-050
python observed-reef-a/retention.py "$ARTIFACTS" --quantized --limited --elastic elastic-carry --head roof-yaw-dropout
python observed-reef-a/field_screen.py "$ARTIFACTS" --aborts
python observed-reef-a/jam_hold.py "$ARTIFACTS"
python observed-reef-a/effort.py "$ARTIFACTS"
python observed-reef-a/outcome.py
python observed-reef-a/student.py
```
