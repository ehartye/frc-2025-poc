# Final scoring-sector approach

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 83b3b9c7fe](../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

The [spatial contact follow-up](../spatial-contact/README.md) carries these signed approach states through withdrawal, scoring assessment and parking. [Watch the pipe land](../spatial-contact/replay.html).

[Open the student replay](replay.html). Move the robot 35 mm off its route, choose either reef sector, or inject a bad observation. The view plots the measured chassis path and the time at which the controller can decide.

**All ten healthy software trials entered the 10 mm position disk and 1° heading window before extension.** The affirmative result applies to the 593-body fixed-low robot with its X48 moving camera and the existing fixed stereo. Under the light and calibration assumptions below, the complete deployed robot retains at least 16.48 mm clearance to occupied stock at A and 24.89 mm at B. The two independent pipe continuations reach first trough contact. The revised route still lacks a complete two-score qualification.

| Input to the terminal approach | A | B |
|---|---|---|
| Centered at the start of the 942.16 mm straight | Permit | Permit |
| 35 mm left and +2°, or 35 mm right and −2° | 2 permits | 2 permits |
| 21 mm normal and 28 mm lateral, differential wheel scale 0.998/1.002 | Permit | Permit |
| Opposite diagonal offset, 1.5° yaw, 12 mm side slip over 120 ms | Permit | Permit |
| Wrong tag ID, duplicated bolt candidate, hidden bolts, or 75 ms camera delay | 4 holds | 4 holds |
| Both landmarks shifted 14 mm in the field survey | False permit | False permit |

Latest accepted optical bounds across the healthy trials were at most 9.23 mm / 0.74° at A and 9.18 mm / 0.74° at B. All first permissions arrived between 2.905 and 2.975 s, before the 3.272 s extension deadline. Fresh images continued to support each permission until that deadline. Nominal A started this approach at match wall 41.737 s and entered the gate at 44.677 s. The B clock is a separate rehearsal at its issued second-trip time. [Complete trial outcomes](results.json) · [Field clock](clock.json) · [Native clearance and fit tolerances](../bridge-tolerance/README.md)

## What the robot observes

The January red reef STEP locates ID10 in occurrence NAUO1071 and its two exposed fasteners in NAUO1187/1192. The 220 mm square optical opening has zero measured material overlap in the 3.397 mm front slab of the fascia. The nominal black border is modeled at 165.1 mm. The bolt-face centers are 234.95 mm apart, at approximately Z425.32 mm. The marker association uses the 36h11 ID, the correct winding, two independent eyes and a stereo reprojection threshold. The controller then associates a unique ordered pair of bolt images with that registered face. Both lenses decode the tag with 600 mm to go. At the dock the robot or carried-pipe envelope prevents a complete tag reading in these images, while both existing bolt faces remain observable. [Measured primitives and native input hashes](scene.json) · [January field model](https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025Field-STEP.zip)

A new render reopens the stowed 593 solids, including the supplier light and actual aft bridge. The carried tube is conservatively enclosed by a box spanning its allowable position and yaw. The renderer projects antialiased tag and bolt patches onto a field-face backdrop, depth-tests them against the robot and tube, and returns 640 × 480 grayscale images for both eyes. OpenCV 4.12 decodes the tag. Grayscale boundary and centroid fits measure the field points. Ground-truth chassis pose is used by the renderer, vehicle plant and independent error checks. The controller receives decoded pixels, stamped deliveries and wheel-encoder displacements. All 24 staged field pieces clear the conservatively enclosed sight corridor by at least 164.65 mm. The other five scheduled robots clear it by at least 1.51 m. [Sensor and interval solver](vision.py) · [Native construction](native.py)

The 5 ms drive loop receives paired captures every 35 ms with 15 ms nominal transport. It transports each estimate from the actual capture time using differential wheel increments quantized to 0.158 mm. A steering curve is fitted when the tag and observed bolts establish a usable start. Wheel speed reaches 0.749 m/s and acceleration remains at or below 1.198 m/s² in the healthy cases. Three distinct settled frames are required. Each accepted terminal centroid must have a total pixel-coordinate error of at most ±0.18 px. Some early steering observations had larger residuals. The terminal check intersects all ray constraints over the full heading circle in 0.1° bins, retaining correlated XY possibilities. It then adds 1.85 mm / 0.14° for shared calibration, survey and stopped-motion effects, plus encoder transport since capture. The largest observed terminal residual was 0.021 px. The position and heading allowances are build acceptance budgets. Actual tilt, flex, exposure, contrast and synchronization need independent measurement. [Drive replay](dock.py)

Shifting the tag and fasteners together preserves their internal geometry. With a 14 mm common survey offset, the robot accepted a physical 13.86–13.89 mm lateral miss at the two sectors. Survey both landmarks against the scoring lip and verify loaded camera extrinsics. Local optical agreement cannot expose that common offset. Duplicated features, a missing pair, stale packets and a different marker all held the slide stowed in the registered tests.

## What the match can credit

The physical replay starts from the identified station pipe and its achieved port-control state. It follows the actual wheel response through the new final approach and uses the attained pipe translation and spin at release. The older free-flight model held the shelf at nominal field X even after the robot stopped about 1.2 mm short. This produces an immediate false shelf recontact. The separate chassis-frame continuation uses the actual shelf and the stationary field trough. Both pipes clear their respective shelves by 1 mm after 22.25 ms and reach first trough contact after 106.90 ms. The final field-transverse velocities are +0.04316 mm/s at A and −0.01422 mm/s at B. Both exceed the planar contact model’s 0.0100 mm/s magnitude limit. A new registered optical release/actuator trial and sustained assessment are still pending for these shifted poses. [Finite-pipe release and historical control](pipe.json) · [Earlier nine-point match with its original approach](../integrated-match/README.md)

The clock therefore keeps the slide retracted and can choose park after A. The alternate starts at wall 45.010 s using relative wheel commands from the admitted dock, reaches park at **54.723 s**, and follows the same five moving partners and opponents. The raw nominal stock margin on the parked route is 9.04 mm. The closest moving robot is 502.67 mm away, and the cage has 109.44 mm residual lateral allowance under the inherited tracking comparison. The front bumper overlaps the park zone by 335.62 mm. The pipe remains on the stowed shelf through wall 156 s in the physical replay, with zero displacement over the final second. The resulting modeled contribution is **5** for LEAVE and PARK. A blind stop at the reef retains **3** for LEAVE. No second station pipe is dispatched in this policy. The most conservative stowed whole-volume stock margin across the tested terminal paths is only 0.83 mm after the 8 mm intersample allowance. A specific field survey and drivetrain tracking check are important here. [Park custody](park-custody.json) · [Traffic, stock and clock](clock.json)

## Reproduce

Run these commands from the project root with the existing scientific Python/OCC/VTK dependencies, NumPy and OpenCV Contrib 4.12.0.88. The public project stores the derived feature measurements. Provide the independently obtained field and signal-light STEP files matching the receipt digests. Native images, exact 5 ms traces and screenshots belong in ignored artifacts.

```sh
python sector-alignment/native.py "$FIELD_STEP" "$SIGNAL_LIGHT_STEP" artifacts/sector-alignment
python sector-alignment/dock.py artifacts/sector-alignment
python sector-alignment/cargo.py artifacts/sector-alignment
python sector-alignment/clock.py artifacts/sector-alignment
python sector-alignment/view.py
python -m unittest discover -s tests -p test_sector_alignment.py
```

The simulations use a flat field, the issued drivetrain and seated tube, and assumed surveyed optical contrast and calibration. The controller and sensor model run offline in Python. The earlier desktop WPILib rack and source acquisition records remain separately versioned. Field observations, real combined processing latency, loaded traction, survey error and post-contact three-dimensional behavior remain unmeasured.
