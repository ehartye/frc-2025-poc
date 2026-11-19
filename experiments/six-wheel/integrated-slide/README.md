# Can both cameras and the rack still see the same shelf?

<!-- cad-still -->
[![Team 1780 · stowed swerve robot](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · stowed swerve robot.** Red alliance R4 with numbered bumper covers. Purchased parts use project-created representations in this public assembly. [Source STEP · abbd8ab838](../../../cad/full-swerve-r4/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

The sensor's new mounting hardware is visible to the fixed camera at full extension. It changes 18 left-eye and two right-eye pixels at each scoring dock. The moving camera's view remains backed by the shelf. Across 3,994 exposure evaluations, the detector returned the same observations as the prior camera study. The newly loaded rack follows a different physical trajectory and still reaches both stow deadlines in 13 of the 19 revised-controller cases. These are desktop results on one registered geometry. [Optical, scheduler and contact results](result.json) · [Native mount drawing](../sp1-mount/shop-drawing.svg)

## Look for an obstruction in each camera

The scene combines the [two registered optical heads and shelf marks](../selected-optics/README.md) with the [SP1-25 installation](../sp1-mount/README.md). Seven reopened metal or fastener solids and three source-dimensioned supplier or cable reservations replace four earlier placeholders. The resulting scene has 991 registered bodies and reservations. The two marks add contrast to verified shelf surfaces. The camera housing and the eye/ferrule have different source quality. The actual eye, soldered wires, cable installation and camera calibration need measurement.

| View | Native depth and image result |
|---|---|
| Fixed head during each station pickup | The isolated new hardware is at least 17.458 mm behind the occupied scene in both eyes. Both station backgrounds and both full 246-frame seat sequences reproduce the issued observations. |
| Moving head at each travel endpoint | The closest isolated hardware lies 14.995 mm behind the foreground backing. A swept check brackets uncertain material through the entire stroke. It retained 144,507 and 144,518 backed rays in the two eyes after removing all potentially blocked reference rays. |
| Fixed head at either scoring dock | Eighteen left and two right hardware pixels reach the visible depth plane at the nominal rectified origin. Re-rendering the finite pipe at each release exposure preserved the measured intervals. Both alternative fixed-origin depth bounds (±0.75 mm), both moving-origin controls and the original attained field poses were checked. |
| Missing, swapped or stale image | In the 70-frame second-pipe control, losing an eye produces 63 unbacked and seven unresolved frames. Swapping eye identities loses the robot reference on every frame. A stale roof transform labels 56 frames outside, two inside and twelve unbacked/unresolved. A single apparent match is insufficient evidence of custody. |

All 19 detector CSVs matched the prior outputs byte for byte. The actual grayscale/depth backgrounds changed and bind to a new observation register. The 28 released packet-policy scenarios reproduced their prior event sequences, including evidence loss and deadline failures. Physical light, reflectance, vibration, camera latency and extrinsics remain unmeasured. [Isolated old and revised part projections](visibility.json) · [Projected material and field pose checks](result.json)

## What load does the slide carry?

The installed Agent FRC 0.22.2 checker applied the raised rails, optical additions and replacement sensor hardware successively. Its direct configuration retains both fixed camera shoes and screws, both marks' finish allowance, all seven SP1 mounting parts and one purchased transducer. The upper inspection estimate is **52.002139 kg**, with **160.984 g** represented headroom. The battery and bumper allowances bring the upper operating mass to 65.672139 kg before a carried pipe. [Assembly and moving-mass calculation](mass.json)

The 5.3571 kg loaded outer-stage basis already includes the original target. Its 5.2 g upper allowance is removed before adding the final clevis, shoulder pin, washer and locknut. The two shelf fastening pairs stay on the slide. The 0.2590 kg roof-shop increment, 0.9071 kg upper allowance for the non-panel cable/chain components, four receiver shims and the marking finish also enter this bound. No credit is taken for the lighter rail cuts. The fixed adapter and optical shoes stay on the chassis.

The manufacturer specifies a 0.19 kg maximum for the complete instrument and bracket. The mass of the moving factory eye and paid-out cable is unresolved. Charging the entire 0.19 kg to the stroke provides a conservative inertia bound while still booking the supplied instrument once in the robot ledger. This adds **1.376792 kg** to the issued plant. Including the historical outer stage and camera allowance gives **7.173892 kg loaded outer mass**. The middle stage and reflected rotor inertia remain in the original equations. If the received eye and paid-out cable fit the issued geometric envelopes and their component densities are at most 8.1 g/cm³, the corresponding moving-end cap would be 18.10 g and the plant addition 1.20489 kg. Weigh the actual eye, stroke-dependent cable and traveling harness, and measure running drag before setting a physical control margin.

## Follow the timing and voltage

The 5 ms WPILib loop, 35 ms acceleration ramp for the revised cases, 300 ms braking ramp and 40 ms acknowledgement deadline remain as issued. The 35 original test scenarios were exercised with this heavier plant. Forty-seven per-pipe trajectory hashes changed. The camera and pressure registration remained bound throughout. [Original calibrated controller](../rack-sensing/README.md)

| Condition | Observed result |
|---|---|
| Calibrated 94% span, nominal drag | Withdraw at 48.295/98.375 s, motion acknowledged 35 ms later and stow at 50.765/100.845 s. Peak rack phase current was 13.96 A. |
| Adverse reading at nominal drag | Both pieces just meet the 40 ms acknowledgement deadline. |
| 42 N drag with adverse reading | The first rack moves 1.363 mm and fails the corroboration gate at 48.335 s. The second piece never receives permission. Open-input and uncoupled-rack cases retain their separate fault evidence. |
| Successful revised paths | 13 cases finish both strokes. Two historical pipe states at two integration resolutions per case give 52 sustained-support results. The minimum front and riser clearances in that contact model are 32.605 mm and 21.406 mm. |
| Shared source under demand | At 11.8 V and 27 mΩ, the limiting right-drive inlet reaches 9.510 V. At 32 mΩ it reaches 9.052 V and crosses the 9.5 V design limit at wall time 0.430 s. The greatest rack phase current among the completed cases is 16.08 A. |

The contact solver uses the original finite-pipe and front-surface model. Transfer to the selected whole front, verified repeatable field pose and actual contact pressure require their own geometry and measurements. The fixed camera's proposed power tail extends above the earlier route hull. Its installation, field clearance, sensor calibration and actual battery resistance remain open. No competition score or robot motion permission follows from this desktop run.

## Reproduce

Use the previously registered project artifacts, official field and signal-light inputs, native geometry and vision dependencies, desktop JDK/Gradle cache and the installed 0.22.2 plugin. The 0.22.1 flattened camera-inclusive map is an input, alongside the archived #93 native depth backgrounds for independent before/after comparisons. Choose a fresh output directory.

```sh
python integrated-slide/replay.py "$ARTIFACTS" "$LAMP_STEP" "$FIELD_STEP" "$OUTPUT" "$JDK17" "$GRADLE_CACHE" "$AGENT_FRC_PLUGIN_ROOT"
python -m unittest discover -s tests -p test_integrated_slide.py
```

This generates the directly selected procurement map, native views, field and exposure checks, HAL inputs and 1 ms rack paths under the output directory. Compact source-bound receipts are written beside the reproduction scripts. Supplier CAD and the field stay outside Git.
