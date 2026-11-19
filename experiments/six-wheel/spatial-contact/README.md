# Can we score after correcting the approach?

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

The [second station feed](../return-station/README.md) now replays these five achieved returns, the real port sequence, and the independent stereo custody clock. [Try the receiver](../return-station/replay.html).

**All five modeled approaches to sector A delivered the first pipe and parked for seven role points.** The robot earns three for leaving during AUTO, two for L1 CORAL and two for parking. Park finishes at wall 60.478 s. Both the kickoff and later rule editions count the pipe at wall 156 s, three seconds after the match ends. The second sector independently supports a delivery in the same five approach conditions. The second station feed still needs a trial from the position the robot actually reaches. [Interactive contact replay](replay.html) · [Field outcomes](match.json) · [Earlier approach and five-point fallback](../sector-alignment/README.md)

| First-trip evidence | Result across five starts |
|---|---|
| Robot motion leading into the 942 mm final approach | Centered, opposite lateral offsets, wheel scale bias, and a 12 mm side disturbance. The field and cargo models apply the approach entry offset over two seconds. |
| Sensed withdrawal | A command at 48.295 s and confirmed stow at 50.765 s under the 15 ms camera delivery assumption. The independent B commands occur at 98.390 and 100.860 s. |
| Separation after first field contact, across both sectors | At least 36.24 mm between the complete robot front and the nearest pipe extent. The nearest inner riser stays over 15 mm away on the conservative X-plane check. |
| After stow on the first trip | At least 498.76 mm to another modeled robot, 70.76 mm to the scored pipe, 24.67 mm to fixed field obstructions, and 8.86 mm to installed stock during the park route. |
| Return toward the second station | Applying the same wheel demands at the actual first dock yields +0.148 to +2.537 mm lateral error at the dispenser. Centered first approach yields +1.209 mm. The issued feeding trial starts at zero. |

## What moves the result

The earlier planar continuation rejected transverse speeds above 0.010 mm/s. Nine of the ten shelf-edge states cross that threshold. The spatial model carries the signed linear velocity, the transverse cylinder axis, axial roll and yaw rate from each recorded approach. The pipe enters flight with lateral velocity from −0.1651 to +0.1829 mm/s. Gravity brings it to first floor contact after 106.8–107.0 ms. It rolls back to the lip about 0.749 s after leaving the shelf. The two faces provide separated axial support in every accepted run through assessment. Maximum modeled compression is 2.36 mm. Final contact does not imply a measured field result. [Source states](source.json) · [Contact receipts](physics.json)

The 593-solid fixed-low X48 camera configuration is used at the station and at full extension. A fresh native stationary-camera rendering seeds custody. The roof follows each corrected trajectory at 17 ms cadence. Fixed-head and roof-head images of the release pass through the existing packet consumer and five-millisecond rack controller in WPILib desktop HAL. A pressure-pad clock is recomputed from the supported source state with a three-millisecond allowance. All ten positive actuator paths have the same independently generated 1 ms motion samples. A late fixed frame on B causes a hold in every tested approach. The 603-body removable mount has its own qualification. [HAL events](events.csv) · [Motion and source hashes](hal.json) · [Aft camera registration](../bridge-tolerance/README.md)

The simulation uses the finite hollow tube's independently specified axial and transverse inertias, unilateral spring/damper normal forces, both tangential friction components and a changing world angular-momentum vector. Axial contact stations span the full tube, including its ends. They test the registered trimmed panels and mounting holes under a five-millimeter inset. The lip uses its finite rectangular cross-section and a conservative transverse material span. The solver evaluates first contact, compression, corner friction, two-sided support and complete-robot separation at each step. A nine-station 0.20 ms baseline was compared on the nominal starts to 17 stations at 0.10 ms and 33 at 0.05 ms. Both finer resolutions keep support and agree within 0.5 ms on first lip contact. Changing nominal friction from 0.35 to 0.10 or 0.55 also retains both deliveries. The maximum computed mechanical-energy increase over initial energy was under 0.001 J. The compliant law is a sensitivity model requiring measured stiffness, damping, friction and material compliance. [Finite trough survey](../mission/target.json) · [Inherited pipe properties](../axial-spin/pipe_axial_contact.py) · [Rigid body solver](SpatialContact.java)

Holding the rack extended or delaying its command 400 ms closes the fore-aft separating plane. The study withholds clearance in those runs. For the positive cases the plane bounds every robot solid after first field contact, including the moving shelf. A scored pipe's settled pose and axis are compared to the earlier native twelve-weldment distance test using the maximum distance any point of the inflated cylinder could move. Both scoring editions run on actual final support, after-release robot separation and branch exclusion. Independent field calibration, physical battery/traction limits, delivery variability and human traffic still condition this desk result. A bad reef observation before placement retains the earlier stowed five-point alternative where its stop and park are supported. Blind holds retain the three-point AUTO leave. [Scoring oracle](../scoring/README.md) · [Native branch check](../mission/branch-exclusion.json) · [Six-robot field checks](../integrated-match/README.md)

## Reproduce

Use the same local source inputs as the [approach study](../sector-alignment/README.md) and the [fixed-height observation](../low-observation/README.md). Supply the ten recorded wheel traces, historical optical cache, official lamp STEP, JDK 17 and populated WPILib Gradle cache. The project keeps raw camera, trajectory, motion and viewer captures in an ignored output directory. A run from the repository root is shown below.

```sh
python spatial-contact/source.py "$WHEEL_TRACES" "$OUTPUT"
python spatial-contact/observe.py "$OUTPUT" "$OPTICAL_CACHE" "$LAMP_STEP"
python spatial-contact/receive.py "$OUTPUT" "$OPTICAL_CACHE" "$LAMP_STEP"
python spatial-contact/control.py "$OUTPUT" "$OPTICAL_CACHE" "$JDK17" "$GRADLE_CACHE"
python spatial-contact/physics.py "$OUTPUT" "$OUTPUT/path-nominal-nominal-coral-A.csv" "$JDK17" --hal spatial-contact/events.csv
python spatial-contact/match.py
python spatial-contact/viewer.py "$OUTPUT/trace.csv"
python -m unittest discover -s tests -p test_spatial_contact.py
```
