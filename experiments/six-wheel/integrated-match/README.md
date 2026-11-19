# Two pipes and a park

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

[Open the interactive field replay](replay.html). Choose a strategy or frame fault, then use the evidence clock to jump to a station pickup, a rack decision or the final assessment. Times below run from the start of AUTO unless marked TELEOP.

| Six-robot study | AUTO LEAVE | Supported TELEOP L1 | TELEOP park | Modeled points |
|---|---:|---:|---:|---:|
| Two pipes, 15 ms image delivery | 3 | 2 × 2 | 2 at TELEOP 92.899 s | **9** |
| Second pipe, one late frame | 3 | 1 × 2 | 0, halted at the reef | **5** |
| Commit to park after the first pipe | 3 | 1 × 2 | 2 at TELEOP 42.489 s | **7** |

These are the focal robot's contributions under the specified traffic and contact trials. The seven-point route requires choosing park before the return trip for pipe B. The operator cannot recover that route retroactively after the camera fault.

## How each pipe earned credit

The red fixed-low robot holds its carriage at H560 mm, receives with its slide stowed and places at 370 mm extension. The source is the 593-body, two-camera, air-removed build with the X98/Y0/Z842 mm moving head. The state of each pipe comes from its own station feed, axial spin and 0.65 m/s port-control trajectory. A single millisecond FPGA clock runs each match through the final assessment at 156 s. Each local trajectory has one fixed time offset at the physical station lip. Its subsequent events retain that offset. The drive permit arms on the last integer millisecond before the prescribed continuous route start. The feed is reported to the nearest millisecond.

| Controller / physical observation | Pipe A | Pipe B |
|---|---:|---:|
| Physically issued from finite station stock | 21.100 s | 71.501 s |
| Port plus five stationary stereo exposures confirm the seat | 23.140 | 73.540 |
| Moving observer holds identity, loaded drive permit arms | 27.032 | 77.434 |
| Roller release issued | 48.010 | 98.100 |
| First trough contact in the physical replay | 48.2833 | 98.3732 |
| Whole-front observation commands withdrawal | 48.295 | 98.390 |
| Independent motor and linear-sensor displacement confirmation | 48.330 | 98.425 |
| Limit, displacement and speed confirm stow | 50.765 | 100.860 |

Each rack drive uses the registered 5 ms control loop, 10 ms motor output delay, 25 N drag, quantized position feedback and the additional moving camera mass. The Java HAL emits actual simulated rack position, current and voltage every millisecond. The first pipe must clear and the slide must physically confirm stow before the second feeder can run. Stow precedes the issued departure by just **10.84 ms** and **5.70 ms**. The fields, motor simulator and date-specific scoring evaluator are coupled by those emitted events and paths. Feedback does not alter the scripted trajectories of the other five robots. [Controller events](events.csv) · [Desktop HAL](IntegratedTest.java) · [Rerun receipt](hal.json)

The contact replay samples at 0.2 ms, applying the actual rack path to the original finite pipe and its incoming spin. Both pipes settle against the trough floor and lip, remain clear of the robot by at least 32.97 mm, and stay supported through the three-second post-buzzer assessment. Their final uninterrupted support durations are 106.97 and 56.88 s. The minimum distance from the modeled other-level branch is 160.24 and 161.39 mm. Both rule editions accept these two L1 receipts at assessment. The scoring evaluator leaves inventory-control questions without evidence marked unknown. [Physical replay](contact.json) · [Field and scoring receipts](match.json) · [Edition-specific predicates](../scoring/README.md)

In the negative case, the sixth fixed-camera release packet on B takes 26 ms. The camera watchdog fires at 98.210 s, after roller release but before the rack commands motion. The shelf stays extended and recontacts B at 98.940 s. B lacks support evidence for the assessment. The robot remains at the reef. Pipe A clears the complete second extended robot by at least 37.19 mm. [Camera delivery cases](../moving-frame-timing/README.md) · [Rack fault trials](../slide-feedback/README.md)

## Field, optics and physical limits

All six registered starting footprints intersect their own tape. The focal robot has 23,574 mm³ of exact front bumper/tape intersection. It completes AUTO travel in 13.492 s with 5.847 m of clearance behind its full hull. The moving roster and 24 placed field-stock objects give a 105.47 mm lower pair separation on the two-trip schedule, and 125.89 mm for early park. The smallest traveling stock bound is 7.85 mm and nominal station standoff is 10 mm. The inboard park has 200,577 mm³ of front bumper in the zone. Its upright-cage margin is 145.33 mm before, or 110.49 mm after, the 34.84 mm comparison tracking allowance. Actual cage displacement, carpet traction, station approach and changing opponent positions still require field evidence. [Registered field model](../fixed-height-match/README.md) · [Installed stock](../full-match/installed-stock.json)

The geometry register verifies 590 identical native solids and three replacement or added solids at both receiving and placing. The fixed stereo renderer recomputes 201 station frames and 550 release frames with the actual scoped signal-light shape. All 550 release fits match the prior numeric intervals and validity outcomes. The unchanged moving camera, shelf and finite pipe share a conservative sight volume. Each changed or removed base-mounted solid clears it by at least 84.30 mm. That preserves the modeled moving-view silhouettes along these sampled paths with the original lighting, contrast, camera calibration and background. New occlusion from other robots, people, glare or wiring is outside this evidence. The alternate X48 mount requires its own images and calibration. [Geometry and optical receipt](optics.json) · [Original two-camera evidence](../moving-observer/README.md) · [593-body installation](../design/fixed-no-air/README.md)

Power qualification uses the [two-camera supply study](../fixed-low-power/dual-camera.md). Under the screened battery model the right drive reaches 9.577 V, just above the 9.5 V design floor. A 32 mΩ common path gives 9.145 V, below that floor. The recorded rack path reaches 32.36 W controller input. Bench measurements of loaded voltage, actuator drag, backlash, camera transport and exposure timing, true station feed and drive tracking remain necessary for physical use. The desktop study runs software and offline geometry only.

## Reproduce

Prepare the low receiver trajectories and study inputs following the [low-observation replay](../low-observation/README.md). Supply the original manufacturer STEP with the SHA in optics.json, the project geometry and scientific Python dependencies, JDK 17 and the primed WPILib desktop Gradle cache. Keep downloaded CAD, images, raw HAL paths and generated caches in ignored artifacts. Run from the project root.

```sh
python integrated-match/optical.py "$LAMP_STEP" "$OBSERVATION_CACHE" artifacts/integrated-match
python integrated-match/verify.py "$OBSERVATION_CACHE" artifacts/integrated-match "$JAVA_HOME" "$GRADLE_USER_HOME"
python integrated-match/contact.py "$JAVA_HOME" artifacts/integrated-match
python integrated-match/field.py
python integrated-match/viewer.py
python -m unittest discover -s tests -p test_integrated_match.py
```

The optical script uses separate native processes per stroke so rendering cannot change a later B-rep fingerprint. The HAL run writes the event log, registered compile and input digests, and per-pipe raw position, phase-current and motor-voltage traces. The contact pass rejects an unregistered trace. The small committed receipts and self-contained HTML can be inspected offline.
