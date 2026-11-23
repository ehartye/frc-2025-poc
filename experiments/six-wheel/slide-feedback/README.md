# Can the slide clear both pipes before the robot departs?

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

The desktop controller corroborated movement and stow in **all 17 camera phases for both pipes** on the 598-body fixed-low robot. Each phase ran with five scheduler offsets, eleven actuator delays and three string-pot biases. In those 5,610 trials, the nearest point on the pipe stayed at least 31.08 mm beyond the robot front. Both pipes retained modeled floor and lip support through three seconds of assessment. [Phase results](phases.csv) · [Machine-readable evidence](results.json) · [Camera handoff](../moving-observer/README.md)

| Result | A, −180 mm sector | B, +220 mm sector |
|---|---:|---:|
| Latest measured movement after earliest pad edge | 177 ms | 176 ms |
| Minimum shelf clearance | 37.37 mm | 37.37 mm |
| Minimum whole-robot forward clearance | 31.52 mm | 31.08 mm |
| Smallest margin before scheduled stow | 4.17 ms | 4.67 ms |
| First recontact with a shelf held extended, local time | 27.2784 s | 26.9859 s |

## From image to measured motion

The earlier 20 ms optical gate admitted an immediate command in 11/17 phases for A and 12/17 for B. The actuator study requires two measurements to confirm actual movement. None of the three conservative actuator policies reserves enough time under the original 150 ms pad-to-motion cutoff. A separate controller services the registered withdrawal every **5 ms** and requires movement within 40 ms of command and 180 ms of the earliest pad indication. It binds the accepted camera, clearance geometry, registered sector, attempt, exposure range and 15 ms delivery. The 180 ms cap applies only to this reviewed set of two transfers. A separate [frame-delivery study](../moving-frame-timing/README.md) changes transit and drops frames. Those inputs have no actuator permission in this controller. [Earlier clock](../low-observation/README.md) · [Controller](Retreat.java)

The reference moves the outer rack 370 mm in 2.4667 s using 70 ms acceleration and 300 ms braking. The NEO 550 drives an 18.93:1 reduction, a 10 mm pitch radius and a doubled outer stroke. The plant carries 5.3571 kg outer, 0.440 kg additional moving-camera allowance and 1.3928 kg middle, with reflected rotor inertia, 70% gear efficiency, a 20 A phase cap, 25 N nominal slide drag, another 2 N mechanism allowance and 9.5 V available. Voltage commands are held and delayed by 0–10 ms. The rack advances in 1 ms steps. Position, integral error and motor-derived velocity close the loop. Peak phase current was 9.56 A and peak motor voltage 2.72 V. An endpoint switch, both position readings and low measured speed confirmed stow 2,470 ms after command. The modeled stop saw up to 20.25 mm/s. Its physical load rating, damping and backlash remain open. [Rack and electrical assumptions](../fixed-low-power/README.md)

One raw 12-bit ADC count on the 635 mm SP1-25 stroke is about 0.155 mm. Motion requires a four-count change from the extended reading, independent motor travel of at least 0.20 mm and positive measured velocity. At the reviewed gearing one 42-count motor Hall increment represents 0.158 mm. The direct-position cases apply differential biases of −0.254, 0 and +0.254 mm, spanning twice the manufacturer's 0.02% repeatability. Its ±0.25% absolute accuracy gives ±1.59 mm over full stroke. Local endpoint calibration and a measured differential error are required. Fresh 5 ms sensor frames, stable excitation, gear lash, controller latency, noise and switch calibration remain assumptions. [TE SP1-25](https://www.te.com/en/product-SP1-25.html) · [WPILib 2025 analog inputs](https://docs.wpilib.org/en/2025/docs/software/hardware-apis/sensors/analog-inputs-software.html) · [REV NEO 550](https://docs.revrobotics.com/brushless/neo/550)

## Faults and contact

A missing track, incorrect pipe or geometry, late packet, stale or inconsistent feedback, or low supply prevents completion. A jam can block first movement or cause a tracking fault during travel. An interruption, disable or contradictory observation drops requested effort while the simulated mechanism continues. The 15 N and 35 N drag alternatives completed at one exposure phase, 4 ms actuation delay and all five scheduler alignments for each pipe. The 80 N and 140 N alternatives failed. The complete HAL run contains 6,912 trials. All 14 fault categories produced modeled shelf recontact at every tested scheduler offset. A rack that seizes during motion recontacts A at or after 27.3212 s local and B at or after 27.0287 s; the controller withholds departure. [Fault results](results.json)

The verifier uses the attained Java rack positions and the issued translational and axial pipe states. Every 0.2 ms it compares modeled contact with the rounded shelf and the continuously changing whole-robot front. The roof-mount parts stay behind the original leading shelf. Each pipe's 2,805 accepted cases reduce to 693 distinct traces and command times. Fixed lighting and background, stationary field geometry, calibrated incoming pipe and frame delivery, custody and the other robots’ behavior remain conditions of the earlier studies. The new whole-match power budget, physical observer performance and actual scoring remain open. [Contact verifier](contact.py) · [Original field clock](../fixed-height-match/README.md)

## Reproduce on desktop

Use the project's pinned WPILib 2026.2.1 desktop baseline for this historical game, JDK 17 and cached Gradle dependencies. Raw 1 ms motion traces and the 6,912 trial rows are written to the chosen output directory.

```sh
python slide-feedback/verify.py --java-home "$JDK" --gradle-home "$GRADLE_HOME_CACHE"
python slide-feedback/contact.py --java-home "$JDK"
python slide-feedback/compare.py
python -m unittest discover -s tests -p test_slide_feedback.py
```

The no-air installation separately retains the forward envelope and moving rack mass used here. At 0, 185 and 370 mm, 590 component B-reps match in the 598- and 593-body installations. The shared bumper or shelf governs all three fronts (425, 425 and 535 mm). All removals/changes are fixed-base and the revised deck and lamp reach only 337 mm. Run the standalone comparison with the issued native register. The control signature and optical approval remain on the 598-body fixture. [No-air transfer](no-air.json) · [Register check](compare.py)
