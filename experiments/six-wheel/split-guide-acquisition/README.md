# When should the shelf try to seat a pipe again?

<!-- cad-still -->
[![Team 1780 · stowed swerve robot](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · stowed swerve robot.** Red alliance R4 with numbered bumper covers. Purchased parts use project-created representations in this public assembly. [Source STEP · abbd8ab838](../../../cad/full-swerve-r4/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

On the left station approach, the pipe touches Entry, moves over Seat, bounces clear of both ports and settles back over Seat. A 20 ms difference in that last transition can leave the original controller waiting until it faults. This desktop experiment uses the original two optical apertures and makes a bounded forward probe after observing that sequence. [Compare the timing](timing.html) · [Predecessor support and station model](../compliant-carriage/README.md)

## What changed on the actual station clock?

Both full-duration simulations recovered the left pipe at 1,340 ms, and the original right approach reached its existing 1,360 ms acquisition at both integration steps. Both pipes stayed in the dual-return state through 5,460 ms. The left first diverged from the issued physical trajectory on the first integration step after the 580 ms probe command. Every recorded right-hand pipe sample remained byte-identical through the earlier 1,700 ms study boundary. [Signed controller and mechanical states](candidate.json) · [Direct trajectory comparison](comparison.json)

| Plant with the bounded program | 31.25 µs | 15.625 µs | Evidence at the decision |
|---|---|---|---|
| Left station arrival | IN_BAND 1,340 ms | IN_BAND 1,340 ms | Entry and Seat continuously true for 240 ms |
| Right station arrival | IN_BAND 1,360 ms | IN_BAND 1,360 ms | Existing sensing and approach reproduce |
| Left, both belt outputs locked | FAULT 1,340 ms | FAULT 1,340 ms | Seat only after the forward demand |
| Left, end-grip traction lost | FAULT 700 ms | FAULT 880 ms | Seat goes clear during the probe |

The good left pipe and the locked-belt pipe give identical two-bit observations from the 580 ms challenge until Java 1,100 ms, when the actual moving pipe first lights Entry again. That leaves a full **520 ms** when these sensors cannot distinguish useful belt motion from a stalled drive. The actuator's current and speed need separate limits and bench verification throughout that interval. The probe supplies a bounded test; its starting condition never grants chassis motion.

## What the robot can observe

The desktop program receives Entry and Seat, two digital returns from 12 mm shelf bores at X100 and X40. Their active ranges are 4 and 17.5 mm above the moving local tangent planes. The 21-mode mechanical model follows the actual cylinder, shelf curvature, four bracket seats, two serial 1 mm guide stages and both stowed dampers. Only the optical bits, timestamps, calibration, enabled state and dock status reach the controller. Before contact, the original guide is 850 mm high and uses its released face law. The shortened backed guide needs a separate joined replay. [Apertures and contact](../shelf-structure/coupled.py) · [Native bracket model and powered docking poses](../compliant-carriage/series.json)

At Java 320 ms, both signed arrivals first lit Entry. Seat lit at 500 ms. The left arrival cleared both ports at 520 ms, then Seat returned alone at 560 or 580 ms. The right arrival was still clear at 580 ms. At that tick the original brake inhibit expired. The unmodified left pipe settled at X−5.61 to −5.75 mm. The right pipe was only 2.6 mm farther back at the same tick and the existing forward probe brought it into the seated band. The position and speed explain the difference to a student. Neither value enters the new controller.

After the original braking interval, the additional branch requires an observed incoming Entry, a Seat-only return, two consecutive 20 ms dark samples after Seat, and a fresh Seat-only return. It can begin only 260–360 ms after first Entry. The probe commands +0.16 m/s. Seat must then be observed on its own during the probe, followed by a new dual return. Dropping either port through the 240 ms dwell inhibits seating. The additional branch times out at 760 ms, limiting commanded belt travel to 121.6 mm even if pipe progress is unknown. The earlier brake, reverse pulse, regular probe, fault-neutral behavior and continuing custody checks remain active. [Controller change](split_controller.py) · [Issued program and desktop adapter](../observable-acquisition/PortConsole.java)

## How the faults differ

On the locked-belt plant, the controller can see the same clear-and-return sequence as a recoverable pipe. The belt jams at the first positive challenge while the pipe and support model continue integrating. Both left-hand runs stayed Seat-only and timed out at 1,340 ms. When end-face friction instead vanished at the challenge, the actual belts reached +0.16 m/s but the pipe moved away from Seat. The controller faulted at 700 ms at the coarse step and 880 ms at the fine step. Physical belt velocity takes a further 6.11–6.13 ms to reach zero. The pipe traveled another 1.09–3.01 mm in the following 200 ms. The locked case moved under 0.001 mm over the same window. The neutral command neither arrests the pipe instantly nor proves indefinite retention. [Locked-belt result](locked.json) · [Loss of traction](traction.json) · [Physical follow-through](postfault.json)

Sixteen chronological fault fixtures cover absent Entry, a frozen Seat, a one-sample dark flicker, an occupied start, missing or unsynchronized samples, loss of dock or enable, ambiguous probe edges, and loss in dwell. They all finish with the command inhibited. A fabricated copy of a passing right-hand port sequence still produces IN_BAND on an empty receiver. Two return bits cannot distinguish a copied sequence from a real pipe. Physical custody and a drive release need an independent current observation of the pipe. [Compiled Java fixtures](signals.json)

## Reproduce the stationary experiment

Use the project desktop Python environment and JDK 17. The station solver regenerates its plant from the original project files. The released two-stage raw histories supplied to the comparison are generated with the predecessor's `replay.py series` entry point. Every candidate trajectory and controlled-fault trajectory gets a separate directory. No physical driving or robot connection is involved.

```text
python split-guide-acquisition/run.py candidate <jdk17> <raw> split-guide-acquisition/candidate.json
python split-guide-acquisition/run.py locked-belts <jdk17> <raw> split-guide-acquisition/locked.json --deadline 1700
python split-guide-acquisition/run.py lost-traction <jdk17> <raw> split-guide-acquisition/traction.json --deadline 1700
python split-guide-acquisition/observability.py <jdk17> split-guide-acquisition/signals.json
python split-guide-acquisition/check_program.py <jdk17> split-guide-acquisition/java-replay.json
python split-guide-acquisition/compare.py <issued-raw> <raw>
python split-guide-acquisition/guide_positions.py
python split-guide-acquisition/restart.py <raw> split-guide-acquisition/restart.json
python split-guide-acquisition/postfault.py split-guide-acquisition/postfault.json
```

The restored 20 ms controller memory comes from replaying the exact public input prefix. The mechanical handoff binds the modal basis, positions and rates of all four support lands and each guide stage, both plunger states, pipe orientation, velocity and angular momentum, end-grip forces and the held belt command. Exposure snapshots use the station epoch at 21.571672635314284 wall seconds and the lip at Java 100 ms. The first wheel service is 0.500 ms after the 5,460 ms station checkpoint. [Registered station clock](../relieved-match/first-feed.json)

No joint preload, physical sensor range, belt traction, motor stall current, pipe motion, camera confidence or loaded field continuation was measured here. A retained hold requires a separately qualified independent observer. The actual dock and fixture must establish a safe probe force, stall timeout, travel and retained contact before anyone can rely on a physical recovery.


At the final checkpoint the modeled pipe centers span X75.958–76.954 mm. The two end belts carry 11.48–11.50 N nominal normal force each under the inherited preload and compliance model. Each of the seven exposure packets supplies the prior held Java command. The released six channel bounding boxes move vertically by −0.00405 to +0.00115 mm over the four parked histories. The bounds map the current modal field onto each individual native channel and omit lateral lash and manufacturing tolerance. These are draw-ready source states; camera visibility and later custody still require a rendered or physical observation of this exact configuration. [Six native channel bounds](guide-bounds.json) · [Restored motion and 0.500 ms handoff](restart.json) · [Java memory replay](java-replay.json)
