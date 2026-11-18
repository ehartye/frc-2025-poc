# Offline controller for the station-fed receiver

This WPILib command operates one provisional station-to-L1 cycle against an explicit I/O interface. It owns the lift, slide, paired belt request and a drive-permission signal. The drivetrain owns physical motion. The desktop tester uses WPILib's CommandScheduler and simulated Driver Station with a scripted I/O adapter. No robot adapter, network discovery or deploy target is included.

Run from the project root with Python 3, an existing JDK 17 and the populated Gradle cache used by the project.

~~~sh
python scripts/run_competition_controls.py --java-home <JDK17> --gradle-home <populated-cache> --output <new-evidence-directory>
~~~

The runner checks the [mission candidate](../mission/candidate.json) against the [I/O and geometry contract](io-contract.json), executes this project’s tests using its pinned Gradle 8.11 and WPILib 2026.2.1 desktop toolchain, and writes receipt.json only after matching the fresh JUnit run token, exact test names and input hashes. Unsuccessful builds retain their diagnostics in the selected directory. The historical game, candidate design and desktop software baselines are independent.

## From an empty station arrival to withdrawal

| Transition | Required observations | Command or inhibit |
| --- | --- | --- |
| Start → receive | Newly scheduled attempt at a confirmed stationary station dock. Both occupancy ports clear, carriage near 940 mm and slide retracted. | Extend to 240 mm. Once in pose, request −0.20 m/s contact-surface velocity along the shelf on both belts. |
| Receive → confirm | Entry at +100 mm changes before Seat at +40 mm. Both ports remain occupied across at least 60 ms of fresh sensor timestamps. | Stop belt surface travel. The contacts can provide stationary grip only if the proposed mechanical preload and available control authority suffice. |
| Confirm → carry | Continue to observe both ports while retracting to 0 mm at 940 mm height. Confirm retracted switch and encoder before lowering to 670 mm. | Release drive permission only in the verified carry pose. Loss of either custody witness withdraws permission and latches a fault. |
| Carry → place | Drive reports target docked, stopped and clear for this attempt. The occupancy gate must distinguish the moving piece from a new obstruction. | Remove drive permission. Lower at 0 mm extension to 560 mm, then extend to 370 mm. |
| Ready → release | Current place pose, end switch, both occupied indicators and a new operator authorization edge. A request held from before readiness is insufficient. | Request +1.05 m/s on both belt surfaces. The ordinary request is a one-shot authorization. An interrupted command, invalid data or lost target permission cancels output on its next observation. |
| Release to shelf clearance | Seat clears before Entry. The independent calibrated shelf packet identifies the attempt and piece. | Stop feed, keep the slide at 370 mm and withhold drive permission. A missing witness times out. |
| Whole-robot clearance to desktop withdrawal | The test-only command matches a fresh registered exposure to the archived piece, sector, assembly, and continuously tracked acquisition. | Request signed slide velocity of -150 mm/s on the accepting tick. The position goal then ramps to 0 mm and tracking disagreement above 15 mm faults. The public command keeps holding and eventually times out. |
| Any active state → fault | Stale, invalid or regressing data, inconsistent limits, custody loss, indicated jam, unmet target permission or elapsed state deadline. Scheduler cancellation or simulated Driver Station disable interrupts the command. | Write neutral to both axes and belts, remove drive permission, preserve the first cause. After externally resolving the problem, an explicit recovery acknowledgement at empty, motionless station-height stow clears the latch. Starting another attempt requires a new schedule. |

All state deadlines, velocity requests, 2 mm axis tolerance and 60 ms input freshness are test hypotheses. They require measurements from the selected mechanism. A neutral output cannot hold up a carriage or stop an already moving pipe. Passive holding, available stopping distance, brake behavior and recovery access need independent mechanical evidence.

## Why the two occupancy channels cannot prove exit

With nominal seated center at shelf-local +80 mm and radius 57.15 mm, the tube's horizontal extent is +22.85 to +137.15 mm. The original −60 mm Seat position misses it even at maximum projection. A proposed transverse beam runs into the opposed belt cartridges. The revised candidate uses bottom-up reflective sensing through two shelf ports at +40 and +100 mm. Both lie inside the seated horizontal projection; the detection range, reflections and guard geometry still determine actual readings.

When the center moves outward to +160 mm, both projected points can be clear and the tube can still overlap the shelf ending at +165 mm. The [mission model](../mission/README.md) can supply a separate conservative 1 mm pipe/shelf clearance event. The nominal model reaches it about 16 ms after the last driven support. A subsequent loaded trough trial predicts recontact 0.614 s after first contact if the shelf stays put (assumed friction 0.35). Its 50 ms delay and 0.15 m/s retreat after independent clearance avoid this model counterexample. Its slower-release counterexample recontacts the shelf before reaching that clearance. The controller receives separate crossing-time bounds in the controller clock. The [stereo measurement design](../release-observer/README.md) tests independent observability and eligibility across sampling phases. The controller requires the event to belong to the current acquisition attempt. Driver software or uncalibrated elapsed time cannot invent it.

## Whole-robot withdrawal trial

The long grip belt could still touch the pipe when the shelf was clear. The revised short belt and finite coast were checked against four reopened STEP poses and 361 named bodies. The offline runner checks the exact assembly and clock record hashes, STEP bytes, part inventory, grip law, and registered route before the HAL test can receive a positive case. A revision that changes their identity requires a new evidence review. [Assembly and departure study](../mission/COMBINED-DEPARTURE.md)

The selected case is coral-A at sector -180 mm, shelf friction 0.10, image error 0.10 px and camera phase 0. Its stereo estimate clears the 537 mm robot front by at least 10.76 mm. The last loaded pad bounds prior contact separately. The source tests every integer motion onset from 72 through 104 ms after true shelf clearance for this exposure. The 600 ms control recontacts the receiver. A 1 ms clock allowance, 8 ms processing, up to 20 ms scheduling and an assumed 0 to 10 ms actuation lag are included. The trial adds zero intentional dwell. Both timestamps share the acquisition clock origin and are rounded to the 1 ms controller input grid. [Per-exposure clock and dynamics](../mission/short-vision-clock.json)

The 537 mm plane, acquisition ID, frame synchronization, calibration, actuator-start latency and unobstructed view all require physical verification for a real robot. The desktop adapter feeds one recorded measurement and scripted I/O into WPILib. Physical onset, scoring and deployment remain unverified.

## Electrical and commissioning evidence

The candidate uses six wheels with one geared drive channel per side, lift and slide axes, and two independently driven belt contacts. The drive and localization owner supplies a timestamped stopped/docked/clear decision. The receiver sends only drive permission. The installed physical drive controller must enforce that permission and its other inhibits. An independent clearance observer and a jam diagnostic remain design work. A single boolean cannot establish the identity of two touching pipes or award a score. Use the [edition-specific scoring check](../scoring/README.md) for scored-piece evidence.

The I/O contract leaves actual part numbers, protections, channel assignments, firmware, calibration and gains unset. Before an implementation can claim onboard readiness, reconcile motor/controller models, sensor voltage/range, protected circuit and conductor sizing, supply/grounding and wire routing. Identify each encoder zero, travel sign, switch polarity, physical stop, belt direction and encoder conversion against the selected assembly. Characterize loaded current, slip, grip loss, bilateral belt mismatch, sensor contamination and timestamp latency. Fit motion/current limits and gains only with evidence for the actual loads and hardware. A clean desktop run checks sequencing and cleanup, and leaves the operational and inspection sign-offs pending.
