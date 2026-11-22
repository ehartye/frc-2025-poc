# Offline controller for the station-fed receiver

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 83b3b9c7fe](../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

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

The active trial reconstructs four exact red STEP exports from the registered blue files and the approved red-kit archive. The runner checks the 401-body inventory, displaced grip geometry, route revision, corrected floor-piece starting configuration, stereo timing and independent native clearance envelope against immutable receipts. Each accepted packet carries its piece, target sector, friction case, image uncertainty, exposure phase and acquisition time origin. [Red field trial](../full-match/field-trial.json) and [whole-assembly receipt](../full-match/red-assembly.json)

The desktop cases cover coral-A at -180 mm and coral-B at +220 mm, with shelf friction 0.10/0.20 and image uncertainty 0.10/0.25 px. Of 136 simulated zero-dwell exposures, 134 meet the prior-contact deadline and complete-robot separation criterion. At 0.20 friction and 0.25 px uncertainty, phase 16 for A and phase 9 for B miss the deadline. All 136 additional records with a deliberate 50 ms wait remain inhibited. The HAL test reaches withdrawal on each eligible case at both ends of its 7-29 ms observation-age bracket. The command issues -150 mm/s on the accepting tick. A 600 ms late-motion control recontacts the receiver. [Clock and dynamics](../full-match/red-withdrawal-clock.json) and [extended native geometry](../full-match/observer-extent.json)

The 537 mm front plane, unique piece identity, friction class, error and response assumptions are pinned simulation inputs. Their physical calibration and actual actuator-start time remain unverified. The ordinary command holds. The predecessor 361-body trial and its single exposure remain reproducible at its original revision. [Historic 361-body fixture](https://github.com/ehartye/frc-2025-poc/tree/05c20f4f9dd8523305ac080b6809403ed8e4f5f8/competition-controls)

## Electrical and commissioning evidence

The candidate uses six wheels with one geared drive channel per side, lift and slide axes, and two independently driven belt contacts. The drive and localization owner supplies a timestamped stopped/docked/clear decision. The receiver sends only drive permission. The installed physical drive controller must enforce that permission and its other inhibits. An independent clearance observer and a jam diagnostic remain design work. A single boolean cannot establish the identity of two touching pipes or award a score. Use the [edition-specific scoring check](../scoring/README.md) for scored-piece evidence.

The I/O contract leaves actual part numbers, protections, channel assignments, firmware, calibration and gains unset. Before an implementation can claim onboard readiness, reconcile motor/controller models, sensor voltage/range, protected circuit and conductor sizing, supply/grounding and wire routing. Identify each encoder zero, travel sign, switch polarity, physical stop, belt direction and encoder conversion against the selected assembly. Characterize loaded current, slip, grip loss, bilateral belt mismatch, sensor contamination and timestamp latency. Fit motion/current limits and gains only with evidence for the actual loads and hardware. A clean desktop run checks sequencing and cleanup, and leaves the operational and inspection sign-offs pending.
