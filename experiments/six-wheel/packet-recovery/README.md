# Can the robot park after the second camera packet arrives late?

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

The 597-body replay demonstrates the controller sequence. At its two extended scoring poses, each inner slide rail intersects the official reef wall by about 1,387 mm³. The revised shop channels and position sensor clear the wall through the modeled stroke. Station docking, powered motion and the physical build still determine whether the robot can score. [Field contact and revised receiver](../selected-match/README.md)

**Yes, for the registered second-feed trial when the roof camera still sees the same pipe.** The fixed camera's sixth exposure took 26 ms to arrive. The robot held its rack while the roof camera maintained the track. It withdrew only after a later fixed exposure cleared the entire robot front, confirmed stow at 100.845 seconds and parked at 110.878 seconds for nine modeled points. Moving the same fault one exposure later leaves the roof without a usable view. That robot holds at the reef. [Watch the shared six-robot field path](../relieved-match/replay.html) · [Compare controller events](events.csv)

| Match time | Sensor or controller observation | Effect |
|---|---|---|
| 98.210 s | The fixed stream exceeded its 40 ms freshness bound. Its last good capture was at local 26,214 ms. The roof exposure at 26,231 ms had arrived after 15 ms. | Inhibit withdrawal. Continue the independently observed track. |
| 98.215 s | The delayed fixed frame, captured at 26,231 ms, is now available. Its XYZ intervals intersect the roof measurement at the same capture. | Re-establish the fixed track. Keep the rack extended. |
| 98.375 s | A later fixed capture at 26,401 ms shows the entire finite pipe beyond the registered 597-body front. The independently qualified pressure prior and remaining motion window are still current. | Permit withdrawal. |
| 98.410 s | Linear and motor encoder travel agree. | Confirm motion. |
| 100.845 s | Linear and rotary feedback, endpoint switch and low speed agree. | Permit the chassis to depart for the parking sector. |

The roof stream must have delivered its own frame **before the previous fixed observation expires**. The same pipe and acquisition epoch must have three consecutive, directly overlapping earlier roof/fixed exposures, one consistent new roof sample, and a matching fixed reacquisition. Both streams retain their actual capture, delivery, sequence and camera registration. A second loss ends the recovery. The independently tracked pipe must then progress to a fresh whole-front image before the original pressure deadline. [Supervisor](GapWitness.java) · [Registered image and pressure inputs](../relieved-match/controller-register.json)

### What the fault trials distinguish

| Input on B | Observed desktop result |
|---|---|
| Sixth fixed frame takes 23–32 ms, tested at each integer value, with the remaining frames at 15 ms | All ten runs confirm both pieces and the same final rack motion. At 32 ms the next frame shares an arrival time and the input retains capture order. |
| Sixth fixed frame takes 33 or 34 ms | The next fixed capture arrives first. Sequence checking inhibits recovery. |
| Same 26 ms delay on the seventh fixed frame | The roof reports unresolved pipe geometry at that capture. The system withholds withdrawal. |
| Both views late, a second gap, missing roof or missing next fixed exposure | The independent coverage or continuity check fails. |
| Contradictory roof bounds, duplicate sequence, wrong pipe or changed epoch | The system holds. |
| Rack jam or expired pressure observation supplied to the rack | The actuator feedback gate withholds a stow confirmation. The chassis stays at the reef. |

The original nine-point nominal, seven-point single-trip and three-credit failed-second-trip controls reproduce their original HAL events. Both physical pickups were rerun with two integration intervals and the 3 mm displaced-feed control. The replay compared both revised feed records to the original source bytes. Two independently executed 26 and 30 ms recovery runs reproduce the successful pressure/custody, motion and stow clocks and the same 1 ms rack trajectories. Four finite-cylinder contact integrations per run continue through the scoring assessment. The field replay includes the preceding A pipe, staged stock, all six scheduled robots and the ending sector. [Controller results](results.json) · [Field and contact replay](geometry.json) · [Acquisition and original controls](../relieved-match/README.md)

The result uses the modeled two-camera calibration, backing, matched capture phase, one-newton pressure prior and clear view of one foreground pipe in the handoff volume. The previously scored first pipe is included in B's fixed-camera background. An indistinguishable unmodeled foreground article, different lighting, capture jitter, or an unmeasured pressure response needs fresh qualification. The two neutral camera streams use a common synthetic clock. Bench measurements must establish their actual synchronization, delivery order and actuation bounds.

### Repeat the desktop study

Use the 597-body raw images and force traces produced by the [relieved-receiver experiment](../relieved-match/README.md), a populated project Gradle cache and its Java 17 toolchain. The geometry replay uses the same Python packages as the original investigation.

```sh
python packet-recovery/replay.py "$RELIEVED_CACHE" "$OUTPUT" "$JDK17" "$GRADLE_CACHE"
python packet-recovery/geometry.py "$RELIEVED_CACHE" "$OUTPUT" "$JDK17"
python -m unittest discover -s tests -p test_packet_recovery.py
```
