# How late can the two cameras deliver a release observation?

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 83b3b9c7fe](../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

On the fixed-low robot, the controller must recognize the same pipe as it moves from the shelf-mounted stereo head into the fixed head's view. A valid image can still arrive too late to begin withdrawing the shelf. At a 5 ms service period, both pipes met the modeled command deadline at all 85 camera/service alignments per pipe for every constant integer delivery latency from 7 through 18 ms. Pipe A missed one alignment at 19 ms. At 24 ms all alignments failed. A physical installation would need a measured latency distribution, including tail events, and timestamps on both sides of the transport. [Registered 598-body geometry and travel](../moving-observer/README.md) · [Independent actuator-feedback study](https://github.com/ehartye/frc-2025-poc/pull/65)

| Equal, constant delivery time | 20 ms service, A | 20 ms service, B | 5 ms service, A | 5 ms service, B |
|---|---:|---:|---:|---:|
| 15 ms | 185/340 | 228/340 | 85/85 | 85/85 |
| 18 ms | 130/340 | 171/340 | 85/85 | 85/85 |
| 19 ms | 109/340 | 149/340 | 84/85 | 85/85 |
| 22 ms | 3/340 | 10/340 | 68/85 | 75/85 |
| 23 ms | 0/340 | 0/340 | 58/85 | 69/85 |
| 24 ms | 0/340 | 0/340 | 0/85 | 0/85 |

Each column crosses all 17 integer-millisecond fixed exposure phases with all integer-millisecond offsets of its controller. The moving head takes one exposure every 17 ms. With 15 ms delivery and the original zero application offset the desktop 20 ms scheduler reproduced the earlier 11/17 and 12/17 phase ticks exactly. The separate 5 ms study reserves 40 ms for actuator feedback within 180 ms of the earliest admissible pad contact. The original 20 ms case reserves 10 ms within 150 ms. Both leave 140 ms until the request. These times bound command eligibility. The observer does not establish that the slide moved or that the field supported the pipe. [Phase ticks, capture times, raw input hashes and delay sweep counts](results.json)

## What the receiver checks

The input comprises two serially ordered streams, an independently supplied contact prior, the release pose and the calibration epoch. Each packet carries head identity, piece and epoch, camera register, exposure time, receiver arrival time and the finite fitted position intervals. Capture and arrival share the modeled monotonic clock. The [numerical inputs](exposures.csv) come from the registered rendered stereo images for the two original trajectories. Three consecutive matching moving/fixed position boxes establish the handoff. The fixed head keeps following the pipe and must put the whole uncertainty interval past the front edge with 1 mm separation. A qualifying capture must be 7–29 ms old at consumption, and the sequence must retain a 40 ms observation bound. A successful request is single-use. [Packet consumer](OpticalWindow.java) · [Pinned registrations and contact clocks](contexts.csv)

A polling-only watchdog can miss a loss of observation. For example, take exposure times 0, 17 and 34 ms and arrival times 26, 43 and 60 ms. A poll at 40 ms sees an age of 40 ms. At 60 ms the newest frame is 26 ms old. But immediately before the 43 ms arrival the prior observation was already 43 ms old. The consumer compares the **next arrival with the preceding capture** before accepting it. One late fixed packet, a 26 ms moving-head delay paired with 9 ms fixed delay, and a duplicate clearance packet delivered in the qualifying batch all inhibit withdrawal for every tested phase on both clocks. A prescribed varying-delay trial gave 7/17 per pipe at 20 ms and 17/17 per pipe at 5 ms at the original application offset. This represents one specific pattern. [Clock and fault fixtures](PacketClockTest.java)

## Reproduce the study

Use the registered scene and trajectories from the [moving-camera reproduction procedure](../moving-observer/README.md), the repository's offline desktop Gradle cache, Python CAD and image dependencies, and JDK 17. From the repository root, run the following commands with the cache paths supplied by your environment.

```shell
python moving-frame-timing/exposures.py --scratch artifacts/fixed-height-observation
python moving-frame-timing/verify.py --java-home "$JAVA_HOME" --gradle-home "$GRADLE_USER_HOME" --output artifacts/moving-frame-timing/sweep.csv
python -m unittest discover -s tests -p test_moving_frame_timing.py
```

The verifier runs 1,428 trials on the desktop WPILib command scheduler and 59,364 on the same packet consumer with explicit service times. The full sweep records one row per case including the process used. The interval from 7 through 41 ms was exercised in 1 ms increments with both heads delayed together and with only the fixed head varied. The 17 camera phase positions and finite simulated trajectories bound the result.

On a bench, log hardware exposure time, first receiver arrival before queuing, service time, pose, serial, calibration revision and reset epoch. Preserve dropped sequences and clock synchronization uncertainty. Exercise both USB streams and host under the expected load, including low light, motion blur and loss of either view. Measure pad association and the interval from a request to independently sensed motion. Actual clock drift, blur, calibration, mount stiffness, physical support and actuator response remain open on this configuration.
