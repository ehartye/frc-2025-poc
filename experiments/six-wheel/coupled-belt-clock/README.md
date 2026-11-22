# What happens when the real controller clock drives the pipe model?

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 14649626c7](../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

The WPILib desktop controller commands **1.05 m/s** as soon as release begins. Both registered pipes then reach the inner riser in the 545-body contact model. An isolated **0.90 m/s** profile reaches the trough at a lower speed. Both pipes settle against its floor and lip **only when the model also withdraws the shelf on the independent clearance clock**. Leaving the shelf extended, as the ordinary controller does, brings it back into contact with both pipes. [20 ms controller trace](controller.csv) · [Numerical results](study.json)

| Command sent to both belts | Speed at first trough contact | Spin at first contact | With the #43 clearance clock | With the ordinary controller’s fixed slide |
|---|---:|---:|---|---|
| Historical step to 1.05 m/s | 1.036 m/s | −0.279 rad/s | Inner-riser contact | Inner-riser contact |
| 0, 0.45, 0.90 m/s over three scheduler ticks | 0.884 m/s | −0.316 rad/s | Sustained floor and lip support at 32.076 and 82.165 s TELEOP | Receiver recontact about 0.566 s after first trough contact |

The rounded values use 0.5 ms contact steps. A second run at 0.25 ms produced the same four outcomes on both routes. At 0.5 ms the finite pipe cleared the powered shelf by 1 mm at **157 ms** after historical release and **190 ms** after shaped release. The 545-body post-contact model waits from this geometric event for its established observer calculation. The separate 401-body Java test supplied scripted port transitions, supplied a shelf-only exit at 400 ms, then held the slide at 370 mm. It issued no withdrawal. Relate the two clocks to the first release command. The scripted exit supplies no measured clearance time. [401 controller](../competition-controls/README.md) · [545 contact study](../axial-spin/README.md)

## Can the controller produce the starting seat?

The acquisition check fixes the pipe’s chute exit at the first WAIT_ENTRY tick. The trace commands −0.20 m/s for 100 ms, then zero during confirmation. With that input, the centered pipe comes to rest at **−32.0 mm**, outside the accepted **+72 to +88 mm** window. At 120 ms the fixture still reports that the +100 mm entry port is blocked. The entire pipe projection already clears that port by **17.0 mm**. A real controller receiving a clear entry port during CONFIRM would fault. Optical response, exact release timing and contact loads still need measurements. [Port contract](../competition-controls/io-contract.json)

The release comparison starts from #43’s feedback-controlled acquisition and carry. That model seats the pipe at +78.5 mm. Its feedback depends on simulated position and speed that the two physical ports cannot directly measure. The fixed-slide and independent-withdrawal outcomes therefore describe a prepared placement. The evidence supports no complete controller-driven match or official score.

## Reproduce

Use the project’s existing JDK 17 and populated Gradle cache for its WPILib 2026.2.1 desktop harness. The additional Java code is compiled only as a desktop test source set. HAL advances its FPGA clock exactly 20,000 µs between scheduler executions. Both profiles use the same ordinary StationCycle command, receiver, fake sensors and port order. A third run clears the entry port at the 120 ms geometric counterexample and verifies a latched seat fault with neutral output. The ramp changes only a positive belt demand. [Desktop adapter](DesktopReleaseIO.java) · [Scheduler fixture](DesktopClockTest.java)

```powershell
python coupled-belt-clock/capture.py --java-home <JDK17> --gradle-home <populated-cache>
python coupled-belt-clock/replay.py
python -m unittest discover -s tests -p test_coupled_belt_clock.py
```

Replay holds each emitted speed until the next scheduler tick. The direct setpoint is passed through the existing belt acceleration and annular friction laws. Free-running belts approach the setpoint while unloaded and apply zero contact force. The original #43 feedback remains active for all earlier stages of the conditional release runs. The original controller, historical study and whole-robot withdrawal qualification retain their separate source identities. The two numerical step sizes test integration sensitivity. Hardware timing, reflective thresholds, friction, true exit observation and 545-body controller qualification remain open. [Replay source](replay.py)
