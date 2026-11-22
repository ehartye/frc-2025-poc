# Can the two shelf sensors save an off-center feed?

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 83b3b9c7fe](../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

Keep the scoring-route feed on hold. The wheel-error trials briefly give the controller an early warning, but cutting or reversing the belt still lets the incoming pipe reach the cartridge stop. A separate 3 mm delivery shift reaches the stop before either sensor changes. Choose the reserved south start before the match if the team needs the demonstrated alternative. Its 543 mm drive preserves a conditional three-point LEAVE and two-point PARK and gives up two possible L1 placements worth four points. No points are assigned to the station approach. [Reserved south start](../station-approach/README.md)

The [passive guide trial](../station-guide/README.md) tests whether the pipe can be centered before that warning arrives.

## What the robot can see

We reopened the nine issued physical traces, verified their hashes, reconstructed the two digital returns from the finite pipe at each 20 ms exposure, and passed those returns through the original compiled Java program. Time below is relative to the controller epoch at match wall time 21.571673 s. The pipe leaves the chute near tick 100 ms.

| Incoming case at the 0.25 ms physics step | Two-port evidence | First cartridge-stop demand |
|---|---|---|
| Five seated drive controls | Entry at 320 ms, both ports at 340 ms, Seat alone at 360 ms | Clear through scheduled carry in the issued traces |
| Left tire small | Entry at 320 ms, both dark at 340 ms, Seat alone at 360 ms | 367.75 ms |
| Right tire small | Entry at 320 ms, both dark at 340 ms | 356.50 ms |
| Delivery shifted 3 mm or unobserved 3 mm camera displacement | Both dark throughout the available trace | 302.00 ms |

The challenge controller watches for Entry alone followed by two dark ports. It can choose zero or −0.65 m/s belt demand immediately on that observation. Both demands still pass through the original 26.25 m/s² drive ramp. The belt needs 24.76 ms to slow from +0.65 m/s to rest. The right case allows 16.50 ms after the warning. Even the longer 27.75 ms left case reaches the stop under either command. The original two opposing outcomes change with the physics step, and in one right-side refinement the warning never arrives. The two 3 mm disturbances have the same complete sensor history as all five passing controls before impact. A command keyed to these returns cannot distinguish those disturbances in time. [Compiled port histories](port-witness.json) · [Closed-loop command trials](motion.json)

## Why the five-of-seven result stays conditional

The original study ran one contact step of 250 µs. Below, each row holds the recorded wheel error, incoming pipe and initial spin, forward belt demand, preload, friction and nominal 1.40 mm outward stop fixed. Values are the largest outward travel observed in mm up to 330 ms from the lip, or through the first excursion past the stop. Values on a stopped trace are first-step exceedances. They cannot size a replacement mechanism.

| Contact timestep | Left tire small | Right tire small | Nominal |
|---|---:|---:|---:|
| 500 µs | 0.578 | 0.679 | 0.134 |
| 250 µs | 2.063, stop | 1.409, stop | 0.134 |
| 125 µs | 1.925, stop | 1.407, stop | 0.135 |
| 62.5 µs | 1.331 | 1.401, stop | 0.304 |
| 31.25 µs | 0.850 | 1.331 | 0.135 |
| 15.625 µs | 1.401, stop | 0.679 | 0.135 |

A separate contact-area refinement uses 5, 10 and 20 strips per belt at 250 µs. Both opposing cases stop at every strip count. Agent FRC 0.22.2 returns `unknown` for both because some simulations leave the physical model domain before the common horizon. The nominal case returns `within-declared-bounds` for the two finest comparisons under the declared 0.05 mm peak-travel tolerance. Its coarser 62.5 µs excursion is visible in the table. All three results keep engineering acceptance blocked. These checks cover the initial encounter only. A held pipe, time to seating, material stiffness and contact forces require separate evidence. [Refinement results, hash pins and endpoints](refinement.json)

Before relying on a human feed, independently measure pipe motion relative to the two cartridge faces, including first contact, lateral offset, pipe-axis attitude and actual stop travel. Include both signed wheel cases, the adverse delivery shifts and the settled pipe. Compare time histories and forces with an independently checked compliant-contact model before extending the tested acquisition domain. A warning after the pipe has already left the chute cannot recall it.

## Reproduce

Use the project Python environment and JDK 17. Generate the hash-bound raw physical traces using the [opening-station instructions](../station-landmark/README.md). Supply paths to the installed Agent FRC package and a private artifact directory. The refinement runner freezes the manifests before simulating both axes, retains inputs and full checker outputs in the artifact directory, and writes the compact tracked review.

```sh
python station-feed-margin/port_witness.py "$OPENING_TRACES" "$JDK17"
python station-feed-margin/motion.py "$OUT" "$JDK17"
python station-feed-margin/refinement.py "$AGENT_FRC_PLUGIN_ROOT" "$OUT"
node "$AGENT_FRC_PLUGIN_ROOT/scripts/check-contact-convergence.mjs" "$OUT/leftSmall-manifest.json" "$OUT/leftSmall-receipt.json"
python -m unittest discover -s tests -p test_station_feed_margin.py
```
