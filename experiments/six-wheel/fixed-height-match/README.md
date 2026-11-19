# Can the fixed carriage deliver twice and still park?

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

[Run the six-robot field replay](replay.html). Change the partner's path, delay the second pipe or return to the station. Drag the clock through the final three seconds to see why the scoring assessment and powered travel have different deadlines.

**The issued observer authorizes zero L1 deliveries.** It corroborates the stationary seats, while uninterrupted carried identity remains unresolved. At both placements its image, scheduler and 50 ms wait miss the reviewed withdrawal window on all 17 phases. Holding the slide instead recontacts each pipe 566.4 ms after first trough contact, at TELEOP 30.850 and 80.940 s. The reserve-lane run below is the opportunity if a qualified observer and actuator can deliver the modeled retreat. The conservative replay holds at the station after its first intake and retains only the conditional AUTO LEAVE.

The zero-dwell sensitivity meets its timing test in 11 of 17 phases for the first pipe and 12 of 17 for the second. This qualifies selected simulated withdrawal timings. The independent optics still lack a continuous identified track from station through carry and extension, and require physical calibration. The inherited timing window is a screening criterion. [Source-bound observer decisions](observer.json) · [Independent optical and native regressions](https://github.com/ehartye/frc-2025-poc/pull/59)

| Six-robot scenario | Result on one clock | Conditional points |
|---|---|---:|
| Issued interlock holds at station | The focal robot retains its AUTO endpoint and withholds carrying, placement and parking. | 3 |
| Reserved lane, stop at X = 372 in | AUTO reaches the station in 13.492 s. Both pieces settle. Red 2 parks at TELEOP 92.899 s and leaves 145.33 mm to the upright, freely swiveling cages. | 9 |
| Original X = 355 in stop | Red 2 finishes at 93.245 s. Its nominal path clears the field, but the freely swiveling cage enclosure overlaps the park leg. | 9 only for the nominal cage pose |
| Partner keeps its starting position | Red 2 overlaps red 1 at TELEOP 88.61 s on the way to park. The planned route fails. | Unqualified |
| Second pipe becomes available after its assigned human release | The robot delivers the first pipe and returns to the station at 47.629 s. | 5 |
| Return after both deliveries | Red 2 reaches the station at 97.333 s and finishes outside the parking zone. | 7 |

The nine-point case combines three AUTO LEAVE points, two TELEOP L1 pieces at two points each and a two-point PARK. Two distinct station pipes and both release times come from the [station supply](../station-supply/). All six robots move in the reserved TELEOP example. Blue 1 and 2 each back away 350 mm in their starting lanes after an 11 s or 45 s delay. Only red 2 moves in the modeled AUTO. The other robots' geometry uses a 850 mm square, 1059 mm tall allowance. Actual alliances must reproduce the reservations and stay inside those envelopes. [All 15 robot-pair checks and four results](match.json)

## Follow each pipe

| TELEOP seconds | coral-A, −180 mm sector | coral-B, +220 mm sector |
|---|---:|---:|
| Pipe exits human chute | 3.672 | 54.073 |
| Java optical model first indicates seat | 5.032 | 55.434 |
| Chassis starts carrying | 9.032 | 59.435 |
| Pipe clears modeled shelf | 30.199 | 80.289 |
| First paired target contact | 30.283 | 80.373 |
| Two-second post-impact observation ends | 32.283 | 82.373 |
| Stationary paired support before the 138 s assessment | 106.972 s | 56.882 s |

These trials inherit the 560 mm fixed carriage, zero intake stroke, 0.65 m/s counterfeed and 370 mm placement, with the original 10 mm chassis station datum. Each attained center position, velocity, axial spin, release clock and identity enters the release dynamics. The four-second hold after the modeled seat remains on the original drive clock. The slide withdraws 110 ms after modeled shelf clearance. The issued fixed-slide counterexample recontacts the pipe. [Original hardware and failed controller settings](../design/fixed-height/README.md) · [Source-to-assessment replay](placements.json)

The first and second carried pipes clear all 24 installed stock objects by at least 33.8 and 53.0 mm along the simulated 0.5 ms path segments. During chassis travel, each moving pipe enclosure stays at least 169.1 mm inside the modeled chassis footprint. The second pipe also clears the settled first pipe by 77.8 mm. Both sectors stay below the other 12 scoring pipes, with at least 160.2 and 161.4 mm of separation under the registered transverse orientation. Both rule editions return positive L1 criteria at 138 s. No post-buzzer commands run. Floor pieces, reef-mounted balls and each consumed human feed retain separate identities. [Placement and branch results](placements.json) · [Edition-specific observations](match.json)

## Where the route is tight

The closest moving pair is red 2 and red 3 at 42.75 s. The 20 ms continuous motion bound leaves **105.5 mm** between their modeled envelopes. The whole-roster rectangle bound against starting stock reaches **7.85 mm**, between the focal chassis envelope and a reef ball. The complete fixed-carriage robot's own retracted component check covers the same carrying route. Both margins depend on localization and tracking. One reserved path passing leaves the partner-held path blocked. [Registered chassis component check](../design/fixed-height/transit.json)

At the second target the earlier settled pipe has **37.19 mm** of clearance from the receiver's nominal stroke envelope. The check encloses the first pipe with an extra 2 mm on each face. The placement stroke clears the registered reef balls by at least 160.8 mm on the first sector and 164.3 mm on the second. The extended receiver and flexible parts still need measured tolerances and a first-departure interlock. Bumper material overlaps the legal starting tape by 23,574 mm³. The AUTO end clears that tape by at least 5.847 m. At the original 355 in stop, the front and rear red bumper cloth occupy 339,521 mm³ of the red parking region at field (355 in, 95.5625 in), heading 180°. [Native geometry and region intersections](native.json)

The installed Agent FRC 0.21.0 route checker supports the stationary front-cover event during TELEOP 135–138 s against the 24 placed stock bodies. Whole-field qualification remains pending in this bounded subassembly request. The active match uses project dynamics and trajectory sources. [Installed checker result](park-event.json)

Moving the parking stop to **X = 372 in** at the same Y and heading shortens the final straight by 431.8 mm. The front bumper still has 200,577 mm³ inside the scoring zone. The 10 ms swept-square check covers the complete new parking trajectory and all six hanging points. It leaves 145.33 mm to the widest freely swiveling, upright cage. Allowing the comparison 34.84 mm tracking error leaves **110.49 mm** of lateral swing, equivalent to a **4.05°** assumed cone. The 355 in path instead has a −15.26 mm enclosure bound. That result withholds a guarantee for arbitrary yaw. Measured swing, tracking, and the earlier traffic and stock corridors still govern either mission. [Hanging hardware source](../leave-park/cage-motion.json) · [372 in bumper event](inboard-event.json)

## Conditions before a team relies on the nine points

The independently corroborated stationary optical seat leaves physical custody through travel, whole-robot separation and executed withdrawal open at this height. The reference release model assumes centered feeds, no adverse bounce and the specified belt and shelf friction. Its post-impact simulation constrains the pipe to the transverse plane. The 577-body low assembly and nominal field geometry assume a standing field and unchanged bumper material. Measured cage motion, floor and joint tolerances, drive balance and traction, source variation, tracking errors and end-of-match officiating remain open. The 65.757 kg upper seated baseline plus 0.252 kg of low lock hardware stays below the 67 kg route-model mass ceiling. Final inspection mass, secure locking and verified power isolation also remain open. The real am-3583 robot signal light (RSL) at the earlier 1020 mm mounting plane reaches 1072.7 mm, exceeding the 1066.8 mm start-height limit by 5.9 mm. An outboard mount with a proposed 0.125 kg allowance requires a new configuration check. All match geometry here remains pinned to the 577-body modeled robot.

Reproduce from the project root with its Python CAD libraries and installed Agent FRC.

```powershell
python fixed-height-match/placements.py
python fixed-height-match/native_field.py
python fixed-height-match/observer_gate.py low-observation
python fixed-height-match/rehearse.py
python fixed-height-match/viewer.py
python fixed-height-match/park_event.py "$env:AGENT_FRC_PLUGIN_ROOT" artifacts/low-match/park
```
