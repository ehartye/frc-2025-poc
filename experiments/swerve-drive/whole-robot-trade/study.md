# Does the station runner earn its place?

Keep the south lane available for an empty leave-and-park assignment. Choose the station-fed role when the alliance can clear the reef approach and supply two CORAL. The current simulated receiver holds its first pipe at the reef. Its two signed runs have earned zero L1 deliveries. The separately rehearsed empty route earned five modeled points from its own starting reservation. The front shallow jack remains a design candidate. [Same-pipe carry](../cam-scoring/study.md) · [Reserved park](../motion/fallback.md)

## Change one robot on a six-robot field

In each published setup, Red 1 starts empty and stages at the same lower station in AUTO. During TELEOP it either cycles CORAL, keeps its first pipe, or clears the reef lane before attempting a park or shallow climb at 107 seconds. The other five robots retain their equipment, preloads, programs and endgame. Both comparisons run on the complete 15- and 135-second powered periods. Add the three-second transition when mapping to wall time. The endgame engine awards once per robot and evaluates contact after the active clock. [Season clock](../../../kickoff/barge-assessment.json) · [Field study inputs](alliance-study.json) · [Per-robot times, pieces and awards](strategy-lab-result.json)

| Red 1 assignment | Qual 3, Blue–Red | BARGE 3, Blue–Red | What Red 1 actually did in the model |
| --- | --- | --- | --- |
| Repeated L1 | 17–13 | 19–31 | Waited 132.85 s behind Red 2 with no delivery in Qual 3. In BARGE 3 it acquired and scored six distinct pipes for 12 points. |
| Retain first pipe | 17–13 | 19–19 | Acquired one and stayed at the chute in both games. |
| Clear lane, then park | 17–13 | 19–19 | Failed to reach the park target in both games. |
| Clear lane, then shallow cage | 17–13 | 19–19 | Never made contact. The six red endgame points in BARGE 3 belong to its partner. |

**Two deliveries require two pieces and two openings.** The separate Qual 2 counterfactuals keep the entire 135-second TELEOP window and Red 1’s scoring program. Two preloads draw from the red lower chute before the first sample. Each accepted event has its own acquisition time, pipe identity, position test and field-engine scoring event. All successful field attempts prescribe a good L1 release. [Four full-window inventories and event lists](hardening-inputs.json) · [Rate and timing results](throughput-result.json)

| Ally hold X (in) | Chute stock after preloads | Accepted L1 | Delivered per minute | Time after last acceptance |
| ---: | ---: | ---: | ---: | ---: |
| 290 | 28 | 15 | 6.67 | 1.85 s |
| 264 | 28 | 0 | 0 | 135 s |
| 290 | 1 | 1 | 0.44 | 127.15 s |
| 290 | 0 | 0 | 0 | 135 s |

The selected role demands two deliveries in 135 s, or 0.89 per minute. One of these four rehearsals meets it. The limiting rate is zero. In the detailed robot model both wheel-calibration signs capture `coral-A` and inhibit release at their attained docks. Neither supplies a full match window containing an observed L1 award. The integrated repetition report therefore retains both windows as open. Replaying one independent successful rim fixture twice would reuse an initial condition and supply no second pipe. [Integrated episode coverage](robot-throughput-result.json) · [Signed robot mechanics](../cam-scoring/results.json) · [Landing criterion](../score-stroke/study.md)

## Use the other CORAL STATION

The first matrix identified a deadlock between the lower station and the opponent-facing side of the red reef. Preserve the initial position and AUTO program. In TELEOP, send Red 1 to the other registered CORAL STATION and keep the same L1, wheel speed, pickup/release dwell, piece inventory, and five other robots. Each named supply owns separate finite pipes. The before-and-after clocks show a change in both the focal runner and the teammate that it releases from traffic. [New programs and controlled replay](replay.py) · [Event-by-event comparison](refinement-result.json)

| Red 1 route | Qual 3 deliveries / wait | BARGE 3 deliveries / wait |
| --- | ---: | ---: |
| Issued lower station | 0 / 132.85 s | 6 / 83.05 s |
| Lower station with an outside return waypoint | 1 / 126.20 s | 1 / 126.20 s |
| Lower station with an inside waypoint | 0 / 132.85 s | 6 / 82.95 s |
| Other station, direct to L1 | **15 / 0 s** | **15 / 0 s** |
| Other station, extra reef waypoint | 14 / 0 s | 14 / 0 s |

With the direct route, Red's total increases from 13 to 49 in the first game and 31 to 55 in the second. The extra six points in each game come from the other red players. With one pipe remaining in the other station the runner delivers one in both contexts. With that station empty it delivers zero in both. Announce the station assignment before the match and reserve its supply. [Opposite-station stock controls](refinement-stock.json) This reroute is the leading field strategy for these two partner configurations. Its different station approach still needs a new sensor registration, matched physical pickup, deck and cable clearance, and qualifying releases before the actual robot can adopt it.

### Can either receiver supply that route?

Selecting Red 1’s opposite-station route yields **15 different accepted pipes in 135 seconds** at red L1, or 0.1111 accepted pipes/s. The requirement belongs to that focal robot, station and interval. [Selected field execution](role-demand.json)

The open stereo receiver stopped with the original pipe still aboard at 30.120 s and 31.960 s. On those same saved chassis and pipe states, the four-pin receiver and two end probes granted a new departure after 255 ms in each case. The pin contacts retained the pipe, but the probe limit stopped the modeled carry at 37.055 s and 40.155 s. Both receiver architectures have zero observed L1 releases in these traces. Neither supplies the selected 135-second scoring evidence. The independent route change shows what traffic would permit if a receiver could sustain it. [Matching incoming state hashes and achieved observations](custody-executions.json) · [Architecture comparison against the selected role](custody-review.json)

## Where to put a jack on the complete robot

The rear- and front-mounted designs both carry the camera, sliding receiver, four swerve modules and bumpers. Each was checked against its own complete robot and the same supplied red and blue cage solids. In the rear layout, the rear-left controller tray occupies space needed by the jack; the fixed camera also enters the cage during approach. The front layout moves the pair of columns to the opposite crossrail and permits a straight-on entry. [Signed complete-assembly results](robot-cases.json) · [Whole-robot comparison](design-review-result.json)

| Actual design and motion | Rear candidate | Front candidate |
| --- | ---: | ---: |
| Bodies in the intended approach | 13 | 0 |
| Bodies in the opposite lateral approach | 14 | 7 |
| Bodies in the suspended travel check | 4 | 0 |
| Interfering internal stroke pairs | 7 | 0 |

Both alliances give the same counts. A lateral entry remains blocked for both designs. The later front-jack manufacturing issue adds a constraint the earlier approach study omitted. At 370 mm of receiver travel, the installed carrier's power and USB jackets enter the right jack motor by 457.30 and 318.53 mm³ respectively. Keep that complete-motion manufacturing hold. The proposed revised cable carrier has a separate geometry and bill. [Cumulative receiver and jack](../../../cad/assembly-engineering/receiver-service/verification.md#moving-camera-and-cable-carrier)

## What carries the robot after power is removed?

Only the front twin-screw jack elevates the chassis. The swerve tires carry the floor reaction before lift-off; the receiver carries a game piece and must be empty and stowed to climb. The current conditional inspection upper is 52.139767 kg. Add the 6.170 kg battery and 7.089134 kg from the 52 removable bumper articles for an operating mass of **65.398901 kg**. Restoring the entire earlier general allowance in addition to the now-identified camera equipment gives 66.531751 kg in operation and exceeds inspection mass by 1.10949 kg. [Mass and article sources](../../../cad/assembly-engineering/receiver-service/issue/mass.json) · [Operating mass and screw-drive calculations](robot-cases.json)

The real screw axes are at X305 and Y±321 mm. At the heavier side of a 65/35 split under 1.5 times gravity, one Roton 59285 screw and 90234 bronze nut carry 625.31 N. The 8.45 kN catalog nut rating is 13.52 times that axial demand. At 40 mm/s the 68/13 gear and assumed 32% screw efficiency require 0.987 N·m at the screw and 7.535 V at the motor, leaving 1.965 V against the assumed 9.5 V terminal supply. The frictionless reverse demand is 0.316 N·m per heavier-side pawl. Its case, pivot, tooth and return spring require an applicable holding capacity. [Purchasing and geometric controls](../../../cad/assembly-engineering/jack-transmission.json) · [Working-load review](elevation-result.json)

One remaining screw at full design acceleration would see 962.02 N. At 20% screw efficiency and 8.5 V the motor falls short by 3.586 V at the requested speed. At power loss, transferring all static weight to the positive-Y screw also leaves 180.22 N·m of transverse moment for the guides and frame. Neither screw torque nor a correctly drawn bearing rates those joints. Across the eight assumed longitudinal/transverse CG corners, the retained forward cage model requires approximately 4.34 kN upward and 3.70 kN opposing contact reaction. The field plate’s allowable load, the front mount’s stiffness, paired motion under skew and the pawl’s power-off holding capacity remain open. The scored match-climb count stays zero. [Registered clamp contact](../../../cad/shallow-cage/contact.json) · [Loaded cage posture and timing](../../../cad/shallow-cage/lift.json)

The spatial free-body calculation checks sixteen combinations of lift, power loss and CG position. At CG X−60/Y−40 mm, both vertical screw reactions leave a 351.14 N·m pitch couple to be carried by the crosshead and cage during a 1.5 g raise. At rest, the pitch demand is 234.09 N·m. Losing the negative-Y support adds a 231.52 N·m roll demand in the opposite sense. A catalog axial rating does not establish either moment capacity. [Signed reactions and unresolved moments](spatial-result.json)

A selecting test would run the exact current rack, both jack drives and the corrected carrier through one continuous station-to-reef trip. A repeat would pick up a different pipe from remaining stock, reset the grip, and demonstrate another legal support before cutoff. The separate climb test needs each engaged lock to retain its share of the operating robot at the limiting tilt, including a one-side failure. Keep the empty start’s proven park route available during these tests.

## Reproduce

The field, design and hardening commands use [Agent FRC at immutable commit ba9241e7](https://github.com/ehartye/agent-frc/tree/ba9241e724d2425fcf007a9d125e8f87186d68bc). Run `python experiments/swerve-drive/whole-robot-trade/replay.py PATH_TO_AGENT_FRC_SCRIPTS` from the project checkout. Each input names the season-owned profile or a project receipt by SHA-256. The [field program](../../../kickoff/season-profile.json), [role comparison](alliance-study.json), [architecture constraints](design-study.json), [elevation inventory](elevation-study.json) and [two-piece demand](throughput-study.json) can be inspected independently.
