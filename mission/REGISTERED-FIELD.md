# A registered drive from the Red 3 station to the reef

Open the [interactive field replay](registered-field.html). The play button and clock scrubber show the occupied robot, wheel speeds, two distinct pipe observations and five other robots holding their declared poses. The [source-pinned desktop results](registered-field-receipt.json) use 96 reopened robot solids, finite source-registered trough panels, the actual eight-sided field boundary and a differential chassis that must turn before it can travel sideways.

| TELEOP event | First pipe, sector −180 mm | Second pipe, sector +220 mm |
| --- | ---: | ---: |
| Release from the station lip | 16.319 s | 66.606 s |
| Sustained two-panel/lip support observed | 45.334 s | 95.307 s |
| Final empty robot recovery | | 114.213 s |

The robot starts at the mirrored center slot from the [project rules model](../kickoff/rules-v0.json), then approaches the station empty. The slot's lateral coordinate is a scenario assumption. Each loaded traverse uses the actual constructed robot's 850 × 850 mm travel envelope, forward/reverse straight segments and in-place turns. The first path is 8.949 m and takes 17.081 s. A direct route through the red reef is blocked. The nominal wheel cap is 1.25 m/s, longitudinal acceleration 1.20 m/s², turn limit 1.25 rad/s, spin-wheel acceleration 0.70 m/s² and track 580 mm. These are design hypotheses pending drive sizing and floor tests. Wheel trajectories start and end each segment at rest. The CLI exports 20 ms samples for a separate powertrain replay.

**The second sector changed because of the actual shelf.** If the second target stays at +180 mm, the first pipe is outside the second pipe's own envelope yet overlaps the ported receiver shelf by 436.650 mm³ in the reopened place STEP. Moving the second sector to +220 mm leaves at least 39.17 mm across all 96 candidate bodies through lift and slide movement. No queued command can authorize the +180 mm placement in this rehearsal.

## Registration and clearance

The [field contract](field-registration.json) references the January 9 FIRST native field export, SHA-256 `19374bc5a48fda1b3ce0505ef604a47d227c904d1784b6514a2c7d654bf3d918`. Its Red 3 station root, occurrence NAUO34, has native inch origin (312.094475, 0, −132.926599). The [January kickoff field layout](../kickoff/field-constraints.json) independently places tag 2 within 0.28 in of the resulting (657.532, 291.427) in station datum. In the project field frame the robotward normal points −125.989°. Native red reef lip source NAUO1053 and floor assembly NAUO36/NAUO1039 put the midfield-facing lip at (481.384, 158.500) in, with +field X into the trough. See the [target surfaces](target.json). The January CAD postdates the V0 game manual. [FIRST field STEP archive](https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025Field-STEP.zip)

The lower station bumper barrier projects 9.525 mm robotward of the station origin. The old centered 434.525 mm chassis distance gives nominal bumper tangency with that barrier. This trial adds 10 mm separation, keeps the encoded receiver geometry and recomputes the moving pipe from the actual registered dock. Minimum modeled field margin is 10 mm, which has no demonstrated allowance for real bumper deformation or field tolerances.

Straight segments check the complete swept chassis bounds. Turns use a circle through the farthest occupied chassis corner. The modeled cage, center barge, reefs, boundary planes, five parked robots and prior placed piece are included. All five other robots occupy explicit 850 mm square footprints. Their positions are scenario assumptions, not live observations. A negative run parks one of them in the used corridor and stops before crossing. Field fasteners, under-structure details, moving traffic, localization error, tip and traction remain separate gates.

## When is the whole robot clear?

For this candidate the 1 mm shelf witness occurs at 26.928 s after the first station release. Belt cartridges remain possible contacts until 26.971 s. The complete body check follows the pipe, moving slide, half-speed middle stage and fixed structures all the way through sustained support. Conservative swept boxes enclose each body from the exact reviewed STEP inventory. The bounds include a relative-motion allowance between samples. The last possible-contact time is an upper bound. The closest conservative body margin after the first 250 ms of target contact is 52.98 mm in this selected case.

At the end of the observation the first pipe has independent floor/lip support. Both selected pipe states are propagated through the shared 135 s horizon, and the tiny observed center drift is included in the previous-pipe occupancy check. The second feed uses a new identity, stock release time, measured-from-model own-robot separation and the new sector screen. The previous synthetic clear-to-withdraw input is unnecessary for this fixed-traffic desktop scenario. Within its declared roster, both V0 instantaneous L1 screens return yes. The later edition still lacks an other-level exclusion observation. Both leave official earned points unknown.

Actual modeled retreat begins 127 ms after the finite-pipe separation witness, including the observer plus control-delay example. The [sampled latency grid](release-latency-grid.json) checks 50–150 ms in 2 ms increments, additional odd millisecond observations, and 500/600 ms controls in both the original centered and registered approaches. Early tested values sustained floor/lip support for the nominal friction/roll; 500 ms left little clearance and 600 ms recontacted. No universal deadline follows.

## Reproduce

```sh
python mission/replay_registered_field.py --wheel-csv wheels.csv --output field-result.json
python mission/render_registered_field.py wheels.csv field-result.json field-replay.html
python mission/replay_registered_field.py --next-sector .18
python mission/replay_registered_field.py --blocked
python mission/replay_registered_field.py --seconds 15
python mission/check_release_latency.py
python mission/check_first_station_source.py /path/to/FE-2025.STEP
python scripts/station_l1_mission.py --dt .001 --sample-every-step
```

The registered field run uses OCP 8.0.1 to reopen the named component assembly. The final command exports dense bench-frame pipe/receiver measurements and pitch assumptions for observer testing. The geometric path and repeat receipt describe a provisional team option. Actual robot qualification still requires selected hardware, calibrated sensors and contacts, dynamic traffic and robot inspection.
