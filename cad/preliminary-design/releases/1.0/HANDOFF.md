# Handoff: Fixed pocket 1.0

For detailed design. Each figure names the file that holds it; paths are from this folder unless they start at the repository root. The design is in progress until the owner marks it effective.

## Design

| | |
| --- | --- |
| Name and version | Fixed pocket 1.0 (issue `fixed-pocket-1.0` in `judgement.json`) |
| Architecture | Four-module swerve carrying a fixed pocket. A station CORAL drops into the pocket, a two-roller nip stops and holds it, and a deploy-once exit lip rolls it off into the L1 trough. |
| Design source | `design.json`: 72 named solids, one per functional volume, millimetres, with sizes worked out from its `parameters`; groups, mounts, contacts, poses, motions, the pipe, materials, nip drive, adjustments and trials. `generated/design.step` and `generated/manifest.json` are what the design library builds from it. `layout-blocks.json` is draft 4's record of each solid's function, parent and what it leaves out; the solids are unchanged. |
| Field | `field/*.step`: 18 official FIRST solids at the two docks, one file each, placed as fixtures in `design.json`; the first scored pipe is a fixture drawn there. `generated/fixtures.step` holds them as built. Source and placement: `evidence/official-fixtures.json`, `cad/field/official/`. Parameters `dock_gap` and `reef_standoff` move the station and reef parts. |
| Builders | None. `python -m frc_design_kernel build design.json --out generated` regenerates the solids; `try` runs the trials. |

## Frame and datums

| Item | Value | File |
| --- | --- | --- |
| Origin | Chassis centre on the tire floor, Z = 0 | `design.json` `frame` |
| Axes | +X toward the reef (delivery) face, +Y port, +Z up | `design.json` `frame` |
| Frame outline | 740 x 730 mm with two 17.25 mm rear plan chamfers | `layout-blocks.json` `frameOutlineMm` |
| Nip throat (pipe centre in the bite) | X 335.0, Z 600.0 | `layout-blocks.json` `poses.throatMm` |
| Lower and upper roller axes, about +Y | (335.0, 523.45) and (335.0, 676.55) at the 6 mm design squeeze; rigid core radius 16.4. The `squeeze` parameter (the `nip squeeze` adjustment, 4 to 8 mm) moves both | `design.json` `parameters.lower_roller_z`, `upper_roller_z` |
| Carry seat (pipe centre) | X 304.13, Z 599.97 | `poses.seatMm` |
| Check flap hinge, about +Y | (161.25, 706.03); opens 74.26 deg | `poses.flapHingeMm`, `poses.flapOpenDeg` |
| Exit lip hinge, about +Y | (366.0, 530.5); 120 deg from stowed to deployed | `poses.lipHingeMm`, `poses.lipDeployDeg` |
| Pocket floor | 15.74 deg; contact polyline through (-340, 795), (-268.1, 701.9), (306.6, 539.9) | `poses.inlet` |
| Mount points | Each mechanism's host and point | `design.json` `mounts` |

## Envelopes

| Item | Value | File |
| --- | --- | --- |
| Bumpers | Ring 75.5 mm outside the frame, Z 43 to 181; outer faces at X 445.5 and -445.5 | `layout-blocks.json` block `Bumper ring` |
| Starting configuration | Nothing beyond the frame front (lip stowed 1.0 mm inside it); height 886.7 mm | `layout-blocks.json` `rules` |
| Greatest reach | Deployed lip tip 100.0 mm beyond the frame front; 109.0 mm during the deploy swing | `rules.deployedProjectionBeyondFrameMm`, `rules.deploySwingMaxProjectionMm` |
| Parked | Lip deployed, pocket empty | motion `withdraw-park` in `design.json` |

## Interfaces and allowances

| Interface | Allowance | File |
| --- | --- | --- |
| Reserve between distinct robot solids | 2.0 mm checked; mated faces are drawn 2.1 mm apart (3.1 mm beside a moving solid, 4.0 mm steer sweep to housing) | `layout-blocks.json` `poses.partitionMm` |
| Tread squeeze on the pipe | 6 mm per roller; the outer 9 mm of each 2 in tread is trial-qualified contact, not a checked clearance | `layout-blocks.json`, blocks `Lower nip roller`, `Upper nip roller` |
| Tread stiffness band | 40 to 120 N/mm per roller at the 6 mm squeeze | `evidence/field-trials.json` `summary.acceptanceBandNmm` |
| Hood above the seated pipe | 10 mm | `poses.hoodGapMm` |
| Pipe on the tray, flap, lip, chute and trough | Trial-solved contact, declared for the motions it holds in | `design.json` `contacts`; draft 4's per-case records in `evidence/interfaces/` |
| Unloaded tread to every neighbour | 2.0 mm at the closest | `layout-blocks.json` `treadClearance` |
| Lip over-travel on its sprung stop | 31.2 deg; 2.1 mm to the nearest neighbour | `layout-blocks.json` `lipOverTravel` |
| Held by a registered mount with no bracket drawn | 17 solids: eleven electronics on standoffs, the CAN trunk, the latch servo and the four steer sweeps | `receipt.json` `attachment.heldByMountOnly` |

No fit in this layout is declared tighter than the 2.0 mm reserve. No solid carries an unmounted-allowance label.

## Operating point and windows

Planar free-piece trials on the official field section, `evidence/field-trials.json`. Each row gives the operating point, then the passing and failing value on each side.

| Input | Operating point | Passes | Fails |
| --- | --- | --- | --- |
| Tread stiffness, receive | 80 N/mm | 30 N/mm | 20 N/mm (11 of 12 seat) |
| Tread stiffness, eject | 80 N/mm | 160 N/mm | 240 N/mm (drive stalls) |
| Dock stand-off at the reef | 50 mm | 0 to 100 mm | 125 mm |
| Dock gap at the station, bumper to lower barrier | 10 mm | 0 to 50 mm | not found; 50 mm is the widest run |
| Lip tip beyond the frame, at 100 mm stand-off | 100 mm | 100 mm | 72 mm |
| Check flap | present | present | removed (0 of 2 retained when leaving) |
| Hit into the nip, main power off, 40 N/mm | 8 g for 50 ms | 16 g | 24 g |
| Deceleration into the nip, main power off, 40 N/mm | 1.5 g for 0.5 s | 6 g | 8 g |
| Bus voltage, eject | 12 V | 9.0 V | not found; 9.0 V is the lowest run |

Counts at the operating conditions: receive 108 of 108 seated, eject 480 of 480 captured and supported 2 s after the withdrawal, powered carry 72 of 72, receive-and-go 24 of 24 (`summary.g1`, `g2`, `g3`, `go`). Controls the trials reject: latch failed with the lip stowed, robot disabled at the eject, check flap removed, rollers decoupled on a 20 N/mm tread.

The nip controller must be set to brake idle mode (`design-findings.json` PD17).

## Motion cases

Checked with the agent's design tool on the design library (`checked-with.json`); results in `receipt.json`, motions in `design.json`. A receive or eject motion is one motion through the solved poses of its recorded trace, the same poses draft 4 checked as straight cases.

| Phase | Motions (draft 4 cases) | Path tolerance |
| --- | --- | --- |
| Acquire | `receive-chute` (receive-01 to 03), `receive-pocket` (receive-04 to 10) | 10 mm from the solved trace |
| Retain | `leave-station` (receive-11 to 14), `seat-pipe` (receive-15, 16), `carry-power-off-hit` (carry-01, 02) | 10 mm |
| Transfer | `eject-nip` (eject-01, 02) | 10 mm |
| Deliver | `eject-trough` (eject-03 to 07) | 10 mm |
| Park | `withdraw-park` | prescribed drive |
| Lip deploy | `lip-deploy` | prescribed hinge rotation |
| Service | ten motions, `service-hood-withdrawal` to `service-breaker-reset` | prescribed lifts and reaches |
| Displaced control | hood seated 12 mm low in `receive-pocket`: rejected on the pipe and hood pair | `receipt.json` `control` |

Reset and endgame are not applicable: the pocket is fixed, the lip stays deployed, and the selected role parks (`missions.json` `cycle`).

## Budgets

| Budget | Value | Limit | Margin | File |
| --- | --- | --- | --- | --- |
| Inspection mass | 47.66 to 49.78 kg | 52.16 kg | 2.39 kg at the heavy estimate | `receipt.json` `budgets` |
| Loaded bus, weak pack, driving away | 9.12 V | 9.0 V | 0.12 V | `receipt.json` `budgets.electricalCases`, `leave-station-weak` |
| Loaded bus, nominal pack, driving away | 9.54 V | 9.0 V | 0.54 V | same, `leave-station-nominal` |

Mass by subsystem, low to high, from `budgets.json` `masses`: modules 10.89 to 11.21 kg; fabricated chassis 18.70 to 19.07; chassis hardware 0.85 to 0.90; fabricated pocket package 4.56 to 5.47; purchased pocket package 2.27 to 2.73; joining allowance 3.40; electrical allowance 5.50; battery tray, bumper mounts and remaining hardware 1.50. Both margins are thin and both are on the bench list.

## Open risks and bench tests

Every risk is in `missions.json` `risks`, closed by a named design change or carried with a measure, a pass value and a fail value. The fixtures and articles for G0 to G7 are in `bench-gates.json`.

| Test | Measure | Pass | Fail |
| --- | --- | --- | --- |
| G0 tread stiffness | Force to press a pipe 6 mm into each installed roller | 240 to 720 N | below 120 or above 1440 N |
| G1 station receive | Powered feeds captive within 0.50 s of passing the flap, of 40; disabled feeds kept, of 10 | 40 and 10 | any pipe through the nip or out of the inlet |
| G2 eject | Ejects captured and supported 2 s after withdrawal, of 150 | 147 or more, every cell 9 of 10 or better | any cell 7 of 10 or worse, or a stall at 9 V |
| G3 carry custody | Retained of 60 powered and 40 disabled pulses; receive-and-go seated within 0.5 s, of 10 | 60, 40 and 10 | any loss at 3 g or below |
| G4 two trips | Cycles with both pipes credited, of 20; lateral dock error | 19 or more; within 70 mm | fewer than 19, or any error beyond 84.7 mm |
| G5 sensing and interlock | Displaced or missing pipes flagged within 50 ms; range error; ejects accepted at 150 mm or beyond | all; within 15 mm; none | any missed loss or any eject accepted beyond 150 mm |
| G6 lip | Tip sag under the pipe; set after 50 pushes of 300 N and 5 wall hits at 1 m/s | 3 mm or less; 1 mm or less | set over 1 mm or any crack |
| G7 power-off custody | Push-through force on the unpowered nip; pipes kept nose-down 60 s, of 5; kept through 8 g, 50 ms pulses, of 20 | 61 N or more; 5; 20 | under 29 N, or any pipe lost |
| Weak-pack voltage | Lowest bus voltage over ten 3 m/s2 launches, nip on hold, weak pack | 9.0 V or more on every launch | below 9.0 V on any launch |
| Mass | Inspection weight without bumpers and battery | 52.16 kg or less | over 52.16 kg |

Closed by design change: the self-locking worm (planetary drive with a powered hold), seat time (the robot leaves on the flap-closed signal), lip stop and side-plate sizing (`evidence/structure-checks.json`), the G412 interlock (`rules-rulings.json`), the station fixture, power-off custody and the second delivery into an occupied sector (`evidence/second-trip.json`).

## Trials and replays

All trials ran on blockout geometry in a planar compliant-contact model: the pipe is a rigid ring, and stiffness, friction and restitution are swept, not measured. Repeat each on real parts.

| Interaction | Trial | Replay on the review page | Not modelled |
| --- | --- | --- | --- |
| Station receive and drive away | `receive-mission` | Recorded trial: station receive and drive away | Pipe yaw and end contact, the chute plate's slots, hood flex |
| Carry through a hit with the main power off | `custody-power-off-hit` | Recorded trial: hit while carrying with the main power off | Tread hysteresis, measured squeeze |
| Eject into the trough and withdraw | `eject-nominal` | Recorded trial: eject into the trough and withdraw | Lip flex, the riser's 245 mm extent behind the 302 mm pipe, the branch over the trough |

## First work items

1. Build the bench article and run G0: the tread stiffness band decides receive, eject and power-off custody.
2. Run G1 and G2 on the article at the two docks, then G3 and G7.
3. Draw the lip hinge brackets, sprung stop and latch on the side plates and run G6.
4. Replace the nip gearbox planning envelope with REV's model and confirm its length beside the port side plate.
5. Draw the electronics brackets and standoffs, then weigh the joining and electrical allowances against the 2.39 kg of mass headroom.
6. Re-run the loaded-voltage budget with measured drive currents: the weak-pack margin is 0.12 V.
7. Register the bumper construction and mounts and check the applicable bumper rule edition.
