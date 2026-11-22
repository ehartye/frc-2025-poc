# A station-fed L1 role

The red middle robot starts empty, earns LEAVE, then takes one CORAL from the station, carries it to its own reef and releases into L1. It returns for a fresh CORAL and uses a second sector so the first pipe becomes an obstacle on the next trip. A final stowed park completes the proposed nine-point role. This is the team's later selected scenario. The [kickoff-day narrative](field-lab-draft.md) preserves the alternatives students had before that choice.

| Opportunity | Count or time window | Requirement on the robot | Current simulation evidence |
|---|---|---|---|
| Leave the starting line | End of 15 s AUTO | Begin on the tape and clear it with the entire occupied robot | Four signed opening runs earn 3 points. [Bumper/tape intersections](../experiments/swerve-drive/motion/opening-line.json) |
| Two TELEOP L1 deliveries | Two distinct pipes in 135 s, plus travel and parking | Count station supply, returns, queues, release and retained score on the same clock. | Both signed cam receivers carry their station-fed pipe to the reef. The installed grip foot intersects each predicted exit path. The primary route has zero accepted deliveries. [Attained docks and release audit](../experiments/swerve-drive/low-placement/study.md) |
| Sustain a deliverable load | From station capture through sector approach | Preserve custody when accelerating, braking or crossing another robot's route. Detect actual displacement or loss. | The shifted lane clears the tested peer reservations; ignoring a camera veto later moves the pipe 20.35 mm into a hard stop. [Mechanical and optical counterexamples](../experiments/swerve-drive/transit/decision-evidence.json) |
| Deposit into the intended trough | A separate release window at each sector | Clear the wall with the whole robot and hollow pipe, then verify support after withdrawal. | At the measured docks, a 337 mm command preserves wall clearance. The reduced model reaches support, but the real receiver's front corner intersects the pipe first. [Signed native interference](../experiments/swerve-drive/low-placement/results.json) · [Earlier independent rim launches](../experiments/swerve-drive/score-stroke/study.md) |
| Park | End of TELEOP and assessment | Reserve the south lane, stow, reach the zone and stop | The separately chosen empty-start fallback completes 156 s and earns 5 total points. It has its own starting configuration and six-robot reservations. [Fallback study](../experiments/swerve-drive/motion/fallback.md) |

Two deliveries in 135 s correspond to an average of one delivery per 67.5 s before reserving time for other actions. That arithmetic alone cannot size the loader. The route model has to establish when station stock is actually available and the maximum time left for each full trip. A faster feeder cannot recover a blocked passage or a lost pipe. [Kickoff field model](season-profile.json) · [Full-match acceptance conditions](../experiments/swerve-drive/motion/match-qualification.md)

## Choosing the final action

PARK earns 2, shallow CAGE 6 and deep CAGE 12. A shallow climb buys four points above a successful park at the cost of travel, mechanism service and any missed deliveries. At 130 s wall time, the supplied empty approach and forward jack finish the conditional lock check at 148.68 s. The 140 s control misses cutoff. Keep the two verified five-point leave-and-park traces available. The occupied receiver has no demonstrated climb route. [Six-match BARGE comparison](series.md) · [Forward jack and load limits](../cad/shallow-cage/study.md)

## Compare station routes and size the complete robot

The strategic narrative and case-study workflow compare the same five other players, change Red 1’s station, and test depletion of that station. The mechanism-CAD working-load study uses the complete front and rear jack layouts, operating robot mass, simultaneous roll and pitch reactions, and the actual stopped receiver traces. [Rehearse the selected role](https://ehartye.github.io/frc-2025-poc/role-lab.html#roleLab) · [Study, inputs and measured outcomes](../experiments/swerve-drive/whole-robot-trade/study.md)
