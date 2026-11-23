# Six REEFSCAPE matches

The [live lab](field-lab.html#rehearsal) uses six fixed robot slots, a live Blue and Red scoreboard, and finite, individually tracked pieces. Each chute has one attendant who takes three TELEOP seconds to stage one pipe. Processor transfers go to the opposing human player. The model allows five seconds per net attempt and leaves its outcome unresolved. These service times and capture ranges are teaching assumptions. The project-registered starting marks coincide vertically in pairs. The top-down contact solver separates overlapping footprints during playback. Twelve reef-mounted ALGAE remain at their registered retrieval stations. [Live profiles and six match totals](live-publication.json)

Both field views and the role comparison are committed static pages, generated once with Agent FRC 0.48.0 at `92ddf41933dd6026b26493c1d2c91505e9d15914`. The generator script was removed with the project's other callers of that tool; Git history keeps it.

## Archived match evidence

In the [field lab](field-lab.html#rehearsal), choose the published series or your edited roster under **Match series**, then play at 10×. Reset restores your edited robots and obstacles after a published scenario. The six games use the same 50 ms simulation steps as the single-match controls. Each has 15 seconds of AUTO and 135 seconds of TELEOP. The opening three games preserve the piece-only comparison. Three BARGE games add an endgame reservation for each participating robot. An independent clock report inserts the three-second disabled transition and records the passive cage assessment after TELEOP. The tables below are the archived six-match study.

| Match | Blue | Red | What changes |
|---|---:|---:|---|
| Qual 1 | 17 | 24 | Blue’s station runner and a partner block each other. Red misses one TELEOP trough attempt but scores with other robots. |
| Qual 2 | 24 | 47 | Red 2 clears the east face. Red 1 cycles 15 distinct pipes through the L1 trough. |
| Qual 3 | 17 | 13 | Red 1 takes one CORAL in TELEOP and holds it. Its two partners account for every red point. |

| BARGE game | Blue | Red | Endgame result |
|---|---:|---:|---|
| Mixed | 41 | 24 | Blue 2 and Blue 3 reserve different cages at 110 and 120 seconds of TELEOP. Their modeled 12 and 6 add 18 BARGE points and one qualification ranking point. Red 3 reaches a deep cage but still touches carpet. |
| Deep | 24 | 49 | Red 1 stops ordinary cycling at 95 seconds and cannot reach its cage. Red 3 earns 12. Red gives up ten ordinary points relative to Qual 2, gaining two net points. Blue 3 runs out of service time. |
| Shallow | 19 | 19 | Blue 3 parks for 2. Red 3 earns 6 in a shallow cage. Project robot Red 1 retains its pipe and earns zero endgame points. |

A climbing robot must reach its own setup-specific cage. The project jack additionally requires the receiver to be empty and stowed. Each cage has one reserved team in this exercise. The manual permits additional contacts under specified conditions. Each peer's eight-second shallow or twelve-second deep service and its final support witnesses are prescribed. In the actual swerve study the shallow jack has its own hardware, clock and eligibility. Wheels on carpet, a blocked path and a late deployment have different failure outcomes. The model charges no additional powered movement after 153 s wall time. Its three BARGE assessments are at 156, 154.5 and 153.5 s. Ranking points are recorded separately from match points. [Decision and clock ledger](barge-assessment.json) · [Kickoff manual §§6.5–6.5.4](https://manuals.plus/m/e98a748723b248f34e890accab1c49c809276fe44de713321834e30736d1e04a.pdf)

All six games start with **126 CORAL and 18 ALGAE**. Before preloads, each alliance has an assumed 30 CORAL at each of its two feeder chutes. Every preload is deducted there. Six stacked floor locations contain one CORAL and one ALGAE each. Twelve other ALGAE are mounted on the reefs. The 24 starting field items reuse the positions and identities in the [native inventory](../experiments/six-wheel/full-match/installed-stock.json). Field placement uses native X and negative Z relative to field center. Floor CORAL under an ALGAE is shown for inventory; the prescribed routes leave those pipes in place.

Each side has separately typed goals for the L1 trough, representative branches at L2, L3 and L4, the processor and the net. The upper-level markers each represent one available branch and accept one CORAL. Other branches, human-player net launches and processor recirculation remain outside these programs. Markers show the field feature; the nearby robot-center approach and its six-inch radius determine eligibility. Upper-branch and net delivery, pipe release, peer dwell times and successful or missed outcomes are prescribed teaching assumptions. The processor cannot accept CORAL. Deliberately missed attempts consume the carried piece and earn zero.

**Try the lane change.** In Qual 2, move Red 2’s hold from the normalized (290, 158.5) to (264, 158.5) inches. The two 32.5-inch bumper footprints then conflict. Red 1 waits 128.35 seconds in TELEOP and loses all 15 repeat trough deliveries. Red’s total drops from 47 to 17. The rest of the field, programs and inventory stay identical. [Paired traffic control](traffic-control.json)

The studied robot’s [primary loaded route](../experiments/swerve-drive/transit/README.md) stops with its first pipe. Qual 3 represents that strategy decision using a simplified route; its pickup time is a rehearsal assumption. The project’s independently simulated [leave-and-park role](../experiments/swerve-drive/motion/fallback.md) earns five modeled points under its own reservation. Neither outcome grants the peer programs’ points to the project robot.

### Reproduce

Use the project’s [season profile](season-profile.json) with Agent FRC 0.36.0 at commit `4154d301e28d36f0168f6df3f568fe57b549791f`. `node <plugin>/scripts/rehearsal-series.mjs kickoff/season-profile.json <new-receipt.json>` reproduces the [six-game ledger](match-series.json), including stock, cargo, clock and scores. `node <plugin>/scripts/kickoff.mjs --profile kickoff/season-profile.json --output <new-field-lab.html>` regenerates the standalone lab at a new path. This command reproduces the archived lab with that engine revision. The project’s [registration script](register_series.py) recreates the season profile from the archived field geometry and the native inventory.
