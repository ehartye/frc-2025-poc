# Three REEFSCAPE matches

In the [field lab](field-lab.html#rehearsal), choose the published series or your edited roster under **Match series**, then play at 10×. Reset restores your edited robots and obstacles after a published scenario. The three games use the same 50 ms simulation steps as the single-match controls. Each has 15 seconds of AUTO and 135 seconds of TELEOP. The modeled clock covers those two active phases; disabled transitions, fouls, endgame and ranking points are outside the series. To change the roster yourself, reset the rehearsal and use **Add blue robot** or **Add red robot**.

| Match | Blue | Red | What changes |
|---|---:|---:|---|
| Qual 1 | 17 | 24 | Blue’s station runner and a partner block each other. Red misses one TELEOP trough attempt but scores with other robots. |
| Qual 2 | 24 | 47 | Red 2 clears the east face. Red 1 cycles 15 distinct pipes through the L1 trough. |
| Qual 3 | 17 | 13 | Red 1 takes one CORAL in TELEOP and holds it. Its two partners account for every red point. |

All three games start with **126 CORAL and 18 ALGAE**. Before preloads, each alliance has an assumed 30 CORAL at each of its two feeder chutes. Every preload is deducted there. Six stacked floor locations contain one CORAL and one ALGAE each. Twelve other ALGAE are mounted on the reefs. The 24 starting field items reuse the positions and identities in the [native inventory](../experiments/six-wheel/full-match/installed-stock.json). Field placement uses native X and negative Z relative to field center. Floor CORAL under an ALGAE is shown for inventory; the prescribed routes leave those pipes in place.

Each side has separately typed goals for the L1 trough, representative branches at L2, L3 and L4, the processor and the net. The upper-level markers each represent one available branch and accept one CORAL. Other branches, human-player net launches and processor recirculation remain outside these programs. Markers show the field feature; the nearby robot-center approach and its six-inch radius determine eligibility. Upper-branch and net delivery, pipe release, peer dwell times and successful or missed outcomes are prescribed teaching assumptions. The processor cannot accept CORAL. Deliberately missed attempts consume the carried piece and earn zero.

**Try the lane change.** In Qual 2, move Red 2’s hold from the normalized (290, 158.5) to (264, 158.5) inches. The two 32.5-inch bumper footprints then conflict. Red 1 waits 128.35 seconds in TELEOP and loses all 15 repeat trough deliveries. Red’s total drops from 47 to 17. The rest of the field, programs and inventory stay identical. [Paired traffic control](traffic-control.json)

The studied robot’s [primary loaded route](../experiments/swerve-drive/transit/README.md) stops with its first pipe. Qual 3 represents that strategy decision using a simplified route; its pickup time is a rehearsal assumption. The project’s independently simulated [leave-and-park role](../experiments/swerve-drive/motion/fallback.md) earns five modeled points under its own reservation. Neither outcome grants the peer programs’ points to the project robot.

### Reproduce

Use the project’s [season profile](season-profile.json) with Agent FRC 0.35.0 at commit `0fbf18d8ebb00e34fbd55f06a98e9169509cfca1`. `node <plugin>/scripts/rehearsal-series.mjs kickoff/season-profile.json <new-receipt.json>` reproduces the [three-game ledger](match-series.json), including stock, cargo, clock and scores. `node <plugin>/scripts/kickoff.mjs --profile kickoff/season-profile.json --output <new-field-lab.html>` regenerates the standalone lab at a new path. The current and historical lab links both ship these same bytes. The project’s [registration script](register_series.py) recreates the season profile from the archived field geometry and the native inventory.
