# Can this robot place two CORAL and still park?

[Open the full match replay](replay.html) and scrub the clock. Red 2 starts empty on its starting line, drives to the station in AUTO, waits through the disabled transition and places two individually identified pipes in TELEOP. Toggle **Earlier AUTO route and collision** to see the floor stack that changed the approach.

## What the route supports

| Scenario | Two-second support confirmations in TELEOP | End of motion in TELEOP | Geometric result |
| --- | --- | --- | --- |
| Partner clears, no initial wait | 31.67 s and 81.38 s | 93.25 s | Park footprint overlaps the red BARGE ZONE by 0.766 m. |
| Partner clears, 16 s wait | 47.67 s and 97.38 s | 109.25 s | The same two deliveries and park fit. |
| Partner stays on its starting slot | Released route withheld | Predicted collision at 88.61 s | Red 2 reaches red 1 on the parking route. |
| Return after second delivery | 31.67 s and 81.38 s | 99.33 s | Finishes at the station outside the parking zone. |
| Second pipe unavailable, return to station | 31.67 s only | 52.02 s | The station cannot produce a second observation. |

At the assumed zero-preload setup, each alliance receives 60 station pipes, with six additional pipes on the field. Six ALGAE initially balance on these floor pipes. The six stack centers lie 48 in from either alliance wall and at 86.5, 158.5 and 230.5 in across the field. The 14.7 in half-width assumes centered configurations and nominal dimensions. The six-robot trial uses the actual transport envelope, 580 mm track and the 67 kg drive limits. The shortest bound between moving robots exceeds 90 mm in the no-wait plan. The smallest bounded station-wall clearance is only 10 mm. Approach and coordinate errors of that scale matter. [Trial record](field-trial.json)

The old 1.25 m alignment turn reaches the upper red stack at AUTO 11.58 s. A nominal pipe across the field intersects 804 mm³ of red cover and 32,628 mm³ of foam in a native solid comparison. A separate centered, hollow ALGAE case also touches the cover. The corrected route aligns 0.65 m from the station datum and uses a northern waypoint at (440, 230) in. The robot arrives at AUTO 13.49 s and waits 1.51 s before the three-second transition. The initial bumper crosses the red tape, and the proposed release bumper overlaps the red REEF ZONE by 0.291 m. [Native collision evidence](stack-negative.json)

The point arithmetic for the first row is three for LEAVE, two per TELEOP L1 CORAL and two for PARK. In the static-target replay the same two identified pipes sustain paired panel-and-lip support through TELEOP 138 s, including the assessment interval. No post-buzzer command is scheduled. The table assumes modeled custody and source stock, branch clearance and no subsequent disturbance of either pipe. The recorded result is a simulation; match scoring depends on the field and officials. [Scoring assessment](../scoring/README.md) · [Retention through assessment](observer-extent.json) · [Separate branch exclusion](../mission/branch-exclusion.json)

## What would change the result?

- **Field objects.** The official setup places another twelve ALGAE on the reefs. Their poses and deflection remain outside this trial. Agent FRC 0.17.0 checks 401 named bodies against the twelve individual floor pieces up to the first station lip event. The corrected starting approach passes that bounded check; the earlier turn fails with four positive material intersections. Removing the same two objects from both the route and the caller’s roster now produces a pending result. Whole-field clearance remains open. [0.17 results](plugin-review-017.json) · [0.16 counterexample](plugin-review.json)
- **Observed withdrawal.** Reopened red and blue exports have matching native geometry in all 401 bodies in each of four poses. The model carries the independent spring-cartridge excursions and rotating shaft bounds. At ±0.10 px each sector and shelf-friction case accepts all 17 zero-wait phases. The old deliberate 50 ms wait passes zero phases. At ±0.25 px, low drag accepts 17 of 17 phases per piece; high drag accepts 16 of 17. The wider whole-robot sweep reaches +119 ms after the shelf witness. Bench measurements of image error, synchronization and actual motor response remain necessary. [Kit comparison](kit-comparison.json) · [Assembly](red-assembly.json) · [Clock evidence](red-withdrawal-clock.json) · [Extended sweep](observer-extent.json)
- **Station supply.** The feed model assumes a lawful human handoff to a transverse pipe rolling into the receiver at 0.5 m/s, with 1.1 s insertion time. The [station-source experiment](../station-supply/README.md) reconstructs a centered introduction with a different 1.697 m/s lip speed and reproduces blocked misaligned approaches. A ready pipe and an open channel are independent requirements. Actual release distance, exit yaw and velocity, human access under G431/G433, and reliable sensing need a physical fixture and a transfer trial.
- **Drivetrain.** The model assumes a level field and adequate traction. The combined build target limits horizontal CG radius to 50 mm, requires the lighter dynamic side to carry at least 44% of the weight and limits the outer axles to 75%. Those targets give peak required friction 0.681; loose permissible mass placement raises it to 1.352. The assumed available value 0.90 requires a measured wheel/load qualification. Tight station clearance also needs validation of physical dimensions, tracking and driver repeatability.
- **Strategy.** An empty AUTO station approach, two L1 pipes and this partner’s parking-lane reservation constitute a candidate for students to evaluate. The route’s 41.75 s nominal margin does not resolve an occupied ending position. No robot has been operated, inspected or awarded match points in this experiment.

## Rule and geometry sources

Use the dated release-day register for a kickoff exercise. The competition trial also consults the later competition manual and the field drawing. In particular, check §§5.2, 5.7, 6.3–6.5, G303, G412–G413 and G431/G433 for the plan, and FE-2025 sheet 3 for start/field markings. The V0 L1 assessment and the later amended L1 predicate remain separately identified. [Kickoff register](../kickoff/rule-register-v0.json) · [FIRST game manual](https://firstfrc.blob.core.windows.net/frc2025/Manual/HTML/2025GameManual.htm) · [FIRST field drawings](https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf)

## Reproduce

Use the project's existing Python/OpenCascade environment and supply the installed plugin root where indicated. The STEP reconstruction authenticates all four project-owned blue inputs, applies line changes for the independently exported red assembly and verifies each original red SHA-256 before writing. The historical issued receipts stay available under their original paths. The issued field trial retains the Windows-decoded station wall separator. An independent UTF-8 rerun differs only in that printed separator; all numeric values and identities are identical. Source-bound consumers use the issued bytes.

```powershell
python full-match/materialize_red.py --output artifacts/red-401
python full-match/match.py --output artifacts/field-trial.json
python full-match/compare_kits.py --red artifacts/red-401 --blue design/competition-package/poses --output artifacts/kit-comparison.json
python full-match/stack_witness.py --poses artifacts/red-401 --output artifacts/stack-negative.json
python full-match/qualify.py --poses artifacts/red-401 --output artifacts/replayed-release
python full-match/observer_extent.py --poses artifacts/red-401 --output artifacts/observer-extent.json
python full-match/plugin_field_check.py --poses artifacts/red-401 --plugin $env:AGENT_FRC_PLUGIN_ROOT --working artifacts/field-route-check --output artifacts/plugin-review.json
python full-match/build_replay.py
```
