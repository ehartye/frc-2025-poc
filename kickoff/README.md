# Release-day opportunity study

<!-- cad-still -->
[![Four-module swerve frame and mounting hardware](../docs/robot/cad/assembly.png)](../docs/robot/cad/index.html)

**Project CAD · Four-module swerve frame and mounting hardware.** Original robot structure for the swerve candidate. The project-local Fusion assembly adds four manufacturer module occurrences. [Source STEP · f8b75f2df4](../experiments/swerve-drive/packaging/swerve-frame.step) · [Rotate the model](../docs/robot/cad/index.html)
<!-- /cad-still -->

Download and open the [standalone REEFSCAPE field lab](field-lab.html) in a browser. It includes the rules model, team rosters, field obstacles, scoring-window calculator and match clocks. The [editable season source](season-profile.json) is the input for regeneration.

The editable observation box starts at 30 × 30 × 30 in and heading 0°. Each rehearsal roster slot initially uses a 26 × 26 in frame with 3.25 in of bumper per side, for a 32.5 × 32.5 in occupied footprint. Use the trial box on a selected roster slot to compare the two configurations.

This exercise starts from the 2025 REEFSCAPE V0 game manual. The existing mechanism fixture in the main README is a later investigation. Both candidate match stories in this exercise remain open for team review.

1. Inspect the [editable V0 rules model](rules-v0.json). It records the inventory, phase durations, point values, field dimensions and illustrative route assumptions.
2. Open the student field lab from the [project-owned season profile](season-profile.json) using the command below. This file supplies season geometry, scoring opportunities, team start slots, proposed routes, obstacle and cage assumptions, period names and limits, and match programs to the portable engine. Compare the [two saved field observations and scoring windows](field-lab-draft.md). An analyst entered these trial values as prompts for student discussion.
3. Replay the one-piece transport schedule with `python scripts/kickoff_shuttle.py` from the repository root. Its only inputs are the saved model and [trial assumptions](trials.json). No CAD software, installed plugin or network is needed for this calculation.

## Which role can use this 15-second window?

The [field occupancy follow-up](FIELD-SPACE.md) checks the full footprint at the station, flags an intersecting centerline, and compares a reroute and two-robot choke schedule.

Each trial starts with the robot at its pickup pose, has one second of setup and assumes mean loaded and empty transit speeds of 60 in/s. All allocated pieces are at their source when the window opens. Straight sections and bends follow the saved route. The trials assume immediate access and successful scoring. Queues, acceleration transients, alignment variation and capture failures need measurements.

| Candidate | Allocated to this robot | Route A → B | Assumed pickup + release | By 15 s |
| --- | --- | --- | --- | --- |
| OP-1. Receive CORAL at the station, place in L1, return to receive again | 4 distinct CORAL at the same station | 139.647 in | 1.0 + 0.5 s | First two complete at 4.827 and 10.982 s, for up to 4 points |
| OP-2. Acquire one accessible reef ALGAE and deliver it to the processor | 1 ALGAE at the reef | 97.952 in | 3.0 + 1.0 s | Delivery completes at 6.633 s, for up to 6 points |

The two alternatives consume the same window. A larger match strategy must allocate robots, supply, time and scoring sites before crediting either result. The six-point processor case assumes a successful removal, transport and score. Feasibility and reliability of removal remain open.

## Turn the scoring window into a technical requirement

The field lab's generic rate screen charges the A → B transit once. With the OP-1 service input restricted to pickup and release, it reports a four-piece ceiling. The campaign needs a separate shuttle schedule because each additional CORAL must come from the station. V0 G409 constrains simultaneous control. In this candidate the robot transports one CORAL on each loaded trip. [2025 V0 game manual, §§6.3–6.5 and G409](https://manuals.plus/m/e98a748723b248f34e890accab1c49c809276fe44de713321834e30736d1e04a.pdf)

For delivery number `n`, the elapsed time is `setup + n × (pickup + release) + (2n − 1) × one_way_travel`. The last delivery requires no return. The third OP-1 delivery finishes at 17.137 s. At 60 in/s, receiving and releasing together must take at most 0.788 s per piece to deliver three within the window. Keeping the 1.5 s service assumption instead requires at least 73.50 in/s mean transit. Meeting either idealized boundary leaves zero scheduling margin. This is why the team should time the parts of a complete cycle before choosing a loader.

Both routes pass within the model's corridor threshold of paths associated with two partner start slots. The estimate uses illustrative starting and goal approaches and has no arrival-time evidence. For OP-1, record exposure of the deployed CORAL guides during pickup and their configuration in transit. For OP-2, record ALGAE retention and the capture assembly's projection while traveling to the processor. Keep both roles as candidates until the team chooses.

**Retained-function experiment.** Establish the expected contact location, direction, load range, duration and mount load path from appropriate evidence. Record the piece and mechanism states. Once a representative contact case is defined and reviewed, compare baseline and post-contact retention, commanded release, and next acquisition under the same conditions. For the three-CORAL candidate, a successful capture plus release must satisfy the 0.788 s service requirement at the assumed transit speed. Record drops, jams, deformation, measurement uncertainty and any missed deadlines. No measured load or retained-function result exists yet.

## Reproduce the field exercise

The saved rules model and browser draft came from Agent FRC 0.12.0. They preserve the baseline for the independent calculations above. To generate a page with the new timed rehearsal, run the portable plugin CLI from the POC root. The same season JSON is also the input to the occupancy, clock, queue and swept-footprint regression suite.

```sh
node /path/to/agent-frc/scripts/kickoff.mjs --profile kickoff/season-profile.json --output field.html
```

Set `AGENT_FRC_PLUGIN_ROOT` to the compatible plugin directory and run `node --test tests/kickoff-field.test.mjs` for the season integration checks. In the lab, compare the default 30 × 30 × 30 in box with a 31 × 29 × 30 in box, then return to the default. Use TELEOP and the two targets in the table. Enter the trial window, stock allocations, service and speed from the project JSON. Leave incoming supply at zero. Capture both opportunities and export the draft. Keep trial geometry, rule facts and calculated feasibility distinct from actual field performance. [Original 0.12.0 source revision](https://github.com/ehartye/agent-frc/tree/1ff7845112276ad50bf4ee8a782c3dfaedca723d)
