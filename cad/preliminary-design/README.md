# Preliminary design: Fixed pocket

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

This folder holds the preliminary design as data. The coding agent checks, stages and publishes it with its own design tool, Agent FRC; `PLUGIN` below stands for wherever the agent keeps that tool. The project's CI runs none of it and the repository holds no pin to it. The solids are built and measured, and the physics trials run, by the design library [`frc-design-kernel`](https://github.com/ehartye/frc-design-kernel), which has no pinned version: its main branch is the release.

| Version | Design | Status |
| --- | --- | --- |
| `releases/1.0/` | Fixed pocket on four-module swerve. A station CORAL drops into a fixed pocket, a two-roller nip (NEO 550, MAXPlanetary 36:1, powered hold) stops and holds it, and a deploy-once exit lip rolls it off into the L1 trough. | in progress |

One design has one version folder. An edit to an in-progress design is the next draft of the same version: edit the data, regenerate, check and stage `1.0` again. [Review page](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Handoff](releases/1.0/HANDOFF.md)

## Source, judgement and generated files

| Kind | Files | Who writes them |
| --- | --- | --- |
| Source | `design.json`: every solid, group, fixture, mount, contact, pose and motion, with sizes worked out from its `parameters`; the pipe, materials, motor, drive, adjustments and the trials. `trials/*.json`: one trial each. `parts/chassis-envelopes.step`: the twenty carried-forward chassis and module-housing envelopes. `field/*.step`: FIRST's official station and reef parts | The designer or the agent, by hand. This is where geometry and physics inputs change |
| Judgement | `judgement.json`: the stage of each motion with its allowance and basis, the cycle, the displaced control, premises, the narrative list and the review text. `budgets.json`, `package-limits.json`, `missions.json`, `premises/` | The designer or the agent, by hand. This is where reasons, allowances and budgets change |
| Generated | `generated/design.step`, `generated/fixtures.step`, `generated/manifest.json` from `build`; `generated/trials.json` from `try` | The design library, from `design.json`. Never edited |
| Checked | `receipt.json`, `package-motion.html` and its scene | The agent's layout check |
| Recorded | `evidence/field-trials.json`, `evidence/transport/`, `trial-replays.json`, `layout-blocks.json`, `evidence/interfaces/` | Draft 4's own trial solver and builder in [`iterations/002/design/preliminary-pocket/`](../../iterations/002/design/preliminary-pocket/), which no longer runs and is due to be removed. The checked receive and eject motions still follow these traces; the trials as data in `generated/trials.json` repeat them |

A finding from a trial or a later stage is fixed in `design.json`, never in a generated file. No program in this repository draws the robot or solves a trial.

The chassis envelopes are unions of the boxes of the 658 bodies of the iteration 002 chassis (`iterations/artifacts/resume-20261121/screws-place-01/four-seated-screws.step`), clipped to the frame outline and split 2.1 mm apart. That simplification was done once, outside the library, by draft 4's builder; the library places the result as a part and never changes it.

## Regenerate, check, try and publish

The agent runs these from the repository root with the geometry Python environment.

```sh
python -m frc_design_kernel build cad/preliminary-design/releases/1.0/design.json --out cad/preliminary-design/releases/1.0/generated
node PLUGIN/scripts/design.mjs preliminary-check . cad/preliminary-design/releases/1.0/judgement.json --write-view --output cad/preliminary-design/releases/1.0/receipt.json --workers 4
python -m frc_design_kernel try cad/preliminary-design/releases/1.0/design.json --out cad/preliminary-design/releases/1.0/generated --workers 4
python -m frc_design_kernel status cad/preliminary-design/releases/1.0/design.json cad/preliminary-design/releases/1.0/generated
python PLUGIN/scripts/stage_preliminary_review.py 1.0
python scripts/preliminary-still.py .
python scripts/cad-readmes.py . --update
node PLUGIN/scripts/design-publication.mjs check .
```

- Regenerate after every edit to `design.json`. The layout check stops when the generated solids are of an earlier source.
- The full check takes about two minutes on four workers. While changing one or two motions, add `--cases A,B`; while only geometry or motion changes, use `--physics-only` in place of `--write-view`. After an edit to names, colours, explanations or grouping only, use `--view-only`.
- `try` runs every trial with its control, half-step repeat, assumed ranges, brackets and adjustment coverage. `--trial NAME` runs one; `--estimate` prints the cost and stops. The 3D trial (`second trip`) needs the library's MuJoCo extra. `status` says which records a source edit made stale.
- Readiness to be marked effective: `node PLUGIN/scripts/design.mjs effective-readiness . --release cad/preliminary-design/releases/1.0 --design "Fixed pocket 1.0"`. The owner marks a design effective in a reviewed pull request.
- Review locally: `python -m http.server -d docs`, then open `http://127.0.0.1:8000/preliminary-design/`.
- After the status or draft number changes, edit the status line and caption of `docs/index.html` by hand. Nothing writes the entry page.

The review is one page with one motion list: the checked motions first, then the replays of draft 4's recorded trials. A replay is playback of recorded frames. The layout check neither runs nor credits it.
