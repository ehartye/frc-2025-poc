# Preliminary design: Fixed pocket

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

This folder holds the inputs of the robot's preliminary design. One design, one folder, one review page: [Fixed pocket 1.0](releases/1.0/), in progress. An edit republishes the same version as its next draft.

| | |
| --- | --- |
| Architecture | Fixed pocket on four-module swerve. A station CORAL drops into a fixed pocket, a two-roller nip (NEO 550, MAXPlanetary 36:1, powered hold) stops and holds it, and a deploy-once exit lip rolls it off into the L1 trough. |
| Review page | <https://ehartye.github.io/frc-2025-poc/preliminary-design/> |
| Inputs | [`releases/1.0/`](releases/1.0/): `package.step`, `layout.json`, `paths.json`, `budgets.json`, `missions.json`, the trial records under `evidence/` and the saved `receipt.json` |
| Handoff | [`releases/1.0/HANDOFF.md`](releases/1.0/HANDOFF.md): datums, envelopes, interfaces, operating windows, budgets, bench tests and first work items for detailed design |
| Builders | [`iterations/002/design/preliminary-pocket/`](../../iterations/002/design/preliminary-pocket/) |
| Field | FIRST's official station and reef parts, placed at the two docks: [`releases/1.0/field/`](releases/1.0/field/), from [`cad/field/official/`](../field/official/) |

The design is in progress until the owner marks it effective.

## Rebuild, check and publish

A coding agent rebuilds, checks and publishes this design with its own design tool; [AGENTS.md](../../AGENTS.md) says what it runs for which change. The project's CI runs none of it: a push scans the branch, checks the site and, on `main`, publishes `docs/` as committed. [The project's CI](../../docs/project-layout.md#the-projects-ci)

The builder under [`iterations/002/design/preliminary-pocket/`](../../iterations/002/design/preliminary-pocket/) is the one remaining set of scripts here that calls that tool. It takes the tool's folder as an argument, is run only by the agent and is due to be replaced by a separate library. It needs the CAD Python (OCP installed) and Node.

| Step | Who | Command |
| --- | --- | --- |
| Rebuild the layout, trials and records, check them and write the review page | Agent | `build_release.py` in the builder folder, with `--layout-only`, `--records-only` or `--records-only --view-only` for a narrower rebuild |
| Publish the next draft, check what the repository publishes, serve the review locally | Agent | The design tool's staging, publication check and review server |
| Render the README still and refresh every README | Anyone | `python scripts/preliminary-still.py .` then `python scripts/cad-readmes.py . --update` |
| Read the published review from a checkout | Anyone | `python -m http.server -d docs`, then open `http://127.0.0.1:8000/preliminary-design/` |
