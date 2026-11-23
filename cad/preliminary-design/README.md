# Preliminary design: Fixed pocket

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

This folder holds the inputs of the robot's preliminary design and the script that publishes it to the site. One design, one folder, one review page: [Fixed pocket 1.0](releases/1.0/), in progress. An edit republishes the same version as its next draft.

| | |
| --- | --- |
| Architecture | Fixed pocket on four-module swerve. A station CORAL drops into a fixed pocket, a two-roller nip (NEO 550, MAXPlanetary 36:1, powered hold) stops and holds it, and a deploy-once exit lip rolls it off into the L1 trough. |
| Review page | <https://ehartye.github.io/frc-2025-poc/preliminary-design/> |
| Inputs | [`releases/1.0/`](releases/1.0/): `package.step`, `layout.json`, `paths.json`, `budgets.json`, `missions.json`, the trial records under `evidence/` and the saved `receipt.json` |
| Builders | [`iterations/002/design/preliminary-pocket/`](../../iterations/002/design/preliminary-pocket/) |
| Field | Official FIRST field parts under [`cad/field/official/`](../field/official/) |

The design is in progress until the owner marks it effective. Print the readiness report with `node PLUGIN/scripts/design.mjs effective-readiness . --release cad/preliminary-design/releases/1.0 --design "Fixed pocket 1.0"`.

## Rebuild, check and publish

`PLUGIN` is a copy of Agent FRC at the revision `.github/workflows/design-publication.yml` names as `AGENT_FRC_REF`. Use an installed copy or an archive of that commit, never a checkout another session can move. Use the CAD Python (OCP installed) with node on the path. Run every command from the repository root.

| Step | Command |
| --- | --- |
| Rebuild everything, check it and write the review page | `python iterations/002/design/preliminary-pocket/build_release.py PLUGIN` |
| Keep the recorded trials, rebuild the layout and check it | add `--layout-only` |
| Keep the trials and STEP files, rewrite the records and check | add `--records-only` |
| Rebuild the page after a presentation edit, no case rerun | add `--records-only --view-only` |
| Publish the next draft | `python cad/preliminary-design/stage.py 1.0` |
| Check what the repository publishes, as CI does | `node PLUGIN/scripts/design-publication.mjs check . --skip-rerun` |
| Render the README still and refresh every README | `python scripts/preliminary-still.py .` then `python scripts/cad-readmes.py . --update` |
| Review locally | `python cad/preliminary-design/serve.py`, then open `http://127.0.0.1:8000/` |

The builder caps the check and the trials at four workers; pass `--jobs N` or `--processes N` for another number.
