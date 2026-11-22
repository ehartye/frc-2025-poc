# Red 2 B′x preliminary design review inputs

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 14649626c7](../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

This folder holds the inputs of each Red 2 B′x preliminary design release and the script that stages a release into the site. The repository is the project root of every layout check, so each pin is a path from the repository root.

| Release | Status | What it is |
| --- | --- | --- |
| [layout-002](releases/layout-002/) | in progress | Fixed pocket, two-roller nip on a NEO 550 and MAXPlanetary 36:1 with a powered hold, deploy-once exit lip on a sprung stop, four placed sensors. 50 checked cases. |
| [layout-001](releases/layout-001/) | superseded | The same pocket with a self-locking worm. Its build used scripts that were never committed, so it cannot be rebuilt; its inputs are kept here as issued. |

## Layout-002

- **Whole cycle.** The receive, carry and eject cases replay three solved free-piece trials: `receive-mission`, `custody-power-off-hit` and `eject-nominal` in [field-trials.json](releases/layout-002/evidence/field-trials.json). The lip deploy and the park drive are prescribed and say so.
- **Custody with the power off.** The tread squeeze alone holds the seated pipe. With the main power off, the softest accepted tread (40 N/mm) keeps the pipe through 6 g sustained and a 16 g, 50 ms hit; it loses it at 8 g and 24 g. A robot that is not driving sees at most a 1.5 g stop and an 8 g hit. [Retention review](releases/layout-002/retention-review.json)
- **Open risks.** Each is closed by a design change or carried as a bench test with pass and fail numbers: [bench gates G0 to G7](releases/layout-002/bench-gates.json), [design findings](releases/layout-002/design-findings.json). No bench article exists yet.
- **Fallback.** The guarded-pan baseline A, if bench gate G1, G3 or G7 fails inside the tread band.

The design is in progress until the owner marks it effective. Print the checklist with `node PLUGIN/scripts/design.mjs effective-readiness . --release cad/preliminary-design/releases/layout-002`.

## Rebuild, check and stage

The builders are in [`iterations/002/design/preliminary-pocket/`](../../iterations/002/design/preliminary-pocket/). `PLUGIN` is an Agent FRC checkout at the revision pinned as `AGENT_FRC_REF` in `.github/workflows/design-publication.yml`. Use the CAD Python (OCP installed) with node on the path.

- Rebuild everything, check it and write the review view: `python iterations/002/design/preliminary-pocket/build_release.py PLUGIN --view cad/preliminary-design/releases/layout-002/package-motion.html`. A rebuilt STEP carries a new timestamp, so every pin after it changes.
- Rewrite the records, receipt and view from the committed trials and STEP files: add `--records-only`.
- Stage the checked release into the site: `python cad/preliminary-design/stage.py layout-002`.
- Render the README still and refresh the READMEs: `python scripts/preliminary-still.py . layout-002 --update`, then `python scripts/cad-readmes.py . --template scripts/cad-orbit.html --update`.
- Check what the repository publishes, as CI does: `node PLUGIN/scripts/design-publication.mjs check .`.
- Review locally: `python cad/preliminary-design/serve.py`, then open `http://127.0.0.1:8000/`.

The field parts in the review (station chute, trough sector and the first scored pipe) are motion-study fixtures in `field-fixtures.step`. They stay in the clearance checks and carry no robot mass. The stable index's link to the main robot page (`../`) resolves on the deployed site, where `preliminary-design/` sits under the season showcase.
