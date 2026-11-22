# Project structure

Start a new study under `experiments/<study>/`. Use `kickoff/` for the current game model and narrative, `cad/` for student Fusion work, `controls/` for promoted controls, `sim/` for reusable simulations, `scripts/` and `tests/` for entry points, and `docs/robot/` for published evidence. Create these folders when they have content. Write local run products to ignored `artifacts/`.

The [six-wheel campaign](../experiments/six-wheel/README.md) preserves the earlier project's relative paths because its tests, viewers and evidence depend on them. Run commands in its documentation from `experiments/six-wheel/`. Its kickoff inputs are a historical snapshot. The active student copy is [kickoff/](../kickoff/README.md). Work on the four-module candidate belongs in `experiments/swerve-drive/`.

From the checkout root, run `node scripts/project-layout.mjs check .` and `python scripts/cad-readmes.py . --template scripts/cad-orbit.html`. To refresh changed source geometry, render with `python scripts/export-showcase-cad.py .` (Python, OCP and VTK), then run the README check with `--update`. Update the source, title and scope in `cad-showcase.json` when the selected assembly changes. For colored models, set `appearances` to the solid register for that exact STEP. The exporter and viewer resolve each role against `scripts/cad-color-convention.json` and the overrides in `cad/component-palette.json`. A geometry or palette change requires a fresh export. Open `docs/robot/cad/index.html` locally to rotate and zoom the offline model, or follow the live showcase link from a README.

Use `python -m unittest discover -s tests` for Python regression discovery, and the original `run.ps1 -JavaHome Ã¢â‚¬Â¦ -GradleHome Ã¢â‚¬Â¦` for the historical simulator.


## Project evidence and release checks

Use Python 3.13 and Node 22. Install the regression dependencies from `tests/requirements-ci.txt` into the active project environment. Set `AGENT_FRC_PLUGIN_ROOT` to the reviewed Agent FRC installation selected for the repository. Set `AGENT_FRC_HISTORICAL_FIELD_ROOT` to the original `ba9241e724d2425fcf007a9d125e8f87186d68bc` runtime for the two archived field studies. From the project root, run `node PATH_TO_PLUGIN/scripts/project-checks.mjs . full` for every registered check, or `node PATH_TO_PLUGIN/scripts/project-checks.mjs . changed BASE_COMMIT` for a checked ancestor. The same commands run on pull requests and main. A manual run performs the full verification. Publication follows a verified push to main.

The 2025 site is built, not served from `docs/`. The **Site build** job of `.github/workflows/design-publication.yml` runs the season's original field builder at `92ddf41933dd6026b26493c1d2c91505e9d15914`, then `scripts/publish-capture.mjs`, then copies `docs/preliminary-design/` into the output.

## Design status on the site and READMEs

The rough-draft showcase model, `draft-001`, is withdrawn and no detailed design is current. `docs/detailed-design/designs.json` holds the status and the reason. The shared builder still emits that model's files, so `scripts/publish-capture.mjs` drops the outputs listed in `docs/robot/site-build.json`, publishes `docs/detailed-design/` and fills `docs/robot/entry.html` for the current preliminary design release. The withdrawn pages are not archived on the site; Git keeps them at commit `33a2315`. The earlier build script is kept at `scripts/withdrawn/draft-001/publish-capture.mjs`.

Each README features the current preliminary design release. Its still is rendered from the release's published review scene, so it needs no STEP: author the title, pose and scope sentences under `caption` in `docs/robot/preliminary-RELEASE.json`, then run `python scripts/preliminary-still.py . RELEASE --update` (Python, OCP and VTK) and `python scripts/cad-readmes.py . --template scripts/cad-orbit.html --update`. The README caption names the release and its design status, so a new current release or a status change takes the same two commands in the same pull request. `cad-readmes.py` keeps the detailed still once the detailed design index shows an in-progress or effective design, and `publish-capture.mjs` stops with an error then; staging the first detailed design includes giving each its new primary view.

One workflow, **Design publication**, checks every push: it scans the branch for machine paths and credentials, reruns the layout check of each in-progress or effective design from its committed inputs, runs the registered checks in `tests/ci.json` and builds the site. Run the same check before pushing: `node PATH_TO_PLUGIN/scripts/design-publication.mjs check .`. A release is published, superseded or marked effective by one reviewed pull request.
