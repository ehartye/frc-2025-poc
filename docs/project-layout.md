# Project structure

Start a new study under `experiments/<study>/`. Use `kickoff/` for the current game model and narrative, `cad/` for student Fusion work, `controls/` for promoted controls, `sim/` for reusable simulations, `scripts/` and `tests/` for entry points, and `docs/robot/` for published evidence. Create these folders when they have content. Write local run products to ignored `artifacts/`.

The [six-wheel campaign](../experiments/six-wheel/README.md) preserves the earlier project's relative paths because its tests, viewers and evidence depend on them. Run commands in its documentation from `experiments/six-wheel/`. Its kickoff inputs are a historical snapshot. The active student copy is [kickoff/](../kickoff/README.md). Work on the four-module candidate belongs in `experiments/swerve-drive/`.

From the checkout root, run `node scripts/project-layout.mjs check .` and `python scripts/cad-readmes.py . --template scripts/cad-orbit.html`. To refresh changed source geometry, render with `python scripts/export-showcase-cad.py .` (Python, OCP and VTK), then run the README check with `--update`. Update the source, title and scope in `cad-showcase.json` when the selected assembly changes. Open the standalone interactive model in a browser or view the linked PNG directly.

Use `python -m unittest discover -s tests` for Python regression discovery, and the original `run.ps1 -JavaHome … -GradleHome …` for the historical simulator.
