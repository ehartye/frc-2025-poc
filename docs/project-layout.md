# Project structure

Start a new study under `experiments/<study>/`. Use `kickoff/` for the current game model and narrative, `cad/` for student Fusion work, `controls/` for promoted controls, `sim/` for reusable simulations, `scripts/` and `tests/` for entry points, and `docs/robot/` for published evidence. Create these folders when they have content. Write local run products to ignored `artifacts/`.

The [six-wheel campaign](../experiments/six-wheel/README.md) preserves the earlier project's relative paths because its tests, viewers and evidence depend on them. Run commands in its documentation from `experiments/six-wheel/`. Its kickoff inputs are a historical snapshot. The active student copy is [kickoff/](../kickoff/README.md). Work on the four-module candidate belongs in `experiments/swerve-drive/`.

From the checkout root, run `node scripts/project-layout.mjs check .` and `python scripts/cad-readmes.py .`. Open `docs/robot/cad/index.html` locally to rotate and zoom the offline model of the withdrawn draft.

Use `python -m unittest discover -s tests` for Python regression discovery (Python 3.14 with the packages in `tests/requirements-ci.txt`), and the original `run.ps1 -JavaHome … -GradleHome …` for the historical simulator.

## The project's CI

One workflow, **Project CI** (`.github/workflows/ci.yml`), runs on every push to every branch. It checks out this repository and nothing else, and runs only files that live here.

| Step | Command | Fails when |
| --- | --- | --- |
| Scan for personal paths and secrets | `node scripts/ci/scan.mjs .` | A tracked file holds a personal machine path or a credential |
| Check the published site | `node scripts/ci/site-check.mjs docs` | A design index and its folders disagree, a named file is missing, or a relative link in a page leads nowhere |
| Check the README stills | `python scripts/cad-readmes.py .` | A project README lacks the still block below its title, the image, or the design name and status |

The job is named **Scan and site check**. On `main` a second job, **Publish the site**, deploys `docs/` to GitHub Pages as committed. Nothing is built: the site is exactly the `docs/` folder. Run the three commands above before pushing.

CI runs no design check. A coding agent checks a design with its own design tool before it pushes; [AGENTS.md](../AGENTS.md) says what it runs for which change. No file in this repository depends on that tool being installed, and the earlier scripts that called it were removed. Git history keeps them.

## Design status on the site and READMEs

The current design is the preliminary design release Fixed pocket 1.0, in progress. `docs/preliminary-design/releases.json` holds its status. No detailed design is published, so `docs/detailed-design/designs.json` lists none. The rough-draft showcase model published before the design-status process is withdrawn and is not on the site: its ID was not a design version, so the index no longer lists it, and Git keeps its pages at commit `33a2315`. `docs/index.html` is the site's front page and is edited by hand.

Each README features the current preliminary design release. Its still is rendered from the release's published review scene, so it needs no STEP: author the title, pose and scope sentences under `caption` in `docs/robot/preliminary-RELEASE.json`, then run `python scripts/preliminary-still.py . RELEASE --update` (Python, OCP and VTK) and `python scripts/cad-readmes.py . --update`. The README caption names the release and its design status, so a new current release or a status change takes the same two commands in the same pull request. `cad-readmes.py` keeps the detailed still once the detailed design index shows an in-progress or effective design.

A release is published, superseded or marked effective by one reviewed pull request.
