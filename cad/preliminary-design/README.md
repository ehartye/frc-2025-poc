# Red 2 B′x preliminary design review inputs

<!-- cad-still -->
[![Preliminary design · in progress · layout-001](../../docs/robot/preliminary-layout-001.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-001.** Red 2 B′x fixed pocket, roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 64 representative solids, with the field fixture, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 1cc8a76119](../../docs/preliminary-design/releases/layout-001/receipt.json)
<!-- /cad-still -->

This folder follows the Agent FRC 0.100.0 whole-robot review layout template. `releases/layout-001/` holds the public inputs for the B′x fixed pocket, roller nip and exit lip. B′x is selected for preliminary review only; the guarded-pan baseline A remains the fallback pending bench gates G1–G4 (their record is withheld with the private package).

**Custody.** The complete package, including `package.step`, lives in the Red 2 private assets repository. It was re-rooted to this release layout and checked there with Agent FRC 0.100.0 `preliminary-check --write-view --require-checked` (status `layout-checked`).

Only inputs that pass the rights gate are published here. [custody-layout-001.json](custody-layout-001.json) lists each withheld file with its SHA-256 and the reason it is withheld: the layout STEP, private evidence, and records that name private CAD. The same file records the block-level geometry check. Every published solid is a box, a box union, a revolved envelope or a project-drawn block, and none is supplier tessellation.

- Stage a release: `python cad/preliminary-design/stage.py layout-001`. Staging needs only the checked view, paths, budgets and limits, and all of them are public.
- Audit the index: `python cad/preliminary-design/stage.py --check-index`.
- Review locally: `python cad/preliminary-design/serve.py`, then open `http://127.0.0.1:8000/`.

The receive, eject and retention trials play in the [field-interaction player](releases/layout-001/field/bx-field-trials.html), which ships inside the release.

The interacting field parts are motion-study fixtures (`field-fixtures.step`, withheld in private custody with the package STEP and registered under each case's `fixtures` with their source, dimensions and placement). They stay in the clearance checks but carry no robot mass, and **Fit field interaction** frames them.

The stable index's "main robot page" link (`../`) is part of the plugin's gated viewer. It resolves on the deployed GitHub Pages site, where `preliminary-design/` sits under the season showcase. A local server rooted at `docs/` has no season index there, so the link returns to this review.
