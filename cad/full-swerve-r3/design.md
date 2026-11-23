# Full fixed-height swerve robot, R3

This is the red, stowed configuration of the straight-guide design. It puts the four swerve modules, two-piece bumper, fixed mast pair, receiver, raised camera supports, electronics and battery into the same robot frame. The two stage rails are retracted. The shelf surface is 500 mm above the floor. X points forward, Y left and Z up. The two red covers follow the 2025 frame perimeter and keep the 2025 removable construction.

## Open the model

The R3 Fusion authoring document carries the four manufacturer module assemblies from chassis R2. The public STEP has been independently reopened. R3 was archived as an unnumbered control. The student handoff now uses the [independently reopened, numbered R4 model](../FUSION.md). Each carries a 76.2 mm wheel and its drive and steering motors. The center coordinates are X +/-282 mm and Y +/-277 mm. Students with the local project inputs can inspect purchased internals in Fusion. The public STEP contains the original frame, two physical covers, manufactured guide, shelf and slide geometry and simpler project-drawn representations of the four purchased modules, two camera housings and signal light. The appearance register identifies each representation by solid. [Project exchange](full-robot.step) / [Ordered solid register](appearances.json) / [Palette](../component-palette.json)

The registry binds the STEP bytes and records each solid's name, semantic color role, source, representation and six bounding coordinates in millimetres. The source verifier checks the solid order and coordinates after reopening with Open Cascade. The two thin registration marks are visual overlays. Envelopes record occupied space where the source engineering study represents a device or lead by its external dimensions. Disconnected stock, spare articles, service-only objects, field geometry and game pieces have no place in this stowed configuration. Component counts describe rendered material, including distinct welds and fasteners. Procurement quantities and measured mass have separate evidence.

## Where the colors come from

The 2025 original station and competition assemblies establish colors for structural stock, drivetrain, sensors, receiver and electrical equipment. Red is the one requested cross-season appearance. In the 2024 issued native robot display, node `fixed-bumper` uses material 5, whose linear glTF RGB encodes sRGB **#D71920**. R3 applies that red to the front and rear 2025 cloth covers. Their foam, backing, sleeves and fasteners retain distinct material roles. The student model preserves the appearances of the four imported commercial module occurrences. Colors describe function or appearance. The separately sourced material, alloy, load and mass properties remain with each engineering study. [2025 original palette](../../experiments/six-wheel/design/station-l1/mechanical.py) / [Competition palette](../../experiments/six-wheel/design/competition-package/build.py) / [2024 display source](https://github.com/ehartye/frc-2024-poc/blob/main/docs/media/robot-native.json)

Change a color in `cad/component-palette.json` and rebuild both editions to use the project setting. Removing an override restores the corresponding default role in `scripts/cad-color-convention.json` when one exists. The generator reads that file and writes the effective palette in ignored build output.

## Installed revisions

- The R2 drilled deck, cranked inner rails, corner saddles and raised host clear the four-module chassis in the source study. Its 40 relocated rail-ear articles and eight welds are placed 75 mm inboard at the four frame ends. [Chassis](../../experiments/swerve-drive/packaging/frame.json) / [Rear hardware](../../experiments/swerve-drive/packaging/rear-electronics.json)
- The selected straight arms retain the match-drilled mast and guide mount datums. The longitudinal receiver uses the 16-piece captured telescoping rail fixture and the machined two-layer shelf. The stowed guides guide contact with the incoming game piece. [Guide mounts](../../experiments/six-wheel/guide-mounts/fit.json) / [Guide setup](../../experiments/six-wheel/guide-setting/fit.json) / [Captured rails](../../experiments/six-wheel/extension-support/keepers.json) / [Machined shelf](../../experiments/six-wheel/shelf-structure/installed.json)
- The optical scene includes the fixed and moving head, the moving camera's physical shelf, SP1 install and release-observer targets. The project represents cables, sensors and the eight motor controllers at the fidelity in their source fixtures. Some cable sections and termination material have only a budget or a service path. Shop fits, provided supplier editions, flex clearance and electrical service remain local project checks. [Fixed camera](../../experiments/six-wheel/selected-optics/mount.json) / [SP1 installation](../../experiments/six-wheel/sp1-mount/fit.json) / [Controller trays](../../experiments/swerve-drive/power/controller-fit.json)

The public and student views share the design pose and project appearance assignments. This assembly documents geometric integration. Full dynamic clearance, structural certification, event inspection, a completed shopping bill and an as-built weigh-in each need their own evidence.

## Rebuild

Use the project Python environment with Open Cascade and the authorized local project inputs from the quoted R2 and fixed-camera studies. The following commands run from the repository root. Keep manufacturer files and the output that contains them in local ignored storage.

```sh
python cad/full-swerve-r3/cache_registered.py <project-artifacts> <local-signal-light.step> <project-artifacts>/full-swerve-fusion/registered
python cad/full-swerve-r3/build.py <project-artifacts>
```

The Fusion operation script supports `create`, `supplier`, `super`, `color`, `export`, `reopen` and `inspect`. Supply a local manifest giving the operation as `stage`, the local output `directory`, the `r2` archive and `r2Sha256`, the resolved `palette` at `<project-artifacts>/full-swerve-fusion/effective-palette.json`, the ordered `appearances` JSON paths, and the configuration ID from the public register. Run each stage once against its named project document. Its checks include the four native tread centers, each original colored solid, both bumper extents and colors, the mast and shelf registration and the public supplier substitutions. [Fusion operations](fusion.py)
