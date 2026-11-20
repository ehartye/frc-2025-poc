# The 2025 robot design workflow

Start with the rulebook and a robot-sized box. Follow one proposed scoring cycle all the way to a physical pipe, an observed stop, and an alternative role the robot can complete. Every document below lives in this repository and opens from an offline checkout.

| Student question | Agent FRC skill | Project document | Evidence to inspect |
|---|---|---|---|
| What did teams know on kickoff day? | `frc-season`, `frc-case-study` | [Release-day rules and assumptions](../kickoff/README.md) | [V0 rule register](../kickoff/rule-register-v0.json) and [geometry constraints](../kickoff/field-constraints.json) |
| Where do pieces, robots and routes compete for space? | `frc-previsualization` | [Field-space study](../kickoff/FIELD-SPACE.md) | [Interactive field lab](../kickoff/field-lab.html) and [trial observations](../kickoff/trials.json) |
| Which opportunities justify a mechanism? | `frc-strategic-narrative` | [Kickoff alternatives](../kickoff/field-lab-draft.md) and [selected competition role](../kickoff/selected-role.md) | [Nine-point acceptance conditions](../experiments/swerve-drive/motion/match-qualification.md) |
| Can the drivetrain reach the station? | `frc-swerve` | [Four-module packaging](../experiments/swerve-drive/packaging/README.md) and [power limits](../experiments/swerve-drive/power/README.md) | [Module centers and gearing](../experiments/swerve-drive/packaging/module.json), [reciprocal-drive evidence](../experiments/swerve-drive/motion/energy.json), [starting-line assessment](../experiments/swerve-drive/motion/opening-line.json) |
| How do we edit and inspect the robot? | `frc-mechanism-cad` | [Download the student Fusion robot](../cad/swerve-captive/Team-1780-public-captive-swerve.f3d), [joint guide](../cad/swerve-captive/study.md) and [numbered assembly design](../cad/full-swerve-r4/design.md) | [Student archive reopen](../cad/swerve-captive/student-reopen.json), [earlier supplier-detail full-robot study](../cad/swerve-articulation/study.md), [joint and archive reopen record](../cad/swerve-articulation/native.json), [static numbered R4 public STEP](../cad/full-swerve-r4/full-robot.step), [offline orbit](robot/cad/index.html) |
| Did the receiver acquire and retain a pipe? | `frc-vision`, `frc-troubleshoot` | [Material, grip and observation study](../experiments/swerve-drive/receiver-material/study.md) and [moving custody decision](../experiments/swerve-drive/transit/README.md) | [Pickup samples](../experiments/swerve-drive/receiver-material/results.json) and [two measured stops](../experiments/swerve-drive/transit/stops.json) and [four-pin retainer study](../experiments/swerve-drive/retention/study.md) |
| Can it release at the intended reef sectors? | `frc-mechanism-cad` | [Receiver stroke and trough study](../experiments/swerve-drive/score-stroke/study.md) | [Editable cam lands](../experiments/swerve-drive/score-stroke/cam-lands.f3d), [station and rack traces](../experiments/swerve-drive/score-stroke/candidate.json), [landing outcomes](../experiments/swerve-drive/score-stroke/landing.json) |
| Which points survive a complete match clock? | `frc-modes`, `frc-java` | [Primary scoring conditions](../experiments/swerve-drive/motion/match-qualification.md) and [empty fallback](../experiments/swerve-drive/motion/fallback.md) | [Complete enabled, disabled and assessment periods](../experiments/swerve-drive/motion/fallback-match.json) |
| How can another student reproduce and explain the results? | `frc-project-setup`, `frc-robot-highlights` | [Project layout](project-layout.md) and [root navigation](../README.md) | [Showcase manifest](../highlights.json) and [pickup and release film](../experiments/swerve-drive/score-stroke/receiver-motion.webm) |

The selected robot uses four swerve modules, a fixed-height sliding receiver, straight guide arms and the chassis gyro. The older [six-wheel campaign](../experiments/six-wheel/README.md) preserves its own parts and match histories. A result transfers to swerve only where the current project record identifies the shared source and checks the new configuration.

The candidate bore pins and rim probes in the [capture study](../experiments/swerve-drive/retention/study.md) have a [loaded endpoint and dock-observation follow-up](../experiments/swerve-drive/retention/observer-study.md). The [interactive capture view](robot/cad/index.html) and downloadable Fusion robot use the same named assembly, including all 26 capture parts. The older R4 STEP and the three-fixture film remain available.

The [observed reef correction](../experiments/swerve-drive/reef-correction/study.md) records the measured cargo and wheel stops, camera calibration and cam-shelf source change for the two development inputs.

The [matched cam-and-capture carry](../experiments/swerve-drive/cam-scoring/study.md) starts with the machined station tray’s own modal and pipe states. Its [two signed runs](../experiments/swerve-drive/cam-scoring/results.json) bind field travel, custody, camera holds and zero scored deliveries.

### BARGE endgame

The season skill anchors cage and park points to release V0. The mechanism-CAD skill produced the [editable forward jack, contacts and load-path study](../cad/shallow-cage/study.md). The match rehearsal compares the [three historical games and three BARGE variants](../kickoff/series.md) on the [registered assessment clock](../kickoff/barge-assessment.json).

The `frc-mechanism-cad` receiver service issue ties each of 1,268 installed solids to the [materials and procurement register](../cad/assembly-engineering/receiver-service/verification.md). Its [shop sequence](../cad/assembly-engineering/receiver-service/shop.md) uses the actual grip-driver, switch, battery and camera-guide access studies. [Editable engineering assembly and pose controls](../cad/FUSION.md)

The earlier `frc-mechanism-cad` engineering pass follows the original default-material discovery through to a [source-bound whole-assembly mass register and assembly traveler](../cad/assembly-engineering/issue/drawing-traveler.md). [Compare the cold native archive](../cad/assembly-engineering/issue/cold-reopen.json) with the [signed assembly issue](../cad/assembly-engineering/issue/interfaces.json), [shop upper estimate](../cad/assembly-engineering/issue/mass.json) and [comparison of alternative field reactions](../cad/assembly-engineering/shop.md).

## Compare station routes and size the complete robot

The strategic narrative and case-study workflow compare the same five other players, change Red 1’s station, and test depletion of that station. The mechanism-CAD working-load study uses the complete front and rear jack layouts, operating robot mass, simultaneous roll and pitch reactions, and the actual stopped receiver traces. [Rehearse the selected role](https://ehartye.github.io/frc-2025-poc/role-lab.html#roleLab) · [Study, inputs and measured outcomes](../experiments/swerve-drive/whole-robot-trade/study.md)

The mechanism-CAD workflow compares four true rear frame setbacks against both purchased rear modules and their removal paths. [Frame and bumper study](../cad/rear-frame-facets/study.md) · [Signed fit results](../cad/rear-frame-facets/fit.json)

The mechanism and vision studies follow Red 2’s captured CORAL from the reef stop through a measured correction, stroke selection and finite-cage clearance. [Low placement and held-out pickup](../experiments/swerve-drive/low-placement/study.md)
