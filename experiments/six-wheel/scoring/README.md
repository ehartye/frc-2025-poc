# Assess a CORAL observation against the applicable rules

<!-- cad-still -->
[![Team 1780 · stowed swerve robot](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · stowed swerve robot.** Red alliance R4 with numbered bumper covers. Purchased parts use project-created representations in this public assembly. [Source STEP · abbd8ab838](../../../cad/full-swerve-r4/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

Use this offline check to ask two questions about one robot and its alliance’s reef. How many distinct CORAL does this robot control at the observed instant? Does a particular identified piece meet the L1 conditions at that instant?

From the project root:

~~~sh
python scripts/assess_coral.py scoring/coral-observations.json --edition kickoff-v0
python scripts/assess_coral.py scoring/coral-observations.json --edition archive-compiled
python -m unittest discover -s tests -p test_coral_assessment.py
~~~

The input is a synthetic counterexample. At AUTO 15,000 ms, pipe-A touches its own trough and an opponent robot. Its L1 screen returns **no** under V0 and **yes** under the archived competition wording. At 15,100 ms the observations change: pipe-A qualifies for another level, making its later L1 screen **no**. Pipe-B is directly supported by pipe-A, which touches the trough. At 18,000 ms the old observation for pipe-A cannot establish either piece’s continuing state. At 18,001 ms the assessment window has elapsed. Both runs leave official earned points unknown.

| Observations required | Interpretation |
| --- | --- |
| Piece ID, observation timestamp and phase-relative evaluation time in integer milliseconds | Use a persistent physical identity. Two distinct CORAL can be controlled at once even if one switch is blocked. Repeated observations of one pipe do not create another delivery. Missing, future and stale timestamps have different consequences. |
| Fully supported by or stuck to the focal robot, or intentionally herded by that robot | These observations include indirect control through other objects. A deflection, touch or beam break alone cannot establish the G409 predicates. Record true, false, or null. To claim an upper bound, inventoryComplete must cover every potentially controlled CORAL. The ALGAE limit is separate. |
| A confirmed push of an already-scored L1 piece while attempting another score | Only the archived G409 screen applies this exception to herding. Unknown applicability keeps the count unresolved when it could change the result. Full support or a piece stuck to the robot remains separately observable. |
| Trough contact on the focal robot's own reef, direct supporting CORAL IDs, contacts with allied and opposing robots, and other-level scoring | A complete empty supporter list means no supporting CORAL. An omitted or null list means unknown. A support link needs contemporaneous trough-contact evidence for that identified supporter. For V0 either alliance's contact disqualifies. For the archive only allied contact disqualifies and a piece scored on another reef level is excluded. |
| Trace phase, command state, monotonically increasing time, observations at that same instant | AUTO and TELEOP use separate 15 s and 135 s active clocks. The model treats the buzzer as the end of powered commands and allows assessment through a maximum of 3 s afterward. An official observation may occur earlier. The trace does not interpolate across gaps. |

The scorer reports the G409 limit screen at the observed instants and raw per-piece L1 predicates. A qualified snapshot supplies evidence for a design trial. Human scoring and match adjustments, including later removal/replacement, require additional information. The project’s [first-contact receipt](../cad/station-receiver/contact-receipt.json) contains six CAD witnesses across separate trials. Its local contact time, geometry and negative controls supply no match-phase clock, known G409 control, subsequent retention or reef support. The script rejects that receipt as the wrong input type.

The [reviewed release register](../kickoff/rule-register-v0.json) includes ten modeled facts and seven provisions with explicitly unmodeled design consequences. Compile it with the installed Agent FRC plugin and the project-owned [season profile](../kickoff/season-profile.json), selecting a new output file. The edition label and each modeled value must agree. Verify the actual source document and its revision separately.

~~~sh
node <plugin-root>/scripts/rules.mjs review kickoff/rule-register-v0.json
node <plugin-root>/scripts/rules.mjs compile kickoff/rule-register-v0.json kickoff/season-profile.json reviewed-model.json
~~~

## Governing documents

- January 4 kickoff reconstruction, 160-page V0. §§6.4–6.5.1 describe phase/assessment timing and trough support. G409 on p. 64 describes possession including indirect control and incidental-contact examples. [V0 document mirror](https://manuals.plus/m/e98a748723b248f34e890accab1c49c809276fe44de713321834e30736d1e04a.pdf)
- Archived 164-page official English manual, §6 V13 and §7 V11, §§6.4–6.5.1 and G409. The later G409 includes a pushing exception. March 4 TU16 confirms the L1 wording change. Read the update cutoff for a specific event before choosing an oracle. [Official English PDF](https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf) · [Team Update 16](https://firstfrc.blob.core.windows.net/frc2025/Manual/TeamUpdates/TeamUpdate16.pdf) · [FIRST archive](https://www.firstinspires.org/resources/library/frc/archived-games)
