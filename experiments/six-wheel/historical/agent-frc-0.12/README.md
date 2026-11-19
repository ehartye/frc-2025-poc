# Historical 2025 source package

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

This preserves the 2025 operating inputs and regression evidence that accompanied the earlier Agent FRC repository. The source predates the project-owned field rehearsal. Its retrospective case uses a 164-page manual snapshot with sections 6 and 7 at different post-kickoff revisions. Use the [kickoff season profile](../../kickoff/season-profile.json) for the release-day exercise.

[The inventory](inventory.json) records the original path, SHA-256, size and disposition for the 2025 case directory, the season Java properties and the other explicitly CORAL/REEFSCAPE/ALGAE-named files in the source tree. Local copies preserve native bytes. Files already owned by this project point to their verified location. Named historical Java, Python, CAD, numerical experiment and fixture inputs are operational records. The profile under `operational/case-studies/2025/kickoff-field.json` is frozen for earlier runs.

The source epoch also contains research narratives, rule and hardware reviews, retrospective strategy candidates and analytical reports. Their findings and provenance belong in the FRC research wiki. This public inventory records their immutable path and hash. The source commit can reconstruct those dependencies on demand for historical verification. Original third-party license and source-asset terms still apply. [Original release tree](https://github.com/ehartye/agent-frc/tree/f6ab64ace84f8aca7168375364a9fd36b4eed224)

## Verify or replay

From this POC root, verify the project files without network access.

```sh
python scripts/historical_season_package.py
```

To replay the original runners, materialize the pinned source tree into a new ignored directory. Supply an existing Git checkout of the original repository, or explicitly use `--fetch` to retrieve the recorded commit from its public origin. The command checks tree identity and every indexed file before promoting the output. It preserves existing output directories.

```sh
python scripts/historical_season_package.py --source /path/to/agent-frc
cd artifacts/historical-agent-frc-f6ab64a
node scripts/case-study.mjs --season 2025
node scripts/strategy-brief.mjs --season 2025
node scripts/rules-audit.mjs --season 2025 --as-of 2026-11-18 --event-stage archive-study
python -m unittest scripts/test-coral-contact-runner.py
```

The three Node entry points and all four CORAL runner tests above executed from the reconstructed tree. The original placement report has a separate execution-freshness boundary. At this source pin, running `python scripts/test-coral-placement.py` stops during setup with `ValueError: stale execution` in its historical integrated-target producer. Its report retains the original input pins. Restoring file bytes does not establish fresh placement verification. Generating a retrospective strategy remains separate from choosing this team's role.
