# 2025 REEFSCAPE proof of concept

Explore the 545-body [blue](design/competition-package/shop-duplex/blue/robot.html) or [red robot](design/competition-package/shop-duplex/red/robot.html) in 3D. Select a carriage pose or isolate a part to follow the pipe from station to trough. The [engineering record](design/competition-package/shop-duplex/README.md) includes the fastened chassis and duplex lift, full STEP files, normal travel, weight bounds and open docking and arrest limits.

[Run the leave-and-park fallback](leave-park/replay.html) with a stowed receiver and empty station. Compare a reserved lane, a blocked approach and a late endgame decision. [Tracking limits and match points](leave-park/README.md)

[Open the complete 2025 match replay](full-match/replay.html). Scrub from AUTO through the end of TELEOP, compare the earlier starting-stack collision, and try a blocked park or a return to station. The [full-match evidence](full-match/README.md) explains how the two pipe identities, contact and clock were qualified.

[Follow both station-fed pipes through the updated 545-body match](sourced-match/replay.html). The new timeline preserves their feeder states and lets students compare axial spin, a late feed and a blocked park. [Conditional scoring evidence](sourced-match/README.md)

[Follow the rolling pipe from the station to the reef](axial-spin/README.md) to see how release-belt speed and pipe spin change the outcome. [Compare the WPILib controller’s actual 20 ms belt commands](coupled-belt-clock/README.md) with the modeled acquisition and placement. [Trace the two-port acquisition controller](observable-acquisition/README.md) from chute entry to its seated-position signal, then [check the independent stereo observation](stereo-custody/README.md).

[Compare the installed articles and inspection weight estimates](design/assembly-accounting/README.md) for both alliance configurations and the proposed carrier and stop hardware.

## Design highlights

[Compare two fixed-carriage configurations](design/fixed-height/README.md) and review how receiver height, counterfeed and slide extension change the reachable station and L1 outcomes. [Follow the camera evidence through both low-carriage placements](low-observation/README.md) to see where the controller can identify a pipe and which withdrawal timings it can test.

Explore the earlier [moving six-robot field rehearsal](mission/moving-field.html) and its [traffic, station stock and reef-branch evidence](mission/MOVING-TRAFFIC.md). Two early departures encounter different robots. A 16-second reservation permits two supported deliveries under the modeled stock schedule.

Open the [registered two-pipe field replay](mission/registered-field.html) or inspect its [route, robot-clearance and timing evidence](mission/REGISTERED-FIELD.md). This candidate runs from the red station to two demonstrated reef sectors within the modeled TELEOP clock.

The [station-to-L1 candidate mission](mission/README.md) follows an identified pipe from a station feed through the moving receiver and into first contact with a registered reef trough. It includes custody losses, timing limits and a conditional second trip. The [CORAL assessment](scoring/README.md) applies edition-specific criteria to observations, using the [release-day rules register](kickoff/rule-register-v0.json) as a separate kickoff record.

For a fresh kickoff exercise, start with the [release-day field model and opportunity study](kickoff/README.md). It compares two candidate roles from the V0 rules and times every loaded trip and empty return before proposing a design requirement.

- **Low-level, low-speed CORAL placement** with one preloaded CORAL.
- 0 degree guide, 0.15 m powered travel, 0.0005 kg m2 inertia per roller, refined duty 0.04.
- Ideal differential-drive approach, 200 ms stopped handoff, virtual pregrip/gate interlocks and powered roller model using WPILib DCMotor/shared supply.
- Refined placement conditionally stops inside the isolated floor patch at assumed friction 0.3/0.5; original 0.08 duty and low-friction cases fail.
- **No official scoring or hardware safety acceptance is claimed.**

## Run on Windows

Requires existing Python 3, PowerShell, Git, JDK 17 and populated Gradle/WPILib cache. No installation or robot deployment is performed.

```powershell
./run.ps1 -JavaHome '<existing JDK17 directory>' -GradleHome '<existing populated Gradle cache>'
```

Open the printed results directory: `SUMMARY.md`, `season-results.json` and `robot-run/receipt.json`. Each invocation executes the unchanged shared three-season regression, then selects this season's results. The extra regressions are deliberate shared-model checks, not other season projects owned by this repository. Fresh output directories and run IDs prevent reusing old passing XML.

## What lives here

The [six-wheel drivetrain engineering review](design/drivetrain/README.md) packages the selected NEO, 20:1 reduction, twin chain planes, bearing carriers, wheel hubs and protected drive circuits. Open any of its four whole-robot STEP poses or replay the 8.95 m station-to-reef transfer.

Open the [editable whole-robot assembly](design/station-l1/README.md) to inspect the differential chassis, telescoping receiver and battery service path in four poses. The package includes named STEP files, a parametric OpenSCAD model, its BOM and the measured interference corrections.

The [receiver load-path study](structural/README.md) connects extension, bumper setback and mounting demand to the decision about when the robot can carry or deploy the shelf.

The [2025 historical input package](historical/agent-frc-0.12/README.md) inventories the season-owned operating files, CAD, executable studies and byte-verified upstream replay route. The [release-day profile](kickoff/season-profile.json) separately owns the current field rehearsal.

- The [station-edge experiment](STATION-EDGE.md) calculates where an incoming CORAL leaves an idealized chute lip. The [receiver-contact screen](RECEIVER-CONTACT.md) follows it to first contact with an extended shelf. Both run with Python alone.
- The [receiver design comparison](STATION-RECEIVER.md) adds two editable full-size shelf solids, native tangency checks and a 72-case comparison of receiving reach, lip shape and incoming pipe motion.
- `cad/`: original fixture STEP, editable OCP generator and dimensions, overview SVG, and earlier sideplate SVG/DXF. No vendor CAD or fabrication-ready assembly.
- `shared/examples/java-2026-sim/src/`: actual Java controller/plant and integration tests, not a link to unavailable code.
- `shared/scripts/`: actual execution, finite-body target and evidence checks.
- `config.json`: season design description checked against pinned runtime duty/inertia. It is not yet a general tuning interface.
- `DESIGN.md`: design scope, assumptions and remaining work.
- `PROVENANCE.json`: exact source commit and file hashes; no Git history or private vault included.

The shared source snapshot is derived from agent-frc commit `21d785397cd08ca9dc5bf5f9849996ab508c9a49`. That development commit need not be accessible remotely to run this checkout. Gradle 8.11, GradleRIO/WPILib 2026.2.1 and JUnit 5.10.1 are pinned; 2024/2025 are game models using the 2026 desktop toolchain, not historical toolchain builds. See [dependency notices](shared/examples/java-2026-sim/THIRD_PARTY.md).

## Open and edit CAD

Open `cad/2025-original-fixture.step` in a STEP-capable CAD tool. Eight exported shapes describe the original floor/gate/guides, simplified rollers and nominal piece. The companion JSON records parameters and limitations. Export and STEP reopen were checked, not fit, loads or fabrication readiness. With an existing OCP 8.0.1 Python environment, regenerate using:

```powershell
python cad/build_original_fixture.py --season 2025 --output artifacts/cad
```

The top view is illustrative and omits guide-slot detail; STEP is the geometric artifact. CAD is not yet coupled automatically to the plant.

Only the documented run entry is supported by this minimal snapshot. Legacy tasks named in the retained upstream build file may have intentionally omitted sources. The configuration is a pinned description: edits are rejected, not silently treated as plant tuning. Shared source and CAD bytes are preserved by .gitattributes; other text uses LF.

## Public contribution checks

Run `python -m unittest discover -s tests` and `python scripts/check_public_content.py` after committing changes. The guard checks committed bytes, private paths/network addresses, unexplained machine aliases and binary additions. Reports identify file/category only. Manually review images/CAD metadata and novel formats before publishing; pattern checks are not a complete privacy guarantee.
