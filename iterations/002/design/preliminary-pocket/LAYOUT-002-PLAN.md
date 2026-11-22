# Red 2 B'x layout-002: working plan (resume from here)

Goal: complete the preliminary design so the owner can accept it as **effective**. Issue it as public release
`layout-002` (status in-progress), successor of `layout-001`. Bar: whole mission cycle checked for clearance, mass and
voltage; every mission-critical interaction passes a physics trial with a control, sensitivity brackets and a
half-timestep check; the mission animation is trial-driven; every open risk is closed by a design change or recorded as
a bench test with pass/fail numbers. No physical bench results needed. No Fusion.

## Where things are

- Private design: this folder (`iterations/002/design/preliminary-pocket/`), branch `private/red2-native-successor`.
- Public project: `.`, worktree to create at
  `.` (branch `bx-layout-002`). PR, do not merge.
- Plugin: `PLUGIN` (0.111.1 at start). Friction goes to agent `plugin-backlog`.
- Python: `python -I -B`.
- Wiki: `WORKSTATION/repos/frc-wiki-vault/wiki/authored/frc/2025-poc/` (wiki-master skills only, no git).

## Custody gap found (2026-11-22)

The layout-001 public release was checked from a **re-rooted copy that was never committed**: it existed only in a
session scratchpad (`package.step`, `field-fixtures.step`, `paths.json`, the `presentation` layout, `missions.json`,
`narrative.md`). The committed builders here still write the pre-release form (`layout.step`, `motion.json`).
Layout-002 must be produced by committed builders in the release form, and the layout-001 re-rooted inputs should be
preserved here (`releases/layout-001/`) verified against `custody-layout-001.json` hashes.

## Design decisions for layout-002 (made 2026-11-22, evidence: field_trials.py @3 run, 1 min 20 s)

1. **Drive: self-locking worm removed.** One NEO 550 through a REV MAXPlanetary 4:1 x 3:1 x 3:1 (36:1, 1/2 in hex socket
   output; sourced in the wiki log `2026-11-22-143007-discover-2025-l1-roller-mechanism-purchasable-parts`), with a
   powered position hold on receive and carry, back-drivable when disabled. Trials: the worm stalls on treads of
   110 N/mm and above; the planetary ejects 480/480 over 40-160 N/mm and stalls at 240. Disabled receive 4/4 and
   disabled sled 18/18 hold, so the lock is not needed. Lower ratios (27:1, 20:1) overspeed the roll-off (lost over
   the riser at 1.9-2.0 m/s), so 36:1 is the ratio.
2. **Tread: acceptance band 40-160 N/mm per roller at 6 mm, nominal 80.** Receive 108/108 in the band with tread mu
   0.3-1.0; fails at 10 N/mm (9/12), passes at 20 and 30. No published stiffness exists for any 2 in compliant wheel
   (researched), so the wheel count and durometer are set on the bench to the band: bench number 240-960 N at 6 mm.
3. **Receive dwell is the captive time, not the seat time.** The pipe is captive when the check flap closes behind it
   (0.093-0.18 s after it passes the flap). Seat time is 0.09-1.25 s (rebound) and no longer matters: receive-and-go
   trials (leave 50 ms after the flap closes, 3 and 6 m/s2) retain 24/24 and seat within 0.103 s of the stop. Without
   the flap the same trial loses the pipe rearward 2/2, so the flap earns its place.
4. **Receive now starts on the registered station chute** (35 deg, 419.1 mm ramp, sill 952.5 mm; rest release exits at
   1.48-1.81 m/s and peaks 2.44-2.96 m/s before the flap; pushed feeds to 3.6 m/s still seat 48/48).
5. **Lip stays at 100 mm.** A 72 mm lip (tip inside the bumper face) loses 5/8 at 100 mm stand-off; no lip loses
   everything from 50 mm. 125 mm stand-off fails 3/8, so the accepted dock stays 0-100 mm.
6. **Second trip: needs lateral separation.** With the first pipe in the same section only 24/180 second ejects are
   captured (93 stay on the lip, 63 fall short). Sector pitch 471 mm less pipe length 301.6 mm leaves 169 mm, so the
   lateral dock error must stay within +/-70 mm at each sector (new premise and G4 measurement).
7. **G412.** Final-manual text (5.2): the reef zone is 93.5 in face to face around a 65.5 in reef, so its tape is
   14.0 in (355.6 mm) outside the reef face. A dock that can score (stand-off <= 100 mm) puts the bumper at least
   255 mm inside the zone. Interlock sensor: CTRE CANrange on the starboard plate reading the trough face.
   Blue box: a launch is "shot into the air, kicked across the floor ... or thrown in a forceful way"; the roll-off
   leaves at 0.5-1.6 m/s. The V0 text could not be downloaded; the final edition was read.
8. **Sensors:** seated-pipe break beam (Adafruit 2168) through the plates at X 314.7, Z 654 (blocked for pipe centre X
   296-333); flap-closed and lip-deployed REV magnetic limit switches (REV-31-1462); CANrange reef range.
9. **Structure:** lip, hinge, stops, shafts and plate landing sized by calculation in `structure_checks.py` (to write).
   Trial loads: roller 968 N peak (160 N/mm), lip 395 N and 35.8 N m about the hinge with a rigid lip.

## Items (status: todo / doing / done)

| # | Item | Status | Finding so far | Next step |
| - | ---- | ------ | -------------- | --------- |
| 0 | Reproduce layout-001 trials and check | trials done | `field_trials.py` at 52c9d46 reproduces `fd5a7e6e...` bit for bit (24 s) | rerun the layout-001 check from the preserved re-rooted copy when the layout form is settled |
| 1 | Receive seat time (0.145-0.689 s, premise <= 1.0 s) | todo | Seat time measures rebound settling, not custody. The pipe is captive once the check flap closes behind it. | Trial: leave the station on flap-closed and settle en route; report captive time and the seated state before eject |
| 2 | Tread stiffness (40 pass, 10 fail) | todo | Locked-nip barrier is friction work, proportional to k; the worm drive (3.5 N m at 30 A, 40 %) stalls on stiff treads, so the usable stiffness window is narrow on both sides | Sweep k 10-160 N/mm for receive pass and eject stall; widen the window by drive choice; record the installed-stiffness bench number |
| 3 | Worm self-lock >= 4.10 N m, no part | todo | Carry custody held with free rollers in layout-001 trials; the lock earns its place only on receive timing | Compare worm vs planetary with powered hold; select a real reducer |
| 4 | Lip stop, tip load, side-plate attachment, sensor brackets | todo | | Size by calculation; place brackets as blocks |
| 5 | G412 reef zone | todo | | Register the reef zone as a field fixture/zone, specify the interlock sensing and its error budget |
| 6 | Second trip into sector B | todo | clearance check only | Trial the sector-B eject with the first pipe present |
| 7 | Bench gates G1-G5 | todo | all open | Re-predict each from layout-002 trials with pass/fail numbers |
| 8 | Weak-pack bus 9.28 V vs 9.0 V; mass 48.3 of 52.16 kg | todo | | Re-run budgets after part changes |
| 9 | Station chute fixture | todo | not registered | Register with a qualified intended contact |
| 10 | Trial-driven mission motion | todo | layout-001 cases are straight/circular strokes pinned to trials | Drive case timelines from trial traces |

## Log

- 2026-11-22: session start. Read plugin skill and docs, private package, public release. Plan written.
- 2026-11-22: baseline trials reproduced. Research agent returned sourced parts and rule text. `field_trials.py`
  rewritten to schema @3 (committed). Decisions 1-9 above. Next: `structure_checks.py`; `build_layout.py` (drive block,
  sensors, fixtures STEP, package.step); `write_package.py` (release form under
  `cad/preliminary-design/releases/layout-002/` with the repository as project root, new trial summary keys, premises,
  budgets, missions, narrative); check; stage; review; PR; wiki. Waiting on `plugin-backlog` for the status of the view
  location and single-viewer changes before staging.
- 2026-11-22 (later): `structure_checks.py`, `build_layout.py`, `write_package.py` and `design-inputs/revise-002.mjs` rewritten
  for the release form. Outputs go to `cad/preliminary-design/releases/layout-002/` with the repository root as the
  project root. Acceptance band narrowed to 40-120 N/mm (steel shafts). Sprung lip stop (6 N m, 31 deg over-travel).
  Writer result: 48 cases (23 receive and 14 eject strokes of the two mission traces), heavy mass 49.31 of 52.16 kg,
  weak bus 9.118 V. Layout-001's re-rooted package copied to `cad/preliminary-design/releases/layout-001/` and verified
  against all 33 custody hashes.

## Reproduce layout-002 (from the repository root; PY = the CAD python, PLUGIN = agent-frc checkout)

```text
R=cad/preliminary-design/releases/layout-002 ; P=iterations/002/design/preliminary-pocket
PY -I -B $P/field_trials.py $R/evidence/field-trials.json --processes 14
PY -I -B $P/structure_checks.py $R/evidence/field-trials.json $R/evidence/structure-checks.json
PY -B $P/build_layout.py . $R PLUGIN/scripts $R/evidence/field-trials.json
node $P/design-inputs/revise-002.mjs PLUGIN $P/design.json $R/design.json
PY -B $P/write_package.py . PLUGIN/scripts
PYTHON=PY node PLUGIN/scripts/design.mjs preliminary-check . $R/layout.json --write-view --require-checked    --previous cad/preliminary-design/releases/layout-001/receipt.json --output $R/receipt.json
```

The builders refuse to overwrite: delete the generated files in `$R` (everything except what an earlier step wrote)
before re-running a step.
