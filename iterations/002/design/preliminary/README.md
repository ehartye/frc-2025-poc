# Red 2 iteration 002 preliminary layout: baseline architecture A (guarded pan)

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

> **Baseline architecture A, superseded pending bench gates.** The evaluation (`iterations/002/design/evaluation`, 7c100a0) recommends replacing the pan with a fixed pocket and roller nip. That package is being built separately. This guarded-pan package is kept as comparator A. Its existing chassis, pan geometry, taps and screws are revisable premises, not fixed boundaries.

## Issue identity

| Item | Value |
| --- | --- |
| Issue | `red2-002-preliminary-layout-A` |
| Robot datum | The taps-02 root: chassis centre on the factory-tire floor (Z = 0), +X toward the dump (reef) face, +Y port, +Z up, in mm. `robotToStepMm` is the identity. |
| `design.json` | revision 2, sha256 `82b445bb6111b839bd366df9bb3c423a31d6600a5f59d2e4971c195fea41a394` |
| `layout.step` | 63 planar and revolved solids, sha256 `5c40bc4fce5880c697892508a6a03bc81baa6200221439f1b6a893d469f8b860` |
| `motion.json` | 14 cases, sha256 `e7e4bb38f5812baa5e567b35a3448926322785b063d7c20eb634c89fe4d033b7` |
| `budgets.json` | sha256 `1f7482bd5903d0f6b8f059c03faeed2e6146a88dd46ffd355f998d3b279335ad` |
| `layout.json` (plan) | sha256 `df9535305be501163812c610d63e091036e444d5ed90b650f89f440ba368bdf6` |
| `package-motion.html` | sha256 `f3e27a52052c88bf8532c431678bd55567944169ebca2d3813bc11a0a67474d3` (verified by the checker) |
| `receipt.json` | sha256 `2ccf89631297a9788342b0b9aad605dca78e366ad0657ed58366a6d9d12b554f` |
| Narrative pre-read | `kickoff/selected-role.md`, sha256 `4936b4a6…` |

## Receipt

- **Command:** `node scripts/design.mjs preliminary-check <worktree> iterations/002/design/preliminary/layout.json --require-checked --write-view`. Wall time was 3141 s; per-case times are in `receipt-run.json` and `receipt-progress.log`.
- **Plugin version:**
  - The process loaded Agent FRC 0.80.0 (157ca45).
  - Main moved to 0.81.2 (f8fcc24) during the run. `scripts/` is byte-identical between the two commits, so the receipt is valid for 0.81.2.
  - #135 adds a review requirement that the checker does not enforce: physics playback of field interactions. That requirement is open below.
- **Status: `provisional`.** All 14 cases pass, the displaced control is rejected, both budgets pass, all 11 premises are within their criteria and the view is verified. The only open items are two narrative objectives:
  - custody and detection
  - trough wall and support
- **Clearance:**
  - Every stationary and moving pair keeps at least the 2.0 mm case gap.
  - The tightest interval lower bound is **2.055 mm between Guarded pan and Pan cassette** in tip-to-dump.
  - The next tightest are 2.09 mm (pipe to feed panel 3, tip-to-dump) and 2.094 mm (pipe to chute crossmember 2, receive-chute).
- **Control:** a breaker-reset hand displaced (+560, −274, +120) mm into the pan arc collides with Guarded pan at t = 0.5 (1814.7 mm³). Both endpoints are clear.
- **Weight:**
  - The R103 heavy estimate is 50.107–51.563 kg against 52.163 kg, a margin of +0.600 kg.
  - The running mass is 64.594–66.049 kg, including the pipe, the 6.17 kg battery and the 7.5 kg bumpers. R103 excludes the battery and bumpers.
- **Voltage:**
  - The lowest weak-pack bus is 9.05 V in loaded-carry (11.8 V, 32 mΩ, conservative simultaneous 85.95 A), against the 9.0 V project threshold. The margin is 0.05 V.
  - The 8.81 V front-strike recovery bound from study 06 is an abuse event outside these stages and stays open.
## How this issue is built

- **Representation.** Every block is a closed solid in one robot datum: the taps-02 root, chassis centre on the factory-tire floor, +X toward the dump (reef) face, +Y port, +Z up, in millimetres. `robotToStepMm` is the identity.
  - Blocks are boxes, section prisms, cylinders (pipe, journals, bearings, gearbox) and solids of revolution (steer envelopes).
  - Mated reserves are partitioned at their mating plane. The lower-priority reserve is trimmed by the higher one inflated by 2.1 mm, or by 3.1 mm beside a moving reserve and 4.0 mm between each steer envelope and its housing.
  - The partition shell is the interface allowance. Seats, fasteners and contact inside it are detailed-design checks.
- **Fixed geometry.** The fixed blocks come from the 658-solid `four-seated-screws.step` (per-body boxes, clipped to the actual frame plan hull). The pan, upper support and inlet come from the r3 section data in `pan/geometry.json`.
- **Generators.** `build_layout.py` wrote `layout.step` and `layout-blocks.json`. `write_package.py` wrote `motion.json`, the budgets, premises, load paths, retention review, carry-forward record and `layout.json`. Both ran the agent's design tool and were removed with `design-inputs/revise.mjs`; their outputs stay here and Git history keeps the scripts. `make_review.py` builds `review/`.
  - `design.json` came from the plugin's `design new` with `design-inputs/design-project.json`, followed by `design-inputs/revise.mjs` (revision 2).
  - The rebuild order was: `design new` starter → `revise.mjs` → `build_layout.py` → `write_package.py` → `preliminary-check --write-view` → `make_review.py`.
- **`layout-blocks.json`.** For every block it lists the function, the fixed or moving parent, what it bounds and what is missing. The mount host and point for each group are in `layout.json` under `mounts`.

## Modelling corrections made during this issue

These are recorded in `modelling-corrections.json`.

- **MC1: steer sweeps.** Each steer sweep is now a true solid of revolution, static in every case. Before, it was a wheel cylinder at one azimuth plus coaxial cylinders, rotated 90° in the loaded carry.
- **MC2: pipe pose.** The STEP now holds the pipe at its stow seat, which `carriedWith` requires.
- **MC3: chassis split.** The chassis is split into perimeter, lower cassettes, pan trusses and stringers, and sills/decks/crossmembers, so box culling works.
- **MC4: retention sweep.** The retention sweep is recomputed from the seat.

## Cycle and cases

- **Operating cycle**, in order:
  1. receive-chute
  2. receive-over-edge
  3. receive-seat
  4. loaded-carry (stow hold band with `carriedWith`; static revolved steer envelopes)
  5. tip-to-dump (`carriedWith`)
  6. release
  7. reset-to-stow
  8. park (withdraw 300 mm from the scored pipe)
- **Service cases:**
  - unload pipe (full extraction)
  - pan cassette lift (full extraction)
  - battery lift 250 mm, then rearward carry
  - SB50 plug disconnect hand
  - breaker reset hand
- **Allowance:** every case uses `minGapMm` 2.0 with its own allowance basis.

## Interfaces

**Verified in this layout,** for the represented reserves only:

- The static clearance of all 63 reserves.
- The receive, carry, tip, release and reset paths of one pipe.
- The cassette and pipe withdrawals, the battery lift and carry, and both service hands.
- The mount points sit inside their host envelopes.
- The frame plan hull with a 17.25 mm effective rear cutback bounds every chassis and module body.

**Provisional:**

- Every mated face and seated contact: chassis joints, the module clamp, the stow pad seat (4.35 mm of the 6.5 mm stack represented), journals, the SB50 mate and the pipe on the chute, floor and lip. These are partition shells (2.1, 3.1 or 4.0 mm).
- The bumper ring: the carried 75 mm stack at Z 43–181 mm. The 002 mounts are undesigned.
- Tray, standoffs, brackets, looms and lead pass-throughs: these are reservations, not designs.
- Gearbox and NEO 550 length: the envelope ends at Y 350 and is unsourced.
- Pan clutch, absolute encoder and limit cam/switch: provisional (`load-paths.json`).

## Open inputs and findings

1. **Retention (`retention-review.json`):**
   - **R1:** the stow pocket has no positive restraint on its open face. Lift-off starts above 3.57 m/s² forward. A rigid sweep meets feed panel 3 after 10 mm, and the path up the floor is free.
   - **R2:** with power removed the pipe is retained only to 2 g opening contact. 3 g and above lose it (study 05). The power-off escape control is in `review/retention.svg`.
   - **R3:** friction and seated-pipe sensing are unmeasured.
2. **Narrative objectives still open:**
   - custody detection (no sensor placed)
   - the L1 trough and reef wall fixture (not placed)
3. **Field interaction playback (#135):** not embedded in this player. The physical evidence is the twelve-case replays (`evidence/P5-K5420-limit.field-bus.json`, study 05 and 06).
4. **Frame outline:**
   - The lower cassette corners sit outside both the 42 mm upper chamfer and the recorded 57.8 mm chamfer. This layout uses the actual 17.25 mm hull.
   - The Field Lab rear-corner clearance needs rechecking.
5. **Other open inputs:**
   - CORAL geometry is not registered in `rules-v0.json`.
   - No R103 entry is in the season rules (the limit is taken from inspection-mass-552).
6. **Most likely to invalidate the role: the 12 mm dock support margin** (`dock-support-margin`).
   - The modeled margin is 1.830205 mm against the 1.828296 mm baseline, a 1.9 µm margin. Physical dock error is unmeasured.
   - **Work budget:** 2 shop sessions plus 1 replay day.
   - **Nominal test:** 10 docks at 1.70 m/s cruise with a 250 ms settle on a fresh pack. Measure the stand-off error and replay the worst measured error. **Stop if** the support margin falls below 1.828 mm or a pipe is lost.
   - **Adverse test:** the same, on the 11.8 V / 32 mΩ pack at 67 kg with 1.5 m/s² braking. **Stop if** any pipe is lost or the reset deadline is missed.
   - Either stop sends the decision back to conceptual design.

## Revisable premises (owner guidance)

This preliminary design finds what will work. Existing Fusion CAD does not constrain it. The items below are retained on their evidence (`chassis-carry-forward.json`, `premises/`), and each lists the result that would justify replacing it.

| Premise | Evidence | Replace if |
| --- | --- | --- |
| Chassis `002-connected-machining-654-r6` | Six converged issues (cad/issue-001–006); occupied-shape closeout (8/8 same form); seated screws | The R103 margin cannot hold without structural savings; bumper or battery packaging needs another perimeter or corner; a better architecture needs other module stations or landings |
| Pan package `002-pan-r3` | Twelve-case r3 replays; the failed fixed-path comparison | Retention needs a retainer, latch or different stow and receive geometry (R1/R2); the dock test breaks the 1.828 mm margin; mass needs the inlet or cassette lightened. The evaluation has now recommended the fixed pocket and roller nip. |
| Taps and four seated 91251A542 screws | screws-place-01 native receipt | The chassis is replaced, or the load and maintenance review rejects the clamp |

## Handoff to detailed design

**Detailed work consistent with this layout:**

- The four SDS L2 module stations and cassettes: their housings and steer envelopes clear everything.
- The taps-02 nutbar taps and the four seated screws: inside the chassis reserve, with no conflict.
- The 552-r5 electronics placements: at least 2.089 mm from every reserve, tightest at the SB50 bracket to the main loom.
- The battery centre lift path through the 136 mm sheet gap.
- The r3 cassette, stops and journals as packaged: the pan clears its supports by at least 2.055 mm through the full tip.

**Detailed work a better architecture may supersede:**

- All r3 pan, cassette, inlet and stringer-extension detailing (taps-02 pan package, `native_pan_build.py`).
- The stow and receive geometry and its stops, spring, clutch and inhibit cam.
- The 30 A pan drive and its voltage stages.
- Any chassis pocketing (C2, C3) and the P3 inlet change, taken only for this architecture.
- If architecture B is selected, the chassis premise must be re-evaluated against B's mass and packaging before more chassis rework.

**Redo before release under any architecture:**

- Bumper construction and mounts on the actual frame hull.
- Battery tray and hold-down.
- Device brackets, looms and sheet pass-throughs.
- Native seats for every partitioned interface.

## Review bundle

`review/` holds only the player (`package-motion.html`, identical to the checked view), `retention.svg` and `index.html`. It contains no STEP, private CAD source or other files, and can be served on the LAN as a static directory.
