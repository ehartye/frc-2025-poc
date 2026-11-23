# Guarded pan and upper support: design package r3 (proposal)

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

This package installs the selected scoring mechanism on the accepted taps-02 chassis (`four-tapped-holes.f3d`, sha256 `e426e576…`). Revision r3 answers the eight blocking findings of the second independent review, plus its non-blocking items. It keeps OD-1 A, OD-2 A, OD-3 A, OD-4 mechanical joining and OD-5. Nothing here has run in Fusion.

| File | Purpose |
| --- | --- |
| `target-roster.json` | `article-functions@1` roster, validated by the plugin, SHA-256 `925b7823…`. It lists 493 occurrences: 136 fabricated (all built) and 357 purchased. The purchased rows include 142 rivets, 50 rivet nuts and 136 screws. |
| `operating-intent.json` | Owner decisions, `/operatingIntent`, poses, both stops, the opening-direction drive inhibit, gearbox protection, custody criteria, chassis interfaces and rules. |
| `geometry.json` | Sections, stock, mass-property deltas and OCP clearances. It holds 777 expected native gaps (111 pairs at 7 poses, 224 of them zero), plus seats, joints and build input. |
| `structure.json` | Rules, cradles, faults, soft-stop structure, root-moment joint, off-post paths, fasteners and rivet nuts, the gearbox, the spring and `worstMargins`. |
| `mass-plan.json` | The itemised R103 heavy estimate on the taps-02 issue. |
| `fabrication-plan.json` | `frc-fabrication-plan/v1` routes for the two non-2D operations: the hand-brake root clip and the waterjet hex. |
| `retained-from-001.md` | The retention decision, its evidence and the reason for each element. |
| `native_pan_build.py`, `job-inputs.txt` | The fail-closed native recipe and its re-hashed inputs. |
| `pan_geometry.py` → `pan_structure.py` → `pan_roster.py` → `pan_mass_plan.py` → `pan_intent.py` | Generators, run in that order. |

## Owner decisions recorded

- **Opening-only drive inhibit.** The −3.5° limit inhibits drive output in the opening direction only. It never cuts the closing drive and never latches, and the idle mode is **coast**. In `pan-stop-study-04`, a full cut fails 6 of 12 resets, while the opening-only inhibit never trips in normal running.
- **Gearbox protection.** Fit a mandatory slip clutch set between the 15.4 N·m normal-duty cap and the output-stage rating. The alternative is a documented output capacity of at least 26 N·m.
- **Retention from 001.** Retaining the pan contact surfaces, axis and drive beyond `commissioning.md:3` is accepted under the delegated mechanical-viability decision. The basis is the 12-case simulations and the failed fixed-path comparison, as recorded in `retained-from-001.md`.

## Blocking findings: what changed

1. **Root-moment joint.**
   - The r2 rivets are replaced by a 1 × 1 × 1/4 in angle, 75 mm tall (Z 502–577), bolted through both legs.
   - Leg A takes 3 × #10-32 through the bulkhead with locknuts.
   - Leg B takes 3 × #10-32 SHCS through the skin into the post inner plate. They sit at X 205, which leaves 8.65 mm to leg A for a 3.97 mm-radius head.
   - The bulkhead moved to X 220–224.8 and was deepened to Z 500.
   - The rows are 8 mm or more from the ends.
   - `root_moment_joint()` checks bolt shear, bearing, screw tension with prying, thread stripping, leg bending, post-stack bending and bulkhead bending.
2. **Fault envelope.**
   - The faults are rerun with the inhibit 5 ms late. F3 then reaches 1710 N per pad and −8.54°.
   - B1, the front bumper strike, opens the stowed pan at about 11.4 rad/s and gives 1183 N per pad.
   - The soft stop is now a solid 3/4 in bar clamped through a 1/4 in keeper under the X250 cradle, with no rivet nuts. Its pads are 18 mm, with the same 250 N/mm, so the replay law is unchanged.
   - The stow lug and bracket are sized for 2 × 296 N from a rear strike.
   - The joint limit and the sweep now run from −9 to 70.1°.
3. **Inhibit hardware.**
   - A waterjet limit cam sits on the port drive hex. Its lobe covers θ −14…−3.5° with 2° ramps and a 3.5 mm rise.
   - A roller-lever switch is seated on the port carrier, wired normally closed to the controller reverse-limit input.
   - The response requirement is ≤ 5 ms trip-to-neutral, measured. F3 needs only 1.46 ms to travel from −3.5° to the pad, so the inhibit is credited only for slow faults.
4. **Off-post screen.**
   - The I values are corrected: 29,106 mm⁴ with both stringers, and 16,654 mm⁴ for the upper stringer and skin.
   - Path A is the upper stringer cantilever. Path C is cheek torsion about the floor-held edge, with the floor acting as a diaphragm.
   - The load splits between them by stiffness, 318 N and 368 N.
5. **Blind fasteners.**
   - Every screw that lands in a closed tube now goes into a roster rivet nut, 50 in all.
   - Locknuts are used only on open faces.
   - Pull-out is checked against a required published value of 1.5 kN, which still needs a source.
6. **Recipe.**
   - The joint limits are −9…70.1°, which covers the 70.082° pose.
   - Every zero-gap witness also checks the native interference volume against the expected common volume.
   - Physical properties use `VeryHighCalculationAccuracy`, with a tolerance of max(0.5 mm³, 2e-4·V).
   - The joint name comes from `geometry.json`.
   - The exported STEP is re-read and its solids counted.
7. **Mass.** See below.
8. **Buildability.**
   - The posts are three waterjet plates (3/8, 3/8 and 3/16 in) with a cut hex and windows.
   - The lug is a stack of a 7/8 in plate, a skin lobe and a 1/2 in plate.
   - The cap and bracket are 2D profiles.
   - The 95.2° root clip has a hand-brake route registered in `fabrication-plan.json`. The plugin comparator accepts that file (desk comparison, exit 2).

**Non-blocking items:**
- The 8 extra bulkhead-clip SHCS rows are gone.
- The −7.7° pose is replaced by the −9° limit.
- `retained-from-001.md` now describes dome and dimpled heads and gives the inlet-post reason.
- `enduranceMPa` is renamed `fullyReversedStrengthAt1e4CyclesMPa`.
- Sources are recorded for Kt (Peterson, ch. 4), the UltraPlanetary rating (40 N·m failure) and the SPARK MAX limit inputs. The rotor-inertia and rivet-nut values are marked open.

## Worst margins (working 120 MPa unless noted)

| Check | Margin |
| --- | --- |
| Off-post 685 N at the cheek front end (path A) | **+0.07** |
| Root-moment joint, leg B bending (thread strip +0.92, bulkhead bearing +1.04) | **+0.14** |
| Soft-stop structure, keeper plate (F3 with the inhibit late) | **+0.42** |
| Landing-clip rivet-nut uplift | +0.42 |
| Web tube at its rivet holes, on yield | +0.43 |
| Cradles, F3 soft stop | +0.53 |

## Gearbox

- With an ideal inhibit, the worst fault reaches 25.8 N·m.
- With the inhibit 5 ms late, it reaches 41.0 N·m. That exceeds REV's published 40 N·m stage failure.
- Brake mode gives 41.5 N·m, so the idle mode is coast.
- The 26 N·m capacity route therefore holds only if a latency of about 1.5 ms or less is measured. Until then the clutch (16.5–18 N·m) is carried as required.

## Mass (R103, heavy)

| Line | kg |
| --- | --- |
| SDS L2 modules, catalog | 10.886 |
| taps-02 measured chassis, 146 bodies | 18.697 |
| Chassis screws, joining, electrical remainder, finish, spacers | 11.213 |
| Outstanding chassis item: the taps-02 seated screws | 0.036 |
| Pan bottom-up, including the clutch, switch and rivet nuts (6 of 17 purchased masses sourced) | 9.274 |
| Previous unweighed-article allowance | 0.693 |
| High case: joints 1.25, upper 0.50, loom 0.25 | 2.000 |
| **Total** | **52.800** |

The limit is 52.163 kg, so **the margin is −0.637 kg (−1.2 %). The heavy estimate is over R103.**

- P1 and P2 are applied, with the contact faces kept (−0.29 kg measured).
- P3, C2 and C3 (1.0 kg) are not applied. They need their own designs or a chassis issue.
- With all three applied, the margin would be +0.36 kg (0.7 %).

## Replay required (12 cases)

Use the r3 moving properties against the sim basis (`retained-from-001.md`):

| Property | r3 | Sim basis | Change |
| --- | --- | --- | --- |
| Mass | 3.6455 kg | 3.3452 kg | +9.0 % |
| First moment u | 309.2 kg·mm | 400.5 kg·mm | −22.8 % |
| First moment w | 106.2 kg·mm | 148.8 kg·mm | −28.6 % |
| Inertia about the pan axis | 0.09488 kg·m² | 0.11503 kg·m² | −17.5 % |
| Stow gravity arm | +1.62 mm | −0.86 mm | sign change |

On-axis additions are 28.5 g. The replay inputs are:
- the M1 contact
- the −5° soft stop at 90 mm (4050 N·m/rad, unchanged)
- the opening-only inhibit at −3.5°
- coast idle mode

## Open

1. The heavy mass is over R103. It needs P3, C2 and C3, or new savings.
2. The measured inhibit latency, and the drive checked placement of the clutch.
3. A sourced rivet-nut pull-out rating; the NEO 550 rotor inertia.
4. Bench pad stiffness and loss for the 18 mm pads.
5. The native structural route, including crippling of the cradle front wall under the soft-stop heel.
6. The end-clip and splice rows for the inlet portal moment.
7. The cam-to-carrier gap is 1.85 mm, and the bearing flange is not modeled.
