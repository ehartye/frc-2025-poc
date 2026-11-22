# 2025 Red 2 iteration 002: remaining work to a passing whole-robot readiness review

Prepared 2026-11-21. Read-only survey. Nothing was edited, committed or run in Fusion.

- Design: worktree `agent-frc-poc-assets-worktrees/red2-native-successor`, branch `private/red2-native-successor` @ 5d57252. Active iteration `iterations/002/design/` (`red2-two-sill-robot`).
- Canonical master: `cad/private/connected-654-precursor.f3d`, revision `002-connected-machining-654-r6`, archive `e7c7386a…`, STEP `c430890b…` (`cad/fusion-authority.json`, `cad/issue-006/`).
- Contract: Agent FRC a7097c1, `docs/mechanical-readiness.md`, `docs/article-functions.md`, `docs/operating-intent.md`, `docs/loaded-transfer.md`, `docs/fastener-stacks.md`, `docs/private-model-release.md`.
- Evidence produced today (outside git, this folder): `issue-006-saved-census.json`, `seven-retained-8-closeout.json`, `taps-02/`. `screws-01/` does not exist yet because another agent is doing that study.

Paths in the tables are relative to `iterations/002/design/` unless marked otherwise.

## 0. Headline gaps (gate blockers that no single study closes)

1. **No frozen target roster or operating plan.** The iteration has no `agent-frc/article-functions@1`, no `operatingIntent` or `transferMissions` JSON, and no `agent-frc/mechanical-readiness@1` study. The only commissioning record is the prose in `commissioning.md`. The review has nothing to reconcile occurrences against. This roster's article list (section 1) was reconstructed from that prose and the studies.
2. **No native joints.** The census pose has `joints: []`. All 146 fabricated bodies have `groundToParent: true` (`chassis/connected-native-03-cold.json` → `coldFixedAttachments`). There are no retained-joint observations, no moving groups (steer ×4, wheel ×4, pan) and no constraint path to a chassis anchor.
3. **256 positive overlaps, all inside the four SDS modules** (64 per module; 0 unresolved out of 2188 pairs). Each one needs a pair-specific `intended-fit` registration, or a failure if it is not intended. The largest classes are screw-in-tapped-host fits (10-32 × 0.5 vs. wheel ×32, NEO vs. 10-32 × 1.5 ×24, bevel gear vs. 10-32 × 0.75 ×24, plate vs. 10-32 × 2.5 ×24) and gear meshes. Four are the **retained steering-key vs. NEO interference control** (`Machine Key 2x2x12mm` ↔ `Motor, NEO`). That pair cannot be registered as an intended fit, so it fails until the steering motor and key are changed (owner decision D2).
4. **Most of the scoring robot is absent.** The pan, its drive, stop, sensors, bumpers, battery, electronics, wiring and all 116 + 4 planned shop screws are outside the 654-body issue.
5. **No drawings, rights records or BOM records exist for any article** (searched for `drawing`, `materialGrade`, `procurementBom`, `asDrawnBom`, `rightsBasis` and `vendor-configuration@1`: no hits in iteration 002).

## 1. Article roster

| # | Article / subsystem | Status | Says so |
|---|---|---|---|
| A1 | 4× SDS MK4i L2 NEO billet modules (508 solids: 2 NEO, CANcoder, stock 2×2×12 key, wheel and nitrile tread each) | **Installed in 654** | `cad/fusion-authority.json`; census (127 bodies × 4) |
| A2 | Lower frame: 6 structural perimeter, 8 socket-rail ends, 8 frame supports (6061-T6) | **Installed in 654** | census; `chassis/inset-chassis-shop.json` |
| A3 | Two longitudinal sills, sill seats, wheel saddles | **Installed in 654** | census (`Inboard sill`); shop plan |
| A4 | Upper/pan support: 2 pan crossmembers, 10 truss members, 2 upper stringers, 8 truss landings, 8 landing nutbars | **Installed in 654** (prospective sizing only) | census; `chassis/inset-pan-truss.json` (`limits`) |
| A5 | Electronics decks: 2 equipment sheets, 8 cassettes, 14 retaining tabs, 6 keyed pedestals + 6 nutbars, 8 key backings | **Installed in 654** | census; `chassis/inset-deck-register.json` |
| A6 | 48 wheel/sill sleeves | **Installed in 654** | census |
| B1 | 4× 91251A542 1/4-20 × 1 in. brace clamp screws + 4 UNC-2B taps in the two landing nutbars | **Designed, not installed.** Taps are modeled only in the `taps-02/` derivative (archive `e426e576…`). Seated screws are pending in screws-01. | `chassis/inset-clamp-hardware-selection.json`; `taps-02/native-receipt.json`; `PAUSED.md` |
| B2 | 116 shop-plan screws (68× 91251A539, 32× 91251A354, 16× 91251A351) | **Designed, not installed** (positions only) | `chassis/inset-chassis-shop.json` (`purchases`); `chassis/inset-connected-mass.json` (`plannedScrewCount`) |
| B3 | Electronics: PDH, roboRIO 2, VH-109, CB185 120 A breaker, 9 SPARK MAX (8 drive/steer + pan), battery plug pair (15 placements) | **Designed, not installed** (prospective placements; breaker source has 16 open shells) | `chassis/inset-electrical-package.json`; `supplier/electrical-limits/*.json` |
| B4 | Alternate steering motor (V1.1) + short P02 key | **Designed, not installed** (provisional registration; sampled withdrawal and leads) | `chassis/inset-v11-*.json`; `commissioning.md` |
| B5 | Pan floor 430 × 322 mm | **Reference only** (form carried forward, not editable shop part) | `../carry-forward/floor-profile.json`; `references/pan-floor-430x322.step` |
| C1 | Pan moving article: shaft, cartridge, 2× REV-21-1915, 6655K17 thrust sets, diaphragm/shim/end cap, guides, mouth wear surfaces | **Not designed** in 002 | `chassis/inset-upper-named-mass.json` (`openArticles`); `chassis/inset-connected-mass.json` (`panOutstanding`) |
| C2 | Pan drive: NEO 550, MAXPlanetary 3:1/4:1/5:1, 13T pinion, ½ in. hex adapter | **Not designed** (catalog masses only) | `chassis/inset-upper-named-mass.json` |
| C3 | Pan passive stow stop / hard stops | **Not designed** (sign, force, rotation and recovery still unmeasured) | `commissioning.md` (Pan row); `strategy/stow-stop.json` |
| C4 | Sensors: pan pose, piece presence, limits, fixture attachments | **Not designed** | `chassis/inset-upper-named-mass.json` |
| C5 | Bumpers: backing, padding, covers (alliance colors), mounts, reverse retention | **Not designed** (beam screen only) | `chassis/bumper-duty.json` (`open`); `chassis/bumper-spreader-clearance.json` |
| C6 | Battery retention, electronics brackets, wiring loom, CAN/power harness | **Not designed** | `chassis/inset-electrical-package.json` (`scope`) |
| C7 | Battery (exact article) | **Not designed** (service envelope only) | `chassis/inset-electrical-package.json` (`service`) |
| C8 | CORAL game-piece model for loaded transfer | **Absent from 002** (historical experiments only) | repo search |

**Counts:** 6 article groups installed (654 bodies). 4 designed but not installed (B1–B4, about 135 discrete parts plus 15 electronics placements). 1 reference-only. 8 not designed.

## 2. Readiness study inputs (docs/mechanical-readiness.md table)

| Field | Exists today | Missing |
|---|---|---|
| `configuration`, `archive`, `step` | Canonical F3D/STEP pinned, lineage with zero external references (`cad/issue-006/`) | A new canonical issue for each installation step that follows (taps-02 is a derivative, not canonical) |
| `reopen` | Cold reopen `chassis/connected-native-03-cold.json`; full census today (654 bodies, 0 hidden, 0 other geometry) | `nativeBodyNames` STEP↔Fusion map and `installedOccurrences` with article/revision/modelSha256 in reopen form; `movingGroups`/`motionRanges` (none exist); `inspectionPoses` beyond the saved pose |
| `geometry` | Native interference over 2188 pairs, complete (`issue-006-saved-census.json`); exchange match (`chassis/connected-654-exchange.json`) | A `mechanical_readiness.py measure` scan of the issued STEP. The 647/seven-stock occupied-form check is closed for 8 retained bodies (`seven-retained-8-closeout.json`, offline only; supplier solids fail OCCT Booleans). |
| `articles` (purchased) | SDS source selection/readback (`private-issues/supplier-l2/`); supplier electrical files | `procurementBom` + `asDrawnBom` (`vendor-configuration@1`) for the SDS modules (tread, motor, CANcoder ordered separately), REV/CTRE electronics, McMaster hardware; `sourceSolids` maps; `neutralSha256`; `shapeResolutionMm3` |
| `articles` (fabricated) | 146 editable native bodies, all 6061-T6 in the shop plan | Controlled `drawing` for each of the 146 (about 30 distinct shapes) and for each future part; `materialGrade` binding per article; revision |
| `rights` / `rightsBasis` | None. 001 retrospective: "OEM format-specific publication permissions remain unresolved" | Item-level basis per purchased source (SDS, REV, CTRE, McMaster, battery). Choose `exact-redistribution`, `derived-views-only` or `private-master`. |
| `installed` | Census world transforms | Expected-occurrence register keyed by native path with article + row-major transform |
| `chassisAnchor` | None named | Choose a measured solid (for example a structural perimeter member) |
| `connections` | None (everything is ground-to-parent) | `retained-joint` observations for each fabricated joint (pair, mechanism, loadPath, inspection, maxInterfaceGapMm). Module-to-frame screws are needed first. |
| `mating` | None | 252 `intended-fit` observations inside the modules (registered or failed); B1/B2 thread fits via fastener stacks; the 4 key/NEO pairs stay failing until D2 |
| `duties` | Many offline screens (section 3) | None are in `mechanical-measurements@1` form bound to a final archive and STEP |
| `coverage` | None | Independent `engineering` and `build` scope reviews (reviewer identity = owner decision D7) |
| `distribution` | None | `approvedBy`, pinned files with scope and rightsBasis; signed packet and reviewer key (`private-model-release.md`) |
| `operatingRequirements` | Prose only (`commissioning.md`) | JSON `operatingIntent` with `publicationScope: "whole-robot"`, `motionRanges`, `nativeCoordinates`, `transferMissions`, `transferCriteria`, `structuralRoutes`, `articleFunctions` digest |

## 3. Duties implied by the operating plan

| Duty | Status | Evidence | Next discriminating step |
|---|---|---|---|
| Sweep: steering ×4 full azimuth, wheel spin | Partial | `chassis/inset-v11-three-axis.json`, `inset-v11-quarter-turn-casing.json` (sampled, alternate motor) | Native revolute joints, then continuous sweep with an interfering control at the chosen motor/key |
| Sweep: pan stow → feed → tip → reset | None (simulation only) | `strategy/limited-pan-12mm.json` | Requires C1–C3 installed; native poses and a STEP sweep |
| Shop: rebuild the hardest feature/joint | Partial | `chassis/inset-chassis-shop.json` (step plans for 146) | Pick the shop (D7). Rebuild the mitered perimeter + cassette key seats and the tapped nutbar from drawings. |
| Assembly | Partial | `chassis/front-corner.json` (`assemblyOrder`); `chassis/inset-cassette-travel.json` | Removal-order proof from the installed inventory after B1/B2 installation |
| Service: module withdrawal, battery swap, breaker | Partial (prism or box bounds) | `chassis/cartridge-service.json`; `chassis/inset-breaker-access.json`; `chassis/inset-electrical-package.json` | Repeat on installed screws and bumpers, then test the actual tool path |
| Fastener stacks (120 screws) | Partial (4 in progress) | screws-01 (other agent); `chassis/inset-brace-clamp-window.json` | Seated stacks for B1, then the 116 B2 screws by representative class |
| Keyed steering torque | **Failing control** | census key/NEO ×4; `chassis/inset-oem-steering-key.json` | D2, then flank/withdrawn cases per `operating-intent.md#check-torque-through-a-straight-key` |
| Strength (frame, sills, truss, screws) | Partial (beam screens on earlier issues) | `chassis/diagonal-lower-elastic.json`, `chassis/inset-corrected-foot-loads.json`, `chassis/inset-loaded-controller-truss.json`, `chassis/terminal-bolt-loads.json` | `structuralRoutes` with the full force/moment set on the installed 654 successor |
| Stiffness | Partial | `chassis/inset-deck-register.json` (finite mounting scenarios) | Deflection against pan and bumper clearances on the installed issue |
| Impact (station, wall, side, pinch) | Partial; **bumper path open** | `chassis/inset-impact-boundary.json`; `chassis/diagonal-inertia-relief.json` (241 MPa pinch control); `chassis/bumper-duty.json` | Design C5, then the supported path from the miters into both rail webs |
| Fatigue | None (words only) | n/a | Choose a life basis for the perimeter, screws and pan shaft |
| Loaded transfer (CORAL, delivery + jam/reset) | None | none in `loaded-transfer@1` form | C8 piece model, installed pan, then `executed-trajectory@1` replay |
| Two scored trips on the match clock | Partial (simulation, 12/12 cases, min reset 89.8 ms, 1.83 mm support margin) | `strategy/limited-field-four-terminals.json`, `strategy/limited-field-12mm-executed.json` | Integrated loaded, sensed run on the installed robot. The commissioning trigger applies: if this fails, compare fixed-path first. |
| Mass / inspection | Partial (estimate) | `chassis/inset-connected-mass.json` (654 basis: R103 margin 2.1–4.1 kg) | Replace the allocations with installed mass. **`commissioning.md` still quotes the older 552 figures (1.9 kg over R103).** |
| Electrical (bus, 30 A pan limit) | Partial | `strategy/timed-bus.json`, `strategy/drive-envelope.json` | Rerun with the installed harness and battery article |

**Duty tally:** 0 have closed evidence in contract form, 11 are partial, 3 have none, and 1 is a failing control.

## 4. Critical path

**N** = native Fusion session. **O** = offline analysis. ★ = needs the owner.

1. ★ **O. Owner decisions D1–D5, then freeze the target roster** (`article-functions@1`) and the JSON `operatingIntent` (motion ranges, transfer missions, structural routes, `publicationScope`). Everything else is reconciled against this.
2. **O. Close screws-01** (in progress) and decide whether taps-02 becomes the next canonical base.
3. **N. Issue a new canonical:** taps + 4 seated 91251A542, the 116 B2 screws and the D2 steering motor/key. Then cold reopen, census and STEP.
4. **N. Replace ground-to-parent with real rigid joints** to a named anchor, plus steer/wheel revolutes. Pin native pose receipts.
5. **O, in parallel with 3–4:** 252 intended-fit registrations (some reusable across all 4 modules); SDS procurement vs. as-drawn records; rights basis per supplier (★ D6 if a supplier must be asked); CORAL piece model; mass ledger refresh.
6. **N + O. Detail the pan (C1–C4)** against the frozen interfaces, with stop and sensor measurements first, as the commissioning plan requires. This is the largest single block.
7. **N. Install electronics (B3), battery and retention (C6/C7), and the harness envelope.**
8. **N. Design and install bumpers (C5).** This can start after step 4 and run in parallel with 6–7.
9. **N. Drawings + material grades** for every fabricated article (146 plus new parts). Start drawing the frozen lower right after step 3.
10. **O. Duty campaign on the integrated issue:** structural routes, impact, fatigue, sweep, service and removal order, shop rebuild, loaded transfer, integrated two-trip run.
11. **O + ★. Coverage scope reviews and distribution** (approver, reviewer key D7), then the checker, private issuance, release verification, and the showcase/README stills.

Offline work that can run in parallel without Fusion: step 5, the drawing templates, and the structural/impact models. The native steps are serial on one station (3 → 4 → 6/7/8 → 9 → final issue).

### Owner decisions

- **D1. Upper architecture.** Guarded pan (current) vs. the simple fixed-path comparison. The commissioning plan only requires the comparison after an integrated failure. This decision gates C1–C4.
  - *Option A: detail the pan now.* Time: slower (4–6 N before the first integrated test; ranks 2nd). Risk: higher (001 recorded zero full-robot scores and an 11.7 vs. 8.2 kg upper; ranks 2nd). Complexity: higher (thrust cartridge; ranks 2nd). Best practice: weaker (`frc-whole-robot-design` asks for a fair alternative after repeated failure; ranks 2nd). Maintainability: same as B (tie).
  - *Option B: run the offline fixed-path comparison first (about 1–2 O), then detail the winner.* Time: ranks 1st if the pan wins, because the delay is small. Risk: lower (ranks 1st). Complexity: lower (ranks 1st). Best practice: closer (ranks 1st). Maintainability: same as A (tie).
- **D2. Steering motor and key.** Keep the stock NEO and key (fails permanently) or buy the V1.1 motor + short P02 key. Purchase.
- **D3. Tread.** Blue nitrile as modeled vs. the black tread that was quoted. Purchase, then loaded-radius qualification.
- **D4. Bumper construction** and alliance cover scheme.
- **D5. Battery article, electronics set and harness standard.** Purchase.
- **D6. Supplier publication rights:** exact redistribution vs. derived views only.
- **D7. Shop, independent engineering/build reviewers and the release approver/key holder.**

## 5. Size estimate and reconsideration

- **About 20–30 native sessions:** canonical screws/taps/motor 2–3; joints and moving groups 2–3; pan 5–8; electronics, battery and harness 2–3; bumpers 2–3; drawings 4–6; pose receipts and final issue 2–4.
- **About 25–40 offline studies:** 252 fit registrations batched about 3–5 ways; BOM/rights 3–4; 15 duty measurements; scope reviews; release.
- This is **11 major steps**, several weeks of station time. The lower frame is about 40 % of the design and about 20 % of the remaining evidence.
- **Is `frc-whole-robot-design` warranted? Yes for the upper, not for the chassis.**
  - The chassis went through six native issues (123 → 244 → 488 → 504 → 552 → 654) and retired several schemes: crossed tubes over 120 MPa, a narrow-radial perimeter, saddle variants, clamp screws through the diagonals. These were converging local fixes, and the 654 frame now has no fabricated-body overlap.
  - The scoring role has failed across iterations. 001 recorded a mass overrun, foot equilibrium failure, a cartridge insertion block and zero recognized full-robot scores. The steering-key clash has also survived from 001 into 002. The 002 pan exists only in simulation.
  - Under the skill's "unattainable scoring role survives local fixes" trigger, the fixed-path comparison (D1 option B) should come before the pan detailing. The key clash is a purchase decision (D2), not an architecture failure.

### Housekeeping noted (not edited)

- The `pause-20261121.json` field `activeWorktree` names `frc-2025-poc-worktrees\red2-native-successor`. The actual worktree is `agent-frc-poc-assets-worktrees\red2-native-successor`.
- The mass paragraph in `commissioning.md` predates the 654 ledger.
