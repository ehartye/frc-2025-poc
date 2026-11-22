# Red 2 iteration 002: whole-robot R103 mass study (mass-01)

Prepared on 2026-11-21. This was an offline, read-only study of the worktree `red2-native-successor`. Nothing was edited or committed, and nothing ran in Fusion. All figures are planning estimates, not scale weights. The machine-readable record, with SHA-256 pins for every input, is in `mass-study.json`. `mass-study-data.json`, the three `mapping-*.json` files and the `assembly-check-*.json` files are the plugin's signed-accounting inputs and outputs (`scripts/assembly-accounting.mjs`, Agent FRC 1419020).

## 1. What R103 counts

- **Limit:** 115 lb, which is 52.163 kg.
- **Counted:**
  - the structure and every mechanism in the inspected configuration
  - robot-side leads
  - the battery tray and its retention
  - bumper mounts that stay permanently attached
- **Excluded:**
  - the bumpers
  - the battery, with its battery-side connector, lugs, fasteners, insulation and up to 12 in of lead per leg
  - the game piece
- **Rule source gap:** the season project's `kickoff/rules-v0.json` has no R103 entry. The basis used here is the `ruleBasis` in `inspection-mass-552.json` (2025 V0, pp. 76/86/112). Confirm it against the season's current manual revision.
- **Exclusion check:** the 52.800 kg heavy estimate includes none of the 6.17 kg battery, the 7.5 kg bumpers or the 0.816 kg piece, so nothing excluded has been counted.
  - One possible small overlap remains. The electrical budget names "main leads". If those leads include the excluded battery-side lengths, under 0.1 kg is counted twice. This was not credited.

## 2. Corrected ledger

| Basis | kg | Lines |
| --- | --- | --- |
| Measured CAD volume | 26.342 | 146 taps-02 chassis solids (18.697); pan fabricated (7.645) |
| Measured supplier solids | 0.815 | 116 chassis screws (0.797); steel spacer (0.017) |
| Catalog | 11.234 | 4 × SDS MK4i L2 NEO at 6.0 lb (10.886); 6 sourced REV pan articles (0.348) |
| Estimated | 1.318 | 11 unweighed pan purchases incl. 0.200 clutch (0.435); pan fasteners and hardware (0.846); 4 clamp screws (0.036) |
| Allowance (kept once) | 10.399 | joining residual 3.403; electrical residual 5.496; hardware and finish 1.500 |
| **Nominal** | **50.107** | margin **+2.056 kg** |
| Item-specific growth (one layer) | 1.456 | 2 % on CAD stock, 3 % on modules, 5 % on supplier solids and sourced articles, 50 % on estimated purchases, 20 % on count-based fasteners, plus 0.15 kg for pan articles not yet itemised |
| **Honest single-contingency heavy** | **51.563** | margin **+0.600 kg (1.2 %)** |
| Stacked layers removed | 2.693 | 0.693 pan residual; 1.25 joints, 0.50 upper, 0.25 loom |
| r3 plan heavy, for reference | 52.800 | margin −0.637 kg |

The growth percentages are project planning choices, following the NASA method in the wiki: one item-specific allowance, with no second allowance on items already estimated. Change them in `mass_study.py` if the team chooses differently.

## 3. Accounting findings

1. **AF1 (high, 1.193 kg).** Two upper contingencies are stacked on top of an itemised pan.
   - The 0.693 kg line is what was left of the retired 8.2 kg allocation after its named items. Once the bottom-up pan replaced that allocation, the line had no remaining basis.
   - The 0.5 kg high-case upper was defined against the same 8.2 kg allocation.
2. **AF2 (high, 1.25 kg).** Joining is covered three times: by the 3.403 kg residual, the 1.5 kg hardware and finish allowance, and the 1.25 kg high case. This is on a chassis whose 146 bodies and 116 screws are already modeled.
3. **AF3 (medium, 0.25 kg).** The loom high case stacks on an electrical remainder that already names the harness and branch extensions.
4. **AF4 (high).** The C2 and C3 credits disappear if `inset_connected_mass.py` is rerun.
   - The script computes `electricalAllowanceAfterSheetsKg` as 7 minus the sheets.
   - It computes `remainingFixedJoiningAllowanceKg` as 8.087 minus the backers, and the backers include the pedestals.
   - Any pocketing therefore flows straight back into the allowances. Freeze both as absolute kilograms, or itemise them, before taking credit.
5. **AF5 (low).** The README's "+1.074 kg versus allocation" understates the upper's growth. The 0.908 kg chassis pan truss was also charged to the 8.2 kg allocation, so the true figure is 10.182 kg against 8.2 kg, or +1.982 kg.
6. **AF6 (low).** `commissioning.md` still quotes the 552 figures (54.063 kg, 1.900 kg over).
7. **AF7 (medium).** The battery tray and retention and the permanent bumper mounts count toward R103, but no line names them. They must be named when the allowances are itemised.

## 4. Ranked candidates

In the table, "Native" means a native Fusion change is needed and "Replay" means the 12-case simulation must be rerun.

| Rank | ID | Change | kg saved (nominal / conservative) | Structure risk | Role risk | Shop effort | Native | Replay |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | A1 | Retire the stacked layers and carry one growth layer | 1.237 / 1.237 | none | none | none | no | no |
| 2 | C2 | Pocket the equipment sheets about 30 % | 0.45 / 0.30 | medium | low | low | yes | no |
| 3 | C3 | Pocket the six keyed pedestals | 0.30 / 0.20 | low–medium | none | medium | yes | no |
| 4 | P3 | Inlet posts to 3/4 × 3/4 × 1/16 in tube | 0.25 / 0.18 | low–medium | low | medium | yes | no |
| 5 | F1 | Titanium for the 68 deck-retaining 1/4-20 screws | 0.176 / 0.15 | low | none | low (purchase) | no | no |
| — | E1, J1 | Itemise the electrical and joining allowances | likely large, **not credited** | none | none | none | no | no |
| — | X1 | Drop the clutch through the 26 N·m route | 0.20 / 0, **not credited** | high | medium | low | no | yes |
| — | W1 | Same-vendor wheel or tread | ≤0.04, **not credited** | none | low | low | yes | yes |
| ✗ | R1 | Crossmembers or sills to 1/16 in wall | **rejected**: roof bearing 68.6 MPa would double past 120; saddle ring 124 MPa | | | | | |
| ✗ | R2 | Perimeter, socket rails, cassettes, saddles | **excluded**: perimeter 149–241 MPa (open), cassette 122.6 MPa, saddle 124.0 MPa | | | | | |
| ✗ | R3 | Further moving-pan lightening | **rejected**: off-post +0.07, leg B +0.14, keeper +0.42; reopens the replay | | | | | |

**C2.** The sheets carry deck-key loads.
- The worst flat bearing is 109.5 MPa of the 120 MPa working limit, at the (110, −95) key.
- Keep solid ligaments around every key and keep the shear path between keys solid.

**F1.** The keys carry the deck shear, so these screws only retain the decks. Before switching, check preload, thread engagement in the tapped 6061 backing, and galling.

**Swerve.** The 6.0 lb module mass already includes wiring. The Falcon and Kraken variants are heavier, so there is no module saving.

## 5. Package options

### Option A: A1 + C2 + C3 + P3 (recommended)

Heavy-case margin: **+1.600 kg nominal (3.1 %), +1.280 kg conservative.**

- **Time.** About 2 native sessions (one chassis re-issue, one inlet), plus offline deck and pedestal checks. Ranks 2nd.
- **Risk.**
  - The deck register is near its limit.
  - The conservative end misses 1.5 kg.
  - Ranks 2nd.
- **Complexity.** Adds one attachment detail and two pocket programs. Ranks 2nd.
- **Best practice.** Single contingency, with savings only where margins exist. Ranks 1st (tie).
- **Maintainability.** Requires frozen allowances (AF4). Ranks 1st (tie).

### Option B: Option A + F1 titanium deck screws

Heavy-case margin: **+1.776 kg nominal, +1.430 kg conservative.**

- **Time.** Option A plus procurement. Ranks 3rd.
- **Risk.** Adds a galling and preload check. It is the most robust of the three for reaching the target. Ranks 1st.
- **Complexity.** Adds a second fastener grade to control. Ranks 3rd.
- **Best practice.** Same as Option A. Ranks 1st (tie).
- **Maintainability.** Spares and grade mix-up risk. Ranks 3rd.

### Option C: keep the stacked r3 ledger and add every physical saving

Heavy-case margin: **+0.539 kg nominal, +0.193 kg conservative.** It does not reach 1.5 kg without touching the open structure.

- **Time.** Same work as Option B. Ranks 1st only because no accounting change is needed.
- **Risk.** Leaves a thin margin. Ranks 3rd.
- **Complexity.** No ledger change. Ranks 1st.
- **Best practice.** Contradicts the single-allowance method. Ranks 3rd.
- **Maintainability.** The contingency stays opaque. Ranks 2nd.

**Verdict.**
- **At nominal savings:** Option A reaches the target, with a heavy margin of 1.600 kg against 1.5 kg.
- **At conservative savings:** neither package fully reaches the target. The best is 1.430 kg (Option B).
- **To close that last 0.07–0.22 kg:**
  - Itemise the electrical and joining allowances (E1, J1). This is the cheapest route.
  - Or weigh the clutch and the estimated pan purchases.
  - Do not take more material from the structure.
- **On the stacked ledger:** no non-harmful package reaches 1.5 kg.

The plugin's `assembly-accounting` check reproduces these figures. It reports a margin of [1.280, 3.056] kg for Option A, [1.430, 3.232] kg for Option B and [0.193, 3.232] kg for the stacked ledger with every physical saving. Coverage is partial, so the check gives no configuration margin. The limit is pinned to this study's data file, because the season rule artifact has no R103 entry.

## 6. Follow-up

**Native Fusion**
- Pocket C2 and C3 on a new chassis issue. Then reopen it cold, take a census and a STEP export, and re-measure the mass.
- Design the P3 post-to-spine attachment.

**Offline checks**
- Rerun the deck register, key backing and brace clamp checks.
- Check the P3 attachment and the inlet portal end clips.
- Check the F1 fastener stack.

**Simulation**
- The package moves the field mass to about 63.6–65.1 kg. That is below the replayed 66.5–68.5 kg band.
- Replay `limited-field` at the light end, covering traction, pipe contact and reset.
- No moving-pan property changes, so the package itself needs no new pan replay. The r3 12-case replay is still owed independently.

**Records**
- Freeze or itemise the allowances (AF4).
- Name the battery retention and the bumper mounts (AF7).
- Update the commissioning figures (AF6).
- Weigh the 11 estimated pan purchases.
