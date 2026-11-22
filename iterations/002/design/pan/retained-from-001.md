# Pan elements retained from iteration 001 (revision r3)

`commissioning.md:3` names the 430 × 322 mm pan-floor profile as the only mechanical reference imported from iteration 001. This proposal retains more than that: the pan contact surfaces, the pivot axis and the drive. `frc-start-over` requires each manually retained element to have a stated reason and a recheck against its new surroundings, so each one is justified below.

## Decision

**Decision (recorded r3):** the owner delegated design decisions on mechanical viability. Under that delegation, retaining the 001 pan contact surfaces, axis and drive, beyond the floor profile named in `commissioning.md:3`, is **accepted**.

**Evidence the decision rests on:**

1. Every iteration-002 custody result was computed on the 001 shop stock. `strategy/limited-pan-12mm.json` pins `historicalStockSha256 = 0b743fed…`, which is `artifacts/red2-integrated/pan-mechanics.json`. That stock passed 12/12 on the 002 chassis clocks (four-terminal and low-line).
2. `pan-stop-01/pan-stop-study.json` adds the stow stop and closing preload on the same surfaces and also passes 12/12, at the 30 A limit and unpowered. `pan-stop-study-04.json` replays the r2 ladder properties on the same surfaces, axis and drive (P4-K6000-limit passes every criterion).
3. `fixed-path-01/comparison-02.json` returns **FAIL** for every fixed-path variant. No alternative without a moving pan, built on other surfaces, has passed.

A different contact surface, axis or drive would invalidate items 1 and 2 and needs its own 12-case evidence first.

## What r3 keeps exactly

The floor top, the cheek inner faces, the lip front faces, the pivot (205, y, 580), the feed and guide contact faces and the M1 stop contact. The joining and structure change. The changed structure includes the riveted ladder, the plate-stack posts and lugs, the bulkhead and its 1/4 in clips, and the limit cam. Those changes alter the moving properties:

| Quantity | Simulation basis (001 stock) | r2, replayed in pan-stop-study-04 | r3 moving pan with joining hardware | r3 against the sim basis |
| --- | --- | --- | --- | --- |
| Mass | 3.3452 kg | 3.5106 kg | 3.6455 kg | +9.0 % |
| First moment u | 400.49 kg·mm | 305.62 kg·mm | 309.22 kg·mm | −22.8 % |
| First moment w | 148.83 kg·mm | 114.30 kg·mm | 106.25 kg·mm | −28.6 % |
| Inertia about the pan axis | 0.11503 kg·m² | 0.09435 kg·m² | 0.09488 kg·m² | −17.5 % |
| Gravity arm at stow | −0.86 mm | −0.82 mm | **+1.62 mm** | sign change |

On-axis additions are 28.5 g. Against the study-04 replay basis, r3 changes mass by +3.8 %, u by +1.2 %, w by −7.0 % and inertia by +0.6 %. The stow gravity arm changes sign: the empty pan now tends to open at stow with about 0.06 N·m, against the 1.5 N·m closing spring.

**A fresh 12-case replay with these properties is required.**

## Element by element

| Element | Retained what | Why it is needed | What changes in r3 |
| --- | --- | --- | --- |
| 430 mm pan floor | Exact section | Named in `commissioning.md:3`. It is the custody surface (`closestMember: pan_floor`). | Riveted to six floor clips per side. Rivets outside the pipe footprint have dome heads. Rivets under the footprint are 100° countersunk rivets in dimpled 2 mm sheet, so the top face stays flush with no knife-edge countersink. |
| Cheek inner faces (001: welded ladder of 28 parts) | The guide-face plane, the 120 mm height and the stringer bands | They are the moving guide (685 N) that the sim's custody depends on. | A riveted ladder replaces the welded one. It has a 1/8 in skin, 3/4 in tube stringers and three 1/8 in-wall web tubes. Waterjet socket-post plate stacks and a 3/16 in pivot bulkhead with 1/4 in clips carry the root moment. |
| Slotted rear lips | Exact sections | They form the custody V at stow. | Screwed to the stop-lug plate stack and riveted to a dimpled root clip bent on a hand brake (`fabrication-plan.json`). The stop no longer bears on the 2 mm lip, which would be about 24× too soft. |
| Pivot axis (205, y, 580), hex sockets, idler stub | Axis and hex location | The sim and the 001 drive are both built on this axis. | The sockets become waterjet plate stacks with a cut hex. The idler stub is lengthened to Y = −229 for the outboard thrust stack and torsion spring. |
| Bearing planes Y = ±195, bore r 14.2875, drive datum Y = 216, six-hole gearbox pattern | Locations and hole patterns | The journal reactions and the 54.83:1 drive basis assume them. | Carriers and a gearbox plate land them on the new 1 in cradles (OD-1 A). Axial location is on the starboard side only. The slip clutch will stand the gearbox plate off by its own length, keeping the pattern. |
| Purchased drive and bearings (REV-21-1651, REV-41-160x/1615/1620, REV-21-1915 ×2, 6655K17 ×2) | Identity and configuration | The current-limited replays use them. | Seats only, plus the required slip clutch and the reverse-limit cam and switch. |
| Fixed feed panels, inlet guides, spines and chute crossmembers | The article-contact faces | They are the sim feed polyline and entry guides. | r3 applies P1 and P2 with the contact faces unchanged. The guides go from 3 to 2 mm and the panels from 2.0 to 1.6 mm. The spines, post tips and crossmember tops follow the moved back faces. |
| Fixed inlet posts and feet | Sections (post tips moved 1 mm under P1) | **Reason:** the posts hold the guide spines, and so the inlet guides, at the sim feed geometry. No other member locates the spines in the 002 layout, and replacing them would move or re-qualify the guide faces. | They land on new upper-stringer extensions (OD-2). The feet are raised 9.525 mm and the post bottoms trimmed to match. P3 (tube posts) is not applied because the post-to-spine attachment is not designed. |

## Not retained

- The 001 chassis, crossframe, towers, feet, knees and foundation.
- The 001 native archives as authority.
