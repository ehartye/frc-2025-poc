# Red 2 iteration 002: scoring architecture evaluation (r2)

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 83b3b9c7fe](../../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

Revision 2 answers the adversarial review of r1, which is commit `66748a4`. It is an offline preliminary comparison: it did not use Fusion and changed nothing in `preliminary/`, `pan/` or `chassis/`. Every number and input hash is in `evaluation.json`. To reproduce, run `python -I -B evaluate.py`; it takes about 10 s.

**The comparison is asymmetric.** A's custody and delivery come from a spatial 12-case field replay that includes peers, the match clock, the current limit and the finite trough panels. B–D come from planar screens and bottom-up mass estimates. A's evidence is deeper. B's advantages are predictions, and the bench gate below must confirm them.

## Role and requirements, kept separate from the solution

The mission is two TELEOP trips in match seconds 18–153. On each trip the robot receives one station-issued CORAL (a 4.5 in OD pipe of up to 0.816 kg) and places it in red L1 trough sector A, then sector B. It keeps the piece credited and returns to a ready, serviceable state. Sources: `commissioning.md` and the wiki page `2025-poc/reference/case-2025.md:19`. Adoption of the role is still unrecorded.

| ID | Requirement |
| --- | --- |
| R1 receive | Accept a pipe arriving at up to 2.47 m/s, rolling axis-across. |
| R2 carry | Keep custody through route accelerations, 3–8 g pulses and bumper strikes. |
| R3 deliver | Leave the pipe supported in the trough (lip top 457.2 mm, 15° floor, riser 518 mm) across the real dock scatter. |
| R4 reset | Be ready again within the trip. |
| R5 fit | R103 mass. The R101–R104 perimeter and the 42 in start height. The R102/R105 18 in extension limit. Weak-pack bus at 9 V or more. Rule source: `experiments/six-wheel/kickoff/rule-register-v0.json`. |
| R6 service | Serviceable in the pits by a student shop. |

**Rules.**
- B launches the pipe, which is airborne for about 0.1 s. Under **G412**, a launched CORAL needs a bumper partially in our reef zone, so the eject command must be interlocked on the robot's reef-zone pose. The register marks G412 as unmodeled.
- It is an open rules question whether B′x's roll-off counts as a launch.
- B′x's deployed lip and A's 179 mm dumped reach both fall under R105.

## The fixed-path test was not fair

`fixed-path-01/comparison-02.json` froze the pan at the −8° dump pose. It had a 14 mm lip and no actuator, and it released only through a scripted 9 m/s² stop. The arrival carries 42 times the energy needed to roll over the lip.

| Variants | Result |
| --- | --- |
| Undamped, c6, c10 | Lost the pipe at receive in 12/12 cases. |
| Notch c15 | Lost it in adverse carry in 6/12 and while parked before release in 6/12. |
| Notch c22, floor-brush c15 and c22 | Lost it in adverse carry in 6/12. In the other 6/12 the pipe was kept but could not be released. |
| Floor-brush c10 | Lost it in carry in 12/12. |

The test rejects a frozen pan with no actuator. It does not test a fixed pocket with a powered release.

**About the KitBot citation.** The wiki passage (`reference/ingestion-coverage-2026-11-16.md:63`) supports only the role: the KitBot is a Coral Station-fed Level 1 scorer. It does not support any particular exit mechanism.

## Architectures

| | Description |
| --- | --- |
| **A** | Guarded tipping pan, r3. |
| **B** | Fixed pocket with its centre at 0.60 m. A 70 mm fence leans 20° toward the pocket and has a low-friction face on compliant backing. Above it sit a 10 mm hood and a two-roller nip driven by one NEO 550 at 12:1. The real exit is at **0.67 m**. The nip gives the pipe no spin. |
| **B′0** | B's pocket and nip, but the pipe rolls off a short exit lip at the frame edge (KitBot-style placement), at low speed with rolling spin. |
| **B′x** | Like B′0, but the lip deploys once at match start so the pipe leaves 100 mm beyond the frame line, at 0.55 m. |
| **C** | B's pocket with a passive gate tripped by contact with the reef. |
| **D** | Pivoting roller end effector. |

## Delivery screen (`delivery.py`)

The screen is a planar rigid disc that makes impulse contacts with restitution e (0.2 and 0.5) and Coulomb friction μ (0.15 and 0.35). It tracks spin and contacts the lip faces, the floor, the riser face and the riser top, so bounces near the 46 mm riser are simulated. It sweeps launch angle (−10°, 0°, +10°) and stand-off (0–100 mm). In the results below, "robust" means every stand-off is captured under every combination of e and μ.

| Option | Speed band that captures every stand-off | Notes |
| --- | --- | --- |
| B at 0.67 m, 0° | **1.1–1.5 m/s (±15 %)** | At 1.9 m/s only 5 of 11 stand-offs are robust; at 1.9 m/s and +10°, 4 of 11. Above about 1.6 m/s the pipe goes over the riser. |
| B′0 | ±5 % at best | Falls short of the lip. It does **not** remove the stand-off sensitivity. |
| B′x at 0.55 m, 0° | **0.6–1.5 m/s (±43 %)** | 0.7–1.4 m/s at −15°. Widest window of all. |
| C | 1.14 m/s at most, from gravity with no losses | Marginal. |

The screen does not cover yaw, contact at the pipe ends, ovality and wear, measured e and μ, or slip at the nip.

## Launch speed and voltage (`budgets.py`)

The NEO 550 at 12:1 gives a free surface speed of 2.44 m/s at 12 V, 2.09 m/s at 10.3 V and 1.83 m/s at 9 V.

- **B** needs 1.3 m/s ± 15 %. Running at a fixed duty alone spreads the speed by about ±11 % between 10 and 12.5 V, before slip and load are counted. B therefore needs:
  - closed-loop roller velocity from the NEO 550 encoder, held to ±5 %;
  - an eject interlock below 9.5 V.
- B's headroom is 1.56× at 10 V.
- **B′x** tolerates open-loop duty.

## Receive (`custody.py`)

The arrival is 2.85 m/s and 6.26 J, and **47 % of that energy is spin**.

- **Vertical fence.** Friction turns the spin into upward motion. With μ = 0.35 the pipe rises **73–92 mm**, which is enough to clear the 70 mm fence and enter the nip. A foam face (μ about 0.6) is worse.
- **Fence leaning 20° toward the pocket.** With μ of 0.35 or less, the vertical velocity after impact is below zero, so the pipe does not rise. The fence must therefore have a low-friction face, not foam.
- **Back-driving the nip.**
  - A stalled 12:1 NEO 550 in brake mode resists about 79 N at the rim at 0.5 m/s and about 158 N at 1 m/s. A pipe that reaches the nip stops within about 2 mm (estimate from catalogue Kt and R), so it cannot drive through.
  - Brake mode needs a powered controller. Unpowered custody therefore has to rest on the leaning fence and the hood.
- **Check flap.** It must close again within 38–77 ms in a 150 mm pocket and within 91–185 ms in a 200 mm pocket. Use the 200 mm pocket.

## Comparison

| | A pan | B | B′x | C | D |
| --- | --- | --- | --- | --- | --- |
| Custody | Needs power. Lost at ≥ 3 g unpowered. | Held by geometry if the fence leans 20° with μ ≤ 0.35. | As B | Geometric until a false trip | Held arm |
| Delivery | 1.83 mm margin; a 12 mm dock change flips the result | ±15 % speed band | ±43 % speed band | Marginal | Not screened |
| R103 margin | +0.60 kg (recorded) | +5.54 kg (estimate) | +5.18 kg | +6.36 kg | +4.66 kg |
| Peak current / weak bus | 30 A / 8.81 V (impact) | 25 A / 9.29 V | 25 A / 9.29 V | 0 A / 9.29 V | 50 A / 8.97 V |
| Governing load | 1710 N per pad; 41 N·m gearbox, so a clutch is required | About 500 N at the fence | As B, plus the lip tip | As B | Same class as A |
| Mechanism time | About 3.5 s | About 0.5 s | About 0.55 s | About 0.9 s | About 1.3 s |
| Fabricated / purchased | 136 / 357 | About 16 / 70 | About 19 / 80 | About 16 / 40 | About 26 / 110 |
| Evidence | Spatial replays; no physical test | Planar screens | Planar screens | Planar screens | General practice |

## Options against the owner's five dimensions (1 = best of 5)

- **A. Guarded tipping pan**
  - Time: 4. It still needs the replay, the mass package, the clutch, a latency measurement and an encoder.
  - Risk: 3. Its evidence is the deepest, but its margins are knife-edge and it holds the pipe only when powered.
  - Complexity: 5.
  - Best practice: 4.
  - Maintainability: 5.
- **B. Low-speed launch**
  - Time: 1.
  - Risk: 2. The speed band is narrow and needs closed-loop control. G412 needs an interlock. Receive with spin has not been tested.
  - Complexity: 2.
  - Best practice: 3. A ballistic launch into L1 is less usual than placement.
  - Maintainability: 1.
- **B′x. Deployed-lip roll-off**
  - Time: 2.
  - Risk: 1. It has the widest window and places the pipe rather than throwing it. It adds a deploy latch and an R105 check.
  - Complexity: 3.
  - Best practice: 1. Roll-off placement is the usual L1 form. This is my judgement; the wiki does not verify it.
  - Maintainability: 2.
- **C. Passive reef-tripped gate**
  - Time: 3.
  - Risk: 5. It can trip falsely and its gravity delivery is marginal.
  - Complexity: 1.
  - Best practice: 5.
  - Maintainability: 3.
- **D. Pivoting roller arm**
  - Time: 5.
  - Risk: 4. Not screened, and it brings back A's stop and clutch problem.
  - Complexity: 4.
  - Best practice: 2.
  - Maintainability: 4.

## Recommendation: the B family, confidence about 55 %

I recommend the B family: a fixed pocket with a powered nip. Take B′x if its deployed lip passes the bench; otherwise take B. My confidence is about 55 % that the B family serves the role better than A, and low-to-medium that it passes the gate first time.

**Do NOT discard the r3 pan package until bench gates G1–G4 pass.** If they fail, A remains selected and the r3 fixes continue.

If the gates pass, the following Fusion rework applies.
- **Discard:** the moving pan, drive, clutch, cam and switch, spring, stops, root-moment joint, post stacks and bulkhead, and the C2/C3/P3/titanium mass work.
- **Change:** replace the inlet and upper support with the tray, side plates and uprights. Re-land the pan trusses. Keep the 430 × 322 mm profile as the pocket width.
- **Keep:** the lower chassis, modules, clamp screws, sills, electronics, and the receive and reef faces.

## Bench gate

**Test article.**
- A 70 mm fence leaning 20°, with a UHMW or polycarbonate face on 25 mm of compliant backing.
- A 10 mm hood, a 200 mm pocket and a spring check flap.
- A two-roller nip with 2 in 35A wheels at 12 mm compression, driven by a NEO 550 at 12:1 under closed-loop velocity control.
- For B′x: the deployed lip, with the pipe leaving 100 mm beyond the frame line at 0.55 m.
- Real pipes, both new and worn, at 0.82 kg.
- A trough mock built to `target.json`.

**Gates.**

| Gate | Test | Pass | Kill |
| --- | --- | --- | --- |
| **G1 receive** | 30 feeds at 2.85 ± 0.10 m/s, measured with light gates or 240 fps video, with the nip in brake mode. Then 10 more with the controller unpowered. | 30/30 seated within 0.5 s. No rise above 10 mm at the fence, no entry into the nip and no rearward escape. 10/10 unpowered. | Any climb over the fence. |
| **G2 eject** | Stand-offs of 0, 25, 50, 75 and 100 mm × loaded bus at 10.0, 11.0 and 12.0 V × 10 ejects = 150 per variant. Target 1.3 m/s for B, 1.0 m/s for B′x. | At least 147/150 captured and still supported 2 s later. Every cell at least 9/10. For B, measured exit speed inside 1.1–1.5 m/s in 150/150. | Any cell at 7/10 or below kills that variant. |
| **G3 sled** | Nip unpowered. Pulses of 1, 3 and 8 g for 50 ms, forward and rearward, 10 at each level. | 60/60 retained. | Any loss at 3 g or below. |
| **G4 end to end** | 20 consecutive cycles: feed, a 5 m carry ending in 1.5 m/s² braking, then eject at a random stand-off of 0–100 mm and a random 10–12 V. | At least 19/20 credited, with no hand recovery. | — |

**Decision rule.**
- All of G1–G4 pass: retire r3 and start Fusion work on the B family.
- G1 or G3 fails: keep A.
- G2 fails for B but passes for B′x: select B′x after a ruling on R105 and G412.
- Before shop release, run a fair spatial 12-case replay that includes a pocket-and-nip element.

## Known issues checked against the files

These are unchanged from r1:
- Confirmed: the +0.60 kg margin, 8.81 V (conservative pairing), lift-off above 3.57 m/s², the 1.5 N·m spring, the 15.4–21.7 N·m clutch band and the −3.5° inhibit.
- The 3.85 mm figure is soft-stop pad compression, not a cradle gap.
- I did not find the third review document.
