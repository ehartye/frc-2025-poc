# Balance the straight-guide robot on its station approach

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

**The 44% lighter-side target is achievable under a specific prebuild mass allocation.** With opposite wheels matched within 20 g per axle, opposite gearboxes within 25 g, the battery construction bound below, and the remaining material centered within Y±10 mm, the lighter side retains at least **44.239%** across both signed approaches. The upper empty-mass endpoint alone gives **45.180%**. The installed bill still leaves enough mass without individual positions to prevent certifying the 44% target from CAD alone. [Student experiment](index.html) · [Signed load-line calculations](transfer.json)

## Where is the weight?

The calculation uses the selected red, fixed-height straight-guide configuration, stowed for the 21.10 s opening. The historic 401-body place STEP and both poses of the 991-body installed optical/receiver scene are reopened. A carried mass row must keep its physical identity, volume, centroid and extents after translation into the stowed position. The grouped electrical rows require all their original bodies. Later named replacements and the repacked chains are excluded from historical transfer. Five checked STEP exchanges supply the final deck ties, mast feet, carriage brackets and match-drilled straight-guide hardware. The installed accounting check resolves 1,189 selected physical identities. [Inherited parcels](parcels.json) · [Shop exchange parcels](kits.json)

Together **441 disjoint parcels** carry 31.028–38.140 kg across the empty field-mass interval of 46.186–65.464 kg. Each bought item can place its mass anywhere in the registered envelope. For the new manufactured uniform-stock parts, the design allows a 3% volume error, permits separately added and missing material when bounding the centroid, and uses explicit conservative density ranges. The earliest deck uses its own signed machining allowance. The remaining mass ranges from 8.046 to 34.436 kg in the enclosing calculation. Its centroid is a design allocation for the combined unregistered articles, remnant budgets, harness and other allowances. The registered material and the residual always sum to the chosen field mass.

| Prebuild case | Guaranteed lighter-side fraction across the tested opening |
|---|---:|
| Centered remainder, battery anywhere inside its case, independent purchased masses | 43.666% |
| Same parts, remainder within Y±20 mm | 42.747% |
| Same battery envelope, wheel and gearbox masses paired, remainder Y±10 mm | 43.691% |
| Pairs, battery density bound and remainder Y±10 mm | **44.239%** |
| Same candidate, remainder allowed Y±30 mm | 43.321% |

The comparisons use residual X±60 mm and Z150–450 mm. At those X and Z bounds the candidate supports a residual lateral centroid from **−15.50 through +18.29 mm**. The broader 850 × 850 × 1100 mm gross enclosure cannot even guarantee two-side support. In the centered original case, a single consistent choice of parcel positions and masses gives **43.944%** at 10.920 s with the left signed wheel error. In the ±30 mm candidate, a consistent 43.585% case also falls below target. These are points within the stated uncertainty model. The true mass placement of a manufactured robot is unknown.

At the upper field mass, assigning 1.813 kg of possible inherited stock and through-sets to the seven historical mounting locations gives 45.008% for the same matched configuration and ±10 mm allocation of the rest. This separate sensitivity expands each stock box by 1 mm and each fastening site by 65 mm for its associated hardware. The 1.813 kg comes out of the same residual budget. The calculation authorizes no installation of these optional parts. [Seven source locations](../winchless-bill/stock.json)

## Why the battery matters

The 6.17 kg NP18-12 has an installed 76 × 181 × 167 mm case. The EnerSys family material sheet describes approximately 10–30% electrolyte by weight, with specific gravity up to 1.350. For the candidate simulation use at least 10% electrolyte, density ≤1.35 g/cm³, and ≤12 g/cm³ for the remainder. Lead and lead dioxide reference densities are about 11.34 and 9.38 g/cm³. The upper density is a design assumption for all constituents. The data sheet allows other ingredients. [EnerSys NP family and material sheet](https://www.enersys.com/493bb4/globalassets/documents/corporate/sds-or-msds/amer/sds-853024.pdf) · [Original procurement reference](../design/electrical-inspection/SOURCES.md) · [Lead](https://pubchem.ncbi.nlm.nih.gov/compound/lead) · [Lead dioxide](https://pubchem.ncbi.nlm.nih.gov/compound/Lead-dioxide)

A limiting arrangement fills the entire side face with the densest permissible material, followed by electrolyte. For mass *m*, section area *A*, electrolyte fraction *f* and density limits *ρh* and *ρe*, the inward centroid distance is at least *m/(2A) × [(1−f²)/ρh + f²/ρe]*. The result places the battery center at least **21.85 mm inward from both Y faces**. It uses the full case area, even where the case and voids occupy volume. Doubling the non-electrolyte limit to 24 g/cm³ leaves an unpaired centered-remainder result of only 43.962%. No cell layout or alteration of the battery is required.

## How to select six wheels

Simulate the six purchased wheels at an effective radius within ±0.1524 mm of nominal and independent vertical rates within 90–110% of reference. Choose the largest effective radius for the center axle **on each side**, while keeping opposite wheel masses within 20 g. The reference settling distance under six equal wheels must be at least 1 mm at the simulated empty weight. The effective gap includes axle height, local floor and wheel radius. Sorting radii alone cannot correct an unknown mounting-height error. Preserve the side-specific rolling calibration used in the station trajectory when selecting a wheel set. [Example wheel assignment tool](assign.py)

For every admitted CG on the paired-and-packed opening, the longitudinal load line lies within ±118.681 mm and the lateral one within ±33.414 mm. Over that entire rectangle a unilateral linear-program check allows individual wheel rates, all height errors, all contact sets and loss of contact. It constrains force and both moments. The single-body dynamic model uses both signed 5 ms clock derivatives and the actual body-frame turn acceleration at the modeled CG, including simultaneous yaw acceleration and centripetal terms. It permits both adjacent velocity differences at a command edge. Chassis suspension transients, changing floor height and robot contact remain outside these opening cases.

| Installed wheel selection | Maximum combined end-axle load |
|---|---:|
| Unsorted radii, rates 90–110% | 77.457% |
| Largest wheels centered, rates 90–110% | **70.968%** |
| Largest wheels centered, rates 75–125% | 76.923% |

The allowed centered/rate-matched case covers the selected 75% ceiling used by the earlier electrical screen. The solver records equilibrium witnesses, including cases with only five contacts. The wider-rate control can have four. A consistent six-wheel sample gives 7 g maximum axle mismatch. Swapping the two sample radius values in the regression control leaves no assignment satisfying both mass and height ordering. [Active contact sets](order.json) · [Earlier electrical replay](../wheel-loads/README.md)

## Reproduce

Use the project’s CAD/vision environment and Agent FRC assembly-accounting CLI. The numerical contact optimization also needs NumPy and SciPy with HiGHS. Generate the source map and optical scene using the linked predecessor records. Supply the privately held lamp CAD to the local scene build.

```text
python lateral-balance/native_parcels.py ARTIFACTS LAMP_STEP STRAIGHT_NO_CUT_MAP
python lateral-balance/kits.py PLUGIN STRAIGHT_NO_CUT_MAP
python lateral-balance/transfer.py ARTIFACTS
python lateral-balance/active_sets.py
python lateral-balance/viewer.py
python lateral-balance/assign.py --example
python -m unittest discover -s tests -p test_lateral_balance.py
```

An optional six-row JSON file for `assign.py --wheels FILE` uses `id`, `kg`, `radiusErrorMm` and `rateRatio`. The case remains a desktop prebuild qualification. Commissioning can check component pairing, loaded wheel radii, vertical rates, retained stock, residual mass placement and six simultaneous normal loads.
