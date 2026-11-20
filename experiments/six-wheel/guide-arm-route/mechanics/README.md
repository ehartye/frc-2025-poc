# Returned guide-arm mechanics

The closed front return preserves the two guide attachments at Z687 and Z787 mm. Its positive-Y lower spring is 11,721 N/m in this model. The opposite lower spring is 32,177 N/m. The first two assembled guide frequencies are 72.20 and 110.41 Hz on the modified side and 109.19 and 123.82 Hz on the opposite side. Each value uses the nominal aluminum sections, installed part volumes and the earlier 23.6% arm participation assumption. The physical modal frequencies are open.

## What the static screen resolves

The unit load acts laterally at the backing screw-pair center. Each straight portion uses the unit-load bending and torsional moments projected onto the member's actual centerline. The high arm rises 73 mm over 71 mm of X travel. Its projected wall and section depth reflect the extrusion of a vertical section, and the returned box follows the native 0.46 X/Z taper. Closed-wall torsion uses the thin-wall shear-flow formula. The opposing mast bolts and folded tab retain the earlier root model. The Z760 attachment also introduces a 73 mm roll lever on its root bolt group. The small lower nose transfer and fused junctions are treated as rigid. The model omits local shell distortion, seam rotation, looseness and joint slip.

| Lower arm | Unit tip travel (mm/N) |
| --- | ---: |
| Straight original | 0.03108 |
| Direct raised attachment at Z760 | 0.07743 |
| Deep closed return to Z687 | 0.08152 |
| Central closed return to Z687 | 0.08316 |
| Front closed return to Z687 | 0.08532 |

The folded front return stays below 0.105 mm of predicted static travel at the normal-pressure centroid of either original signed pipe arrival. On the two later serial-carriage histories the full distributed lateral loads give peak work-weighted face travel of 0.06637 and 0.09706 mm. Moving the attachment to Z760 increases those to 0.93434 and 0.92307 mm. With an infinitely stiff backing and that same raised attachment, the modeled lower/upper arm translations alone give 0.86825 and 0.83630 mm. Both exceed the nominal 0.30 mm per-side interference. These point and distributed values measure travel at the loading location. The successor replay also tracks each free tip between contacts.

The unloaded free tip can move farther than the loaded area. Across the complete positive-Y face and both serial-carriage histories, the closed return reaches 0.09608 and 0.11972 mm. The raised direct attachment reaches 1.48709 and 1.40306 mm at its free lower tip. These bounds include the curvature allowance between the 1 mm sampling stations. The opposite guide retains its original attachments and reaches 0.09438 and 0.05042 mm.

The front return spends 0.04328 mm/N in torsion of the rising tube and 0.00786 mm/N in torsion of the level portion. Opening the torsion cells along their length removes their closed-section shear path. This adverse calculation reaches the nominal allowance early and then leaves the small-deflection regime. Its extrapolated large travel only identifies the failure mechanism. The same fully open-wall control is retained for all four alternatives. [Static receipt](screen.json) · [Distributed contact receipt](distributed.json)

## Simulated station custody

The left signed arrival first reached Java `IN_BAND` at 1,340 ms, and the right reached it at 1,360 ms. Each held through 5,460 ms with the belts stopped on both the 31.25 and 15.625 µs steps. The maximum absolute travel anywhere along either guide was 0.12103 mm across the four runs. Reconstructing the contact on the saved pipe trajectories recovered every active guide state with a maximum normal-force discrepancy of 5.2 × 10⁻¹⁵ N.

| Signed arrival | Fine-step peak face travel at −Y / +Y (mm) | Terminal pipe-center separation between steps (mm) | Terminal pipe-axis difference (degrees) |
| --- | ---: | ---: | ---: |
| Smaller left wheel | 0.10116 / 0.08848 | 0.33775 | 0.27437 |
| Smaller right wheel | 0.04718 / 0.12095 | 0.05119 | 0.03221 |

The 0.274° left-side difference matters for a camera looking along the pipe. Each of the four runs supplies its own camera poses and an independent mechanical restart. The six scheduled image times are Java 5,372, 5,389, 5,406, 5,423, 5,440 and 5,457 ms. Simulated port acceptance makes those poses eligible for separate optical evaluation. This study ends at the station and leaves loaded driving, scoring and physical custody open. [Four mechanical trials and terminal states](joined.json) · [Exact camera poses](station-frames.json)

## Reproduction

Use Python with NumPy and the Python dependencies of the source project's guide-impact study. The trial needs a JDK 17 directory. The mechanics directory supplies `mechanics.py` and `inputs.json`. In these examples, `PROJECT` points to the source checkout, `PREVIOUS` to the original guide-impact trial directories, `ROUTES` to the generated CAD alternatives, and `OUT` to a writable output location. The selected `ROUTES/front-return` directory contains `route.step` and `revision.json`. Keep a single BLAS thread for numerical repeatability.

```text
python mechanics.py --project PROJECT --out OUT/static static --raw PREVIOUS --routes ROUTES
python mechanics.py --project PROJECT --out OUT/static distributed --raw PREVIOUS
python mechanics.py --project PROJECT --out OUT/trials trial --case leftSmall --step 31.25 --configuration ROUTES/front-return --jdk JDK17
python mechanics.py --project PROJECT --out OUT/trials trial --case rightSmall --step 31.25 --configuration ROUTES/front-return --jdk JDK17
python mechanics.py --project PROJECT --out OUT/trials trial --case leftSmall --step 15.625 --configuration ROUTES/front-return --jdk JDK17
python mechanics.py --project PROJECT --out OUT/trials trial --case rightSmall --step 15.625 --configuration ROUTES/front-return --jdk JDK17
python mechanics.py --project PROJECT --out OUT/reopened handoff --raw OUT/trials --configuration ROUTES/front-return
```

The successor starts with both signed powered approaches, the two serial carriage stages, the machined shelf, finite belt and annular guide contact, and the recovered-edge Java program from the source register. Each side receives its correct machined arm mass. The full parent finite element basis has 50 coordinates per side. The solver updates its mass and stiffness matrices before retaining the first five modes under 1,800 Hz. It records pipe and guide states, contact work, port transitions and both free-tip displacements. The handoff independently reconstructs the local guide loads and all modal coordinates, checks terminal state consistency, and requires both wheel cases at both time steps. It exports the six exposure times for cases meeting the modeled Java custody requirement plus two time-window endpoints. Camera visibility must be evaluated on the returned arm itself.

## Physical qualification

Gauge the positive-Y lower arm at the mast tab, upper junction, lower nose and both ends of the installed backing to distinguish root motion, weld or seam rotation, and overhang. Reverse a measured lateral load through the modeled lower reaction range near 0.75 N, and check the upper joint through approximately 2.25 N. These figures are loads reached by the original arrival traces and do not set a proof load or fabrication rating. Check return-to-zero, bolt seating, the as-clamped throat, local overhang compliance and ringdown. Use both signed approach offsets with the actual liner and pipe, moving carriage, port sensors and camera finishes before claiming repeatable capture.
