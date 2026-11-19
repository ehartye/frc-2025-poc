# Join the four primary chassis rails

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

The candidate uses the four existing aluminum tubes. Each crossrail gets a prepared end on three faces and an interior fillet on the fourth. On the selected shop and camera configuration, the four joints add **3.69 g nominal net metal**. Reserving **20 g** for all four completed welds leaves **65.08 g** against the represented mass upper bound. The assembly still has unweighed items. This study issues a shop trial and records the remaining acceptance measurements. [Dimensioned sheet](frame-sheet.svg) · [Reopened STEP](frame-weldment.step) · [Installed accounting](accounting.json)

## Drawing and fixture traveler

1. Obtain four tubes of documented weldable alloy and temper. The proposed stock is 6061-T6, with measured 2 mm walls. The 700 mm side members have 25 × 100 sections and the 650 mm end members have 20 × 100 sections. Check the actual extrusion radii, end squareness and root fit at each seam. Record the chosen filler, lot, procedure and expected as-welded properties with the fabricator. The published hardness profile of welded 6061 shows why the mill temper alone cannot establish joint strength. [Manufacturer guide, figure 2-1](https://ch-delivery.lincolnelectric.com/api/public/content/3353e67959b74173b336498aface1f3f?v=d5d4eabf)
2. Locate the crossrail inner toes at |X|330, the outside ends at |X|350 and side-tube inner faces at |Y|325. Prepare the crossrail only. The two horizontal end edges and the exterior vertical end edge carry a 1.2 × 45° single bevel and nominal 0.8 mm land. The native model restores the cut volume once. Specify full fusion through the thin wall on those seams, a 2.5 mm equal-leg inside fillet, and at most 0.60 mm reinforcement above and below. Dress the exterior vertical seam flush and verify the remaining section on the coupon. Count four joints, one per corner. The four colored or individually named metal regions in each CAD joint describe one fabrication operation.
3. Support the bare frame on a flat, open strongback with three coplanar pads. Use pad centers (−200,337.5), (200,337.5) and (0,−337.5) in robot X,Y mm. Locate the +Y exterior against two removable buttons at X±180 and the +X exterior against one at Y0. Fit light opposing clamps and temporary midspan supports to measured flats. Keep access to all four corners. Prove the fixture and a means of turning/supporting the assembly on the bare-stock trial. Measure both outside spans against the 700 maximum and record both diagonals against the nominal 989.95 mm. An equal pair of diagonals alone cannot establish width, twist or axle alignment.
4. Have the responsible fabricator qualify the process on matching cutoffs, including rolled corners, fit-up, opposite-face restraint, venting and flush dressing. Confirm an open vent path in each hollow member during joining. Tack in the fixture, check squareness, and balance heat between opposing corners using the qualified sequence. Allow cooling and release the clamps before measuring residual distortion. Inspect root continuity, section, porosity and heat-affected material on the coupon. Record actual section and measured strength on the selected material and procedure.
5. Gauge the 700 mm frame in both axes, opposing-face parallelism over each carrier span (0.15 mm), paired bearing axes (Ø0.10 over 90 mm), and free shaft rotation at center and both ±4 mm axle adjustments. Use dye on the bumper contacts, mount both alliance kits, check the 850 mm outside gauge and remove both U sections through 440 mm. Recheck these dimensions after the representative contact trial. Weigh all bare rails together before preparation and the joined frame after dressing on the same calibrated scale. Return the actual mass and joint proof results to the build record. [Existing rail tolerances](../design/shop-package/README.md) · [Bumper contact and withdrawal procedure](../design/bumper-kit/README.md)

## Clearance, contact and load

The source is the registered 603-body shop assembly. Twenty project-owned solids reopened, comprising four primary rails and sixteen weld-metal regions. All four corners clear the tire by at least 19.66 mm across ±4 mm adjustment, including a 0.25 mm allowance between native samples. The conservative moving-body boxes have at least 196.61 mm clearance. Both entire bumper withdrawal gauges have zero common volume with every weld. The flush outside seam touches the end contact strip as designed. A 0.60 mm proud face on a 4 × 80 mm patch penetrates that strip by **48 mm³** at each corner. [Per-corner witnesses and model hashes](weld-fit.json)

The elastic line-group screen places the entire horizontal reference force at one bumper corner. It distributes direct shear, normal force, in-plane torsion and out-of-plane bending over a 20 × 100 mm rectangular perimeter. It checks every horizontal heading and all vertices of the assumed load-position box, |X| and |Y| 350–425 and Z42.5–181.5. Stress varies linearly over a section and the stress norm is convex in position for a fixed heading. The corner extrema therefore bound the stated continuous envelope. The singular value of the heading-to-stress matrix gives the worst horizontal heading. Reactions elsewhere in the chassis, local wall buckling, fatigue, residual stress and contact shock require separate evidence.

| Horizontal force on one corner | Assumed effective throat | Maximum equivalent elastic demand |
|---:|---:|---:|
| 1 kN | 1.4 mm | 56.04 MPa |
| 1 kN | 1.0 mm | 78.45 MPa |
| 2 kN | 1.4 mm | 112.07 MPa |
| 2 kN | 1.0 mm | 156.90 MPa |

These forces are design scenarios. The group model assumes that the whole perimeter transfers load. Have the fabricator verify that assumption after dressing. Use a representative corner and bumper/load fixture, record force versus deflection and residual squareness, and inspect both the weld and thin parent wall. Agree allowable permanent deformation, proof factor and repeated-contact conditions before authorizing production. Record any actual contact pulse before converting a match impact into a peak load. [Calculation and limiting headings](load-demand.json)

## Rejected layouts

| Layout | Measured result | Decision |
|---|---|---|
| Low interior splice | One tire overlaps the unfastened corner by 73.91 mm³ at outward take-up. | Reject. |
| Raised interior splice | Wheel clearance exceeds 4.97 mm after accounting for the full adjustment. The bare-frame puller envelope clears by only 0.21 mm. | Insufficient installation allowance. |
| External upper and lower plates | 80 rivets and eight plates. After deck relief the plates clear the deck by 2 mm. Outward take-up produces 2.87 mm³ of tire interference. Net machined aluminum adds 316.71 g before any rivet. | Exceeds the available mass and intersects the wheel. |

The original interior and plate studies can be repeated with the same shop input. The manufacturer specifies hole and grip ranges. Its rivet coupon ratings require a separate check on the 2 mm tube wall. [Fastener drawing](https://www.cherryaerospace.com/docs/stdpg/cmax/CR3213.pdf) · [Installation catalog](https://www.cherryaerospace.com/docs/catalogs/CA-1011.pdf) [Interior control](inside-control.json) · [Plate control](plate-control.json)

## Reproduce

Use the CAD-capable project interpreter and the installed Agent FRC plugin. The artifact root must contain the selected `camera-datum-shop/place.step` and the camera installation map generated by the [camera disconnect study](../camera-disconnects/README.md).

```text
python frame-corners/weld.py <artifacts> <output>
python frame-corners/loads.py
python frame-corners/account.py <installed-plugin> <camera-selected-map> <map-output-directory>
python frame-corners/geometry.py <artifacts> <interior-control-output>
python frame-corners/plates.py <artifacts> <external-plate-control-output>
```

The joined-frame map counts the four welded joints as made components and the pre-existing rails once. The purchase requirement is traceable filler compatible with the qualified process, local fixture materials, coupon stock and fabrication labor. Obtain an actual quote and assign labor after the trial. The mass range is conditional on the 20 g fabrication allocation. The installed checker retains the existing omissions and reports an unknown physical inspection margin. [Mass input](weld-mass.json)
