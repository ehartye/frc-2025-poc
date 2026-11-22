# What happens when the carriage flexes under the station feed?

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 83b3b9c7fe](../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

The two fixed brackets carry the receiver on four seats. An impact bends the front arms, lifts the nested rails off their lower stops, and moves both ends of the shock absorbers. This study couples the bracket geometry to the receiving-shelf model and runs the same desktop Java acquisition controller on the resulting moving-port observations. [Explore the contact histories](chart.html) · [Comparison](comparison.json) · [Shop geometry](../carriage-support/)

## Which robot and approach were tested?

The assembly has the absorber-drilled bridge, two fixed bracket noses at X167, two rear crossbar seats at X−87, the original Ø8 height pins at Y±222.5, and the fixed-low end-belt receiver. The original two signed station-drop inputs and the two powered docking poses have independent identities. Each uses the stowed receiver, the selected force law, a free cylinder and both finite shelf apertures. A success in this record means the Java acquisition state entered IN_BAND before the desktop deadline. Capture retention, vision-based custody, field travel, release and a scored placement require separate downstream evidence. [Original drop](../shelf-structure/) · [Powered coordinates](../conditioned-match/opening.json)

The feed uses the issued 850 mm guide top, 40 mm face, +20 mm fore/aft offset and original guide force law. The proposed 813 mm guide and its mounting change the geometry and compliance. This study grants them no acquisition or cable-clearance evidence. [Guide inputs](../shelf-contact/experiment.py)

## What did the sensors report?

The same powered approach that reached IN_BAND with an equivalent 2 mm guide faulted on the left after the middle and outer stages were allowed to move independently. The seat beam saw the pipe; the entry beam remained clear through the 1,020 ms timeout. Halving the integration step reproduced that failed sensor sequence. The table reports acquisition time in milliseconds on the original Java clock, with the feed lip at 100 ms. “Fault” means the controller inhibited further acquisition.

| Approach and support | Time step (µs) | Left calibration | Right calibration |
|---|---:|---:|---:|
| Original signed drop, rigid | 31.25 | 1,360 | 1,340 |
| Original signed drop, equivalent guide | 31.25 | 1,340 | 1,360 |
| Powered dock, rigid | 31.25 | 1,360 | 1,360 |
| Powered dock, equivalent guide | 31.25 | 1,340 | 1,340 |
| Powered dock, equivalent guide | 15.625 | 1,360 | 1,340 |
| Powered dock, two serial guides | 31.25 | Fault at 1,020 | 1,360 |
| Powered dock, two serial guides | 15.625 | Fault at 1,020 | 1,360 |
| Powered dock, softer guide stops | 31.25 | Fault at 1,020 | Fault at 1,020 |
| Powered dock, closed guide clearance | 31.25 | 1,340 | 1,340 |

The left powered arrival pushed one front bracket down 0.587 mm and lifted the two-stage guide by up to 1.894 mm in the finer run. Peak shelf contact was 2.158 kN. In the equivalent-guide run, the first accepted left observation shifted by one 20 ms scheduler tick when the timestep was halved. The finer timestep reached the upper stop on the right guide in this left-hand arrival; the coarse timestep missed it. Use the failed and passing observations together when specifying the shop measurements. [Two-stage traces](series.json) · [Integration refinement](refine.json) · [Guide controls](limits.json)

With 0.10 mm radial slack and zero residual preload, the four support lands carry the modeled incremental loads without engaging either old pin. Seating the old pins at zero gap makes the pins share the load. Peak force across both powered arrivals was 494–542 N per pin, equivalent to 177–194 MPa nominal cantilever bending. The coarser integration showed both pipes IN_BAND. At 15.625 µs both pre-seated cases faulted. The numerical refinement supports the force range but fails to establish either detection outcome. This is a checkable fit decision. Measure both installed pins under load and rebound before accepting the direct support as their sole vertical load path. [Pre-seated comparisons](engaged.json)

## How force reaches the sensors and old pins

The released 54-body STEP supplies independent front/rear bending compliances. Each cut of the native bracket contributes its actual unsymmetric section. The four M6 bolts per mast provide assumed shear/axial joint stiffness at their actual coordinates. Both sides use the measured CAD spacing; material modulus, grip and seating are design inputs. The 90 nominal slices used in the front-arm model omit unresolved torsion, slip and fixture rotation. Shop strain, dynamic damping, assembled support stiffness and absolute pin preload remain unmeasured. [Derived boundary](boundary.json) · [Original fastening and loads](../carriage-support/README.md)

Four vertical support freedoms reproduce rigid heave, pitch, roll and differential twist. They are coupled through the actual integrated shelf mass and curvature to the issued 13 plate modes. The non-plate mass is retained separately for the middle and outer stages or combined for the diagonal comparison. The 14.298 kg parked bound includes the pipe; the model removes that pipe from the support mass and simulates its rigid body separately. The nominal outer/middle mass allowances and equal end distribution are conditional. [Stage mass inputs](../integrated-slide/mass.json) · [Model](model.py) · [Mass and work checks](basis-checks.json)

The shelf moves with the rail stack while both absorber bases move with the bridge. Each absorber uses its own relative travel and speed, lower-face stand-off and finite return. The nominal guide stiffness is a trial 5 MN/m per side. The two-stage study gives each serial stage 10 MN/m and 1 mm of upper clearance, preserving the same static series stiffness and total free travel as the equivalent 2 mm model. Bottom and top contact are unilateral; the rail gap transfers neither spring force nor damping while open. A zero-clearance guide control, softer-guide trial and stiffer-bracket comparison expose unmeasured boundaries. The 2% plate damping acts only on deformation of the plate. Actual stick-slip, local torsion, side/pitch contact and force-dependent rail/stop damping remain unresolved. The dynamic model drives vertical surface coordinates. Horizontal bearing and screw traction need a separate measurement.

The Ø8 pins use the source 18 mm cantilever and actual 0.10 mm nominal radial slack. The first run assumes an installed gravity-loaded datum with zero residual vertical pin load. The comparison also seats both old pins without slack. Their bilateral stiffness then enters the modal matrix directly. Applying an already-seated stiff spring one integration tick late produced nonphysical excursions in the retained negative control. A pin that was carrying load when the shop fitted the bracket may retain that load. Each real pin needs an independent force or strain measurement. [Earlier pin load](../fixed-hoist/lock-transfer.json)

## Replay and qualify

The tables retain source hashes for both approach families, exact Java transitions, integration steps, first-contact histories, receiver deflection, guide travel, damper-base motion and calculated old-pin force. The chart shows the first 150 ms of contact with min/max-preserving decimation; each raw CSV remains rebuildable from the project sources. The equivalent-gap and two-stage calculations carry separate identifiers.

```text
python compliant-carriage/boundary.py
python compliant-carriage/verify_basis.py
python compliant-carriage/replay.py source <jdk17> <output-folder> --workers 2
python compliant-carriage/replay.py refine <jdk17> <output-folder> --workers 2
python compliant-carriage/replay.py limits <jdk17> <output-folder> --workers 2
python compliant-carriage/replay.py series <jdk17> <output-folder> --workers 2
python compliant-carriage/replay.py engaged <jdk17> <output-folder> --workers 2
python compliant-carriage/report.py <output-folder>
```

Before relying on the candidate, preload and instrument both original pins, measure all four stopped seat heights and guide gaps, and sample the middle and outer stages independently. Observe relative stroke at both absorber ends and the actual entry/seat apertures on a restrained bench fixture. Use the measured masses, stiffness, friction and preloads to repeat the two attained starts. A new mechanical configuration needs a new receipt.
