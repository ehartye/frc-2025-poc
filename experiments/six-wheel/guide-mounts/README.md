# Mounting the station guides

<!-- cad-still -->
[![Preliminary design · in progress · layout-001](../../../docs/robot/preliminary-layout-001.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-001.** Red 2 B′x fixed pocket, roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 64 representative solids, with the field fixture, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 1cc8a76119](../../../docs/preliminary-design/releases/layout-001/receipt.json)
<!-- /cad-still -->

[Compare the cable crossing and the removable guide in 3D](guide.html). At a slide extension of **327.80 mm** the modeled moving power cable passes through the old right-hand liner. Its center is at X259.34, Y−154.50, Z830.09 mm. A 0.25 mm-radius sphere centered there sits wholly inside the original face. Both shortened guides end at **813 mm**. The new liners retain the original inward surfaces below that height. [Native fit and centerline witness](fit.json)

## Assembly and service

Each mast carries two 30 × 12 × 0.8 mm aluminum rectangular arms centered at Z687 and Z787. Each arm meets a 6 mm root plate stiffened by two rear returns. Two M4 through-fasteners and 50 mm crush sleeves connect that assembly to the two machined mast walls. At the nose, two M3 flat-head screws join the folded 0.8 mm backing and fitted wedge to an insert tapped inside the open arm. The screw heads are recessed below the bonded 3 mm liner. Replacing covered screws requires removing and renewing the liner. Form radii, joining distortion, fastener seating, thread retention, tool sizes, adhesive thickness and peel strength need receiving measurements. The underside wedge and liner bond must preserve the surveyed inner faces.

The STEP exchange contains 58 reopened single solids, including both modified masts. All 56 added articles clear the registered full-stroke bodies and the screened harness. All 24 root-driver and extraction gauges clear. The earlier wide arm and the original tall guide preserve distinct interference controls. The native exchange represents one welded arm/root per station and one article per loose fastener. [Reopenable STEP](guide-mounts.step) · [Editable construction](parts.py) · [Swept fit](fit.json)

With the slide stowed and unloaded, the symmetric shop removal path uses 65 mm of inward movement followed by 670 mm forward. The root fasteners are removed and the sleeves remain in the masts. The conservative swept regions cover all 24 moving module articles. Reversing the sequence and pulling forward first crosses a roof post. Pulling the right module outboard with the slide fully extended reaches the reserved carrier volume. Physical clearance, personnel access and restraint of the stowed robot still need checks on the finished machine. [Service path and adverse cases](service.json)

### Assembly position changes the throat

The root screws can all fit with both modules 0.315 mm outboard. On each mast the sleeves shift 0.040 mm, the screw axes shift 0.120 mm and the modules shift 0.315 mm before tightening. The 58 native bodies retain all four mast lands with no within-side material overlap. At the lower parallel faces this changes a 301.025 mm throat into 301.655 mm. A centered 301.625 mm pipe then clears each end by 0.015 mm. Both bore patterns can accept that placement. Set and survey the position of each face relative to the robot centerline at two elevations before and after tightening. A width check alone misses a common sideways shift. Match-drilled locating features and a repeatable service gauge are potential revisions. [Measured-geometry control](datum.json)

## Loads at pickup

Both attained, machined-shelf histories contain 171,520 stored states at 31.25 µs steps. Reconstructing the assigned annular end contacts at all eligible states gave maximum contact heights of 787.37 and 795.38 mm. None of their penetrating guide cells touches the removed strip. Peak normal loads were 2.160 and 2.395 N. These trajectories used the earlier rigid guide and the original support assumptions. A new compliant pickup can take a different path.

With the existing pressure resultant applied at its height on the folded spine, the nominal estimate gives **0.088 and 0.104 mm** peak normal movement. It includes arm bending, gusseted root bending and tension, plus the axial bolt couple over a 14 mm pitch. Halving the effective couple increases the right-side peak to 0.237 mm. Omitting the root ribs gives 0.273 mm, and the flat backing reaches 0.552 mm. The model assumes seated joints, a rigid mast datum, the stated aluminum modulus and a sound liner bond. Guide twist, vibration, mast-wall deformation, fabrication variation and joint slip still need a fixture and a new coupled pickup. The contact face has only 0.30 mm nominal interference per side. [Both histories and end conditions](loads.json)

## Whole-robot allowance

Agent FRC 0.22.2 applies the four supports, two face/liner assemblies, hardware, modeled weldment stock and joint allowances to the winchless fixed-height build. The estimated addition is **0.482–0.591 kg**, including the liners. Both machined masts replace their predecessors with zero drilling credit. The same 1,189 active identities appear in all four comparisons.

| Shelf material basis | Full old winch allowance verified absent | Zero winch-group credit |
| --- | ---: | ---: |
| Matched shelf material | +1.002 kg | −0.798 kg |
| No original shelf material credit | +0.082 kg | −1.718 kg |

The 82 g upper-estimate margin leaves little room for unlisted fittings or stock variation. Verify the whole retired group and weigh the complete inspection configuration. An as-built margin, qualified optics and scored placement remain open. [Installed accounting results](accounting.json) · [Winchless mass basis](../winchless/README.md)

## Receiving measurements

1. Survey the guide tops, throat, liner surface, contact strip, bolt seats and both body-to-mast datums. Gauge under-head recess and the actual sleeve, countersink, thread engagement and projection. Inspect weld continuity and accessible torque witness marks.
2. Apply measured loads near Z627, Z687, Z781 and Z787. Record normal displacement at the contact face and both roots in both load directions. Measure clearance, preload loss, plate twist and recovery after unloading. Compare those curves to the signed force histories.
3. With independent shop restraint, verify the full slide path, moving cable envelope and stowed module-removal sweep. Re-survey both camera backgrounds with the new visible material, then replay station capture and loaded travel at both signed approach errors.

## Reproduce

Use the same project-generated 991-body scene, signed raw trajectories, and winchless material maps as the linked studies.

```text
python guide-mounts/fit.py <fixed-camera-registered-scene.json>
python guide-mounts/service.py <fixed-camera-registered-scene.json>
python guide-mounts/datum.py
python guide-mounts/loads.py <coupled-corrected-trace-directory>
python guide-mounts/account.py <installed-plugin> <winchless-supported-maps> <map-output-directory>
python guide-mounts/view.py
```
