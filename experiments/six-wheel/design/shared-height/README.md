# Inspection inventory for two fixed heights

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 14649626c7](../../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

Keep the single-height mast for the selected match role until a representative two-hole mast passes a load test. Bringing the other single-height mast pair adds at most **1.383 kg** under the material allowance. That puts the represented inspection inventory at **53.056 kg**, with **14.983 kg** remaining under I103. The second pair can preserve the original section at the unused elevation. Its installation would require a supported pit rebuild, alignment, controller selection and any re-inspection the change requires. Either role still needs its own structural proof and finished weigh-in. [Inspection comparison](inspection.json) · [Fixed carriage and retention](../fixed-height/README.md)

## What can be shared

The [two-hole mast STEP](masts.step) provides a fabrication candidate for the same carriage in either position. In chassis coordinates both pin axes run along X at Y ±222.5 mm. Their centers are at Z 422.5 and 802.5 mm. The original single-height mast blocks the opposite sleeve by 452.39 mm³ on each side. An additional 12.1 mm through-bore clears the 12 mm sleeve and leaves a continuous 6 mm radial band of material on both 2 mm faces. One pair of sleeves goes with the carriage to the active elevation. Both inactive holes stay empty. [Native material intersections, sections and source revisions](native.json)

Registering the two lock kits in the same coordinate system yields 38 matching bodies after the mast change. The high configuration also needs a hole through each of six nested slide channels. Those six parts fit within the unmodified low rail stock in the corresponding carriage frames. Use the high drilled rails in both builds. Low travel requires both stroke pins to be withdrawn. The high build adds two pins, two washers and two keepers. Keep those six pieces in the inspection inventory even when presenting the low build. The previous single-height pin study records positive material contact after 1 mm of attempted motion. Rail-hole edge finish, bearing travel, wear, keeper fit, fastener reuse and actual assembly access require shop checks. [Rail and pin volumes](native.json) · [Pin retention witnesses](../fixed-height/native.json)

## What the second bore costs

At the unused lock elevation, native 0.1 mm horizontal sections centered on the bore measure 284.0 mm² before drilling and 235.6 mm² after. The centroidal fore-aft bending section modulus drops from 3,603 to 2,487 mm³. For 100 N of fore-aft force on one mast applied 380 mm above that section, the nominal elastic bending stress increases from **10.55 to 15.28 MPa**. This comparison treats the upper lock as the load point and the lower hole as the section of interest. Actual carriage load sharing, material, local stress concentration, mast-base restraint, welds, fatigue and collision loading determine strength. Test a representative mast and mount using the intended load paths before selecting the two-hole design. [Section properties](native.json)

## What goes on the scale

| Declared configuration | Represented physical items | R103 upper kg, bumpers off | R408 upper kg, bumpers on | Estimated R103 / R408 headroom kg |
|---|---:|---:|---:|---:|
| High with shared masts and six stroke-lock pieces | 595 | 51.671 | 59.171 | 0.492 / 2.064 |
| Low with shared masts and free slide | 589 | 51.642 | 59.142 | 0.521 / 2.093 |

The installed Agent FRC assembly checker returned the same mass bounds for each alliance color separately. The high assembly contains the whole 595-item union, so the shared-mast I103 upper estimate is 51.671 kg. The 597-item alternative starts from the original single-height high build and adds a second, dedicated pair of low masts. Both options use one set of drilled rails, lights, actuators and electronics. They assume those parts can be transferred and that no second assembled mechanism or extra installed hardware needs to be presented. Recount any separately assembled carriage, electronics change or alternate receiver. Physical item counts consolidate some CAD bodies into supplied assemblies and include known fasteners without CAD solids. [Installed checker results](inspection.json) · [Source accounting and omissions](../assembly-accounting/README.md)

For the historical event rules, weigh each fully dressed match configuration with the R103 exclusions applied. Keep the robot-side connector, attached mounting hardware, wiring and decorations with the robot. Weigh with each actual detachable bumper set for R408. Present every mechanism and configuration intended for use without re-inspection, plus the operator console, and count each distinct item once for I103. The console and R103 exceptions are excluded from the I103 weight. Present both color sets. The present estimates use 5.9–7.5 kg per removable bumper set. Finished scale readings, a complete loose-hardware inventory and inspection acceptance remain open. [2025 manual, R103 p. 78, R408 p. 88 and I103–I104 p. 116](https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf) · [Source and exclusions register](../electrical-inspection/SOURCES.md)

## Reproduce

Run from the repository root with the project OpenCascade Python and Agent FRC 0.21.0. The first command reopens both lock exports and the issued low slide, writes the two common masts and measures the exported solids. The second builds four source-pinned maps, invokes the installed checker and compares its physical item lists.

```powershell
python design/shared-height/build_masts.py
python design/shared-height/inventory.py $env:AGENT_FRC_PLUGIN_ROOT artifacts/shared-height/maps
python -m unittest discover -s tests -p test_shared_height.py
```
