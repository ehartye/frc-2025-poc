# Receiver load path in a shared lane

<!-- cad-still -->
[![Preliminary design · in progress · layout-001](../../../docs/robot/preliminary-layout-001.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-001.** Red 2 B′x fixed pocket, roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 64 representative solids, with the field fixture, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 1cc8a76119](../../../docs/preliminary-design/releases/layout-001/receipt.json)
<!-- /cad-still -->

The provisional station-fed robot retracts its shelf for travel. In that pose the shelf tip lies **260 mm inside** the front bumper outline. At the 240 mm receive stroke it lies **20 mm inside**. At the 370 mm placement stroke it projects **110 mm past** the outline. The respective shelf tops are 610, 880 and 500 mm above the floor. All are above the 190 mm bumper top, so a plan-view setback alone cannot establish protection from an elevated obstruction.

## What load reaches the supports?

The study uses the 265 × 420 × 15 mm receiver stock, the four proposed shelf attachment centers at X = −80/+140 mm and Y = ±180 mm, and the shelf's rear datum 100 mm behind the robot origin at zero stroke. The attachment group translates with the shelf. The ported stock, shelf spacers and nested slide rails transmit load to the carriage and chassis. The exposed corner of the shelf is the selected point of application. Robot +X points forward, +Y left. [Shelf dimensions](../cad/station-receiver/design.json) · [Study inputs](study.json)

| Hypothetical encounter | Available relative energy | Average force over assumed compression | Largest modeled in-plane shelf-fastener demand | Placement yaw moment about robot X = 0 |
| --- | ---: | ---: | ---: | ---: |
| 1 m/s relative, 50 mm combined compression | 12.5 J | 250 N | 90 N | 134 N·m |
| 2 m/s relative, 50 mm combined compression | 50 J | 1,000 N | 360 N | 535 N·m |
| 2 m/s relative, 25 mm combined compression | 50 J | 2,000 N | 721 N | 1,070 N·m |

Both moving bodies are assumed to be 50 kg. These scenarios use the installed plugin's central free-body impact model to set a load scale. The calculation then applies that entire average force sideways to the shelf. Actual force transfer, rotation, crush distance, frame constraints and peak force require measurements. An impact on the bumpers follows a different path. The mass pair is an explicit trial input.

Each fastener carries one quarter of the applied in-plane force plus the torsional share about the attachment group's centroid (X = 30 mm). The torsional share is perpendicular to its radius vector and proportional to radius. Both summed force and summed moment are independently checked. This model assumes equal shear stiffness, rigid load distribution and no frictional preload credit. At 1,000 N the attachment-group moment magnitude is 135 N·m in every pose. The longer lever arm from the robot datum raises the corresponding moment from 165 N·m in transport to 535 N·m at placement. Bracket loads additionally depend on the actual support coordinates.

## Consequence for the design

Use the retracted state for movement through a contested route. Require a confirmed retracted indication before leaving the interaction zone, and record the chassis stop and fault behavior if that indication is absent. Keep the placement stroke stationary. A confirmed stroke alone cannot establish the lane is clear. If contact while extended is part of the selected strategy, specify a measured load case and qualify the stock, fasteners, inserts, slides, carriage and frame together.

The remaining mechanical evidence includes the polymer grade and temperature, material around ports and counterbores, fastener grade and engagement, out-of-plane loads, joint slip, bearing and tear-out, carriage play and permanent deflection. Define the permitted residual tip misalignment from the measured seating/release margin. The current scenarios supply neither a peak rating nor an attachment pass. Record any supervised test separately with the measured load–displacement curve, actual configuration and post-load retention, retraction and release results.

## Reproduce the calculation

From the project root run the following with a local Agent FRC checkout or installed package. Use a new filename to preserve earlier observations.

```sh
python structural/receiver_load_path.py --plugin "<agent-frc directory>" --output receiver-load-run.json
python -m unittest discover -s tests -p test_receiver_load_path.py
```

The [recorded scenarios](scenario-receipt.json) identify the plugin version and hash the actual calculation code and inputs. The report intentionally leaves peak force and structural acceptance unresolved.
