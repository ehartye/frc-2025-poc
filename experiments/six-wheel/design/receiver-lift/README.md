# Station receiver lift

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 83b3b9c7fe](../../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

The alternative twin Acme drive now has a source-bound comparison of clearance, critical speed, split-drive racking, stopping distance during power loss and mass displaced by a full transmission replacement. [Independent screw-drive comparison](../lift-screw/README.md)

The 545-body placement model shows a moving plate reaching its fixed service arm before the backup line completes its modeled catch. The successor study registers the interference, the independent end-damper packaging and the midtravel brake requirements. [Shared-shaft and power-loss study](../lift-arrest/README.md)

The registered 401-body delivery audit estimates 8.422 to 11.609 kg loaded before the proposed secondary suspension. Its separately modeled duplex lift adds 0.564 kg moving. The following study preserves the earlier 96-body sizing. [Current load, independent supports and qualification](RETENTION.md)

One NEO V1.1 drives two single-layer winch drums on a common shaft through a 16:1 Sport gearbox and a 1:1 transfer. A separate line climbs each mast, passes over its own sheave and ends at the moving crossbar. Mirrored winding takes up both lines in the same shaft direction. The mast guides carry side loads. The drive screen retains the mission's two seconds to raise the empty carriage from 670 to 940 mm. Cutting that move to one second needs at least 14.69 V under the model.

| Motion | Travel and time | Peak speed | Highest modeled motor voltage across drum revisions |
| --- | --- | --- | --- |
| Receiving height to transport, loaded | 270 mm in 2 s | 253 mm/s | 8.02 V |
| Transport height to place, loaded | 110 mm in 1.5 s | 138 mm/s | 4.72 V |
| Placement height to transport, empty | 110 mm in 1 s | 206 mm/s | 6.66 V |
| Transport height to receive, empty | 270 mm in 2 s | 253 mm/s | 7.99 V |

The 96-part design's moving frames contribute up to 10.768 kg in the [BOM](../station-l1/bom.csv). Allow 1 kg of additional moving hardware and the 0.8165 kg payload for a 12.585 kg loaded design case. Model bounds include 20 N guide resistance, 78% lumped drive efficiency, a 9.5 V loaded rail, a 35 A motor phase limit and an estimated motor-side rotating inertia of 0.000075 kg m². The largest phase current over the four scheduled moves is 9.68 A. Downward motion uses a conservative absolute effort envelope. Actual regenerative bus behavior, brake heat, guide friction, spool inertia and grip during transit await qualification. [Rerunnable sizing](size_lift.py) · [Numerical results](sizing.json)

## Hardware and load path

| Interface | Candidate and sizing decision |
| --- | --- |
| Motor and gearing | REV-21-1650 NEO V1.1 and AndyMark 16:1 CIM Sport, historical am-3761, with matching 16T pinion or vendor-selected Sport 8 mm round-input variant. Motor torque reaches the common shaft through a 1:1 transfer. The original catalog lists the historical gearbox sold out. Confirm stock, dimensions and compatible ratchet carrier for the selected build. [Motor](https://www.revrobotics.com/rev-21-1650/) · [Historical gearbox](https://www.andymark.com/products/16-1-cim-sport-with-1-2-in-hex-output) · [Current configuration](https://andymark.com/products/sport-gearbox) |
| Shaft and lines | Two am-4647_2000 drums, 1/2-in hex shaft, two am-4559 1/8-in lines, separate positive terminations and guarded guide sheaves. The 2022 kit layout gives a 0.750-in core and 1.06-in working width. The current family also lists a 0.787-in core. Across 380 mm of travel, the smaller core needs 7.44 turns including two retained turns. Close winding leaves 3.29 mm width. Check fleet angle, grooves, termination and cycle wear on both sides. Layer buildup changes travel per revolution and can rack the carriage. [Spool](https://andymark.com/products/climber-in-a-box-winch-spool-pulley) |
| Passive hold | Candidate am-4424 ratchet with am-4425 pneumatic release. Orient lock to oppose downward load. Provide a spring-biased engage linkage and a de-energized exhaust path, engaged-position feedback and a positive mechanical service pin. The complete regulator, protected pneumatic supply, valve, mounts, switch and checked engagement time are outstanding. The actuator/catalog language alone cannot establish default engagement with residual air or a bound linkage. [Ratchet Sport](https://andymark.com/products/ratchet-sport) |
| Release and recovery | While stationary, establish motor support and unload the pawl. Permit controlled descent only after release feedback. At the next stop establish and observe the mechanical lock before removing holding torque. A disagreement inhibits a new move. Loss of enable during descent requires a physical spring return, proven capture distance and independently measured arrest load. Never credit motor neutral as passive retention. |
| Loads | Full static gravity at the larger spool radius requires 1.43 Nm at the common shaft. A 2× static sizing target is 2.86 Nm. For a 10 mm fall at 253 mm/s the payload and carriage alone release 1.64 J including initial translational energy. Rotating inertia, compliance, shock amplification, ropes, mounting fasteners and a single-sided jam still need an arrest-load model and physical qualification. A gearbox ratchet cannot catch an independently failed line or anchor. |

## Registered CAD

The [routing generator](check_routing.py) replaces the original lift-drive reserve with 34 routing and mounting bodies in the [four-pose robot](../station-l1/README.md). The common shaft center is X −92 mm, Z 330 mm. Drums and top sheaves sit at Y ±252 mm. Shaft and sheave bearings feed the existing mast, while the motor mount rises from the deck. The bearing arms, crossbar anchor tabs, response to line failure, guards, ratchet actuation, guides and fastening details need fabrication drawings. The cylinder for the common shaft encloses the selected hex across all corners. Simplified interfaces and component reserves preserve room for a fit review.

At each pose the generator reopens the pinned STEP and checks 34 new bodies against 95 original solids and each other. At stow it also checks battery and cover removal. Two earlier layouts overlapped the electrical cover path, first at the transfer pulley, then at the left bearing. Moving the common shaft forward clears the two service paths at their nominal dimensions. The straight rope reaches mark one nominal winding lane. Spool traverse, curved wraps, side deflection and tolerance are additional swept volumes. [Routing results](routing.json)

Generate four named complete-robot STEP files for CAD review:

```sh
python design/receiver-lift/check_routing.py --step-dir artifacts/receiver-lift
python design/receiver-lift/size_lift.py
```

The alternative twin 3/4-6 Acme screws would turn at 3,588 rpm during the fastest scheduled lift. Fixed/supported 520 mm shafts meet the assumed critical-speed screen at 4,727 rpm, but the nut surface reaches roughly 706 ft/min and the supplier has not qualified this duty. The lower-speed winch remains the candidate. [Screw engineering data](https://www.roton.com/products/acme-lead-screws-nuts/engineering-data/) · [Nut wear model](https://www.roton.com/screw-university/screw-actions/screw-wear-life/wear-equations/)
