# Power, weight and tire grip for four-module swerve

<!-- cad-still -->
[![Four-module swerve frame and mounting hardware](../../../docs/robot/cad/assembly.png)](../../../docs/robot/cad/index.html)

**Project CAD · Four-module swerve frame and mounting hardware.** Original robot structure for the swerve candidate. The project-local Fusion assembly adds four manufacturer module occurrences. [Source STEP · f8b75f2df4](../packaging/swerve-frame.step) · [Rotate the model](../../../docs/robot/cad/index.html)
<!-- /cad-still -->

The revised opening fits the simulated power and tire limits at **65 kg and 72 kg**, for both signed module calibrations. Inspection weight remains the binding design constraint. With the shorter wiring target the dressed R103 upper endpoint is **54.792 kg**, 2.629 kg over the 52.163 kg limit. The 72 kg opening covers even the longer wiring estimate with the battery, 7.5 kg bumpers and one pipe, although that trial drives an empty robot. [Reconciled mass](installed.json) · [Four achieved opening clocks and adverse control](routes.json)

## Why slow the transition into the turn?

The first trajectory's four fixed normal loads hid a rear-right tire deficit at 3.135 s. Keep equal static corner weights, put the CG at 310 mm, make the two front and rear vertical tread rates 90% and 110% of nominal, and resolve the inertial load transfer. The rear-right patch has 71.02 N normal and needs 59.37 N tangential. That requires friction 0.836 on the assumed 0.80 surface. The revised controller slews the translation vector before the next waypoint and recomputes four unilateral normals during every 1 ms plant step. [Consistent counterexample](routes.json) · [Four-pad contact model](loads.py)

| Revised empty opening | Maximum attained tire force / normal | Worst modeled terminal on 11.8 V, 27 mΩ pack | Peak pack current |
|---|---:|---:|---:|
| 65 kg, positive calibration | 0.171 | 11.401 V | 11.523 A |
| 65 kg, negative calibration | 0.171 | 11.401 V | 11.524 A |
| 72 kg, positive calibration | 0.154 | 11.386 V | 12.086 A |
| 72 kg, negative calibration | 0.154 | 11.386 V | 12.085 A |

Each clock supplies four 5 ms samples of force, steer rate, motor rotation and signed drive/steer phase demands per tick. The independent screen checks both neighboring velocity differences, body-frame centripetal and angular-acceleration terms at the offset CG, different vertical tread rates and friction ellipses reduced here to circles. With all four static loads at least 22% of total and ±10% vertical rate variation, the minimum remaining tangential capacity is over 69 N on the issued clocks. The bound uses the actual row mass and the stated allocation for the remaining articles. A 1 mm diagonal height-error control loses one wheel. The wide, unrestricted remaining-mass case cannot guarantee four-point contact. [Load and CG assumptions](balance.json) · [Route calculation](route_grip.py)

## What draws from the battery?

Each of the four REV-21-3005 modules has a 76.2 mm wheel, 12T low reduction at 5.50 motor turns per wheel turn, NEO V1.1 drive and NEO 550 azimuth at 9424/203 motor turns per fork turn. Each motor has its own SPARK MAX. The central drive input and carrier-mounted intermediate gears give a locked-wheel coupling magnitude of one drive motor turn per azimuth turn. The signed fork calibration remains corner-specific. [REV module and mass](https://docs.revrobotics.com/ion-build/motion/maxswerve) · [REV drive motor](https://docs.revrobotics.com/brushless/neo/v1.1) · [REV steering motor](https://www.revrobotics.com/rev-21-1651/) · [REV controller](https://www.revrobotics.com/rev-11-2158/)

The four drive channels are PDH 0/1/10/11 with CAN 11/12/17/18. Steering uses PDH 12/13/14/15 and CAN 19/20/21/22. In front-left, front-right, rear-left, rear-right order, each NEO Hall lead plugs into its own controller. Each through-bore absolute encoder feeds the corresponding steering controller's data port and uses the controller's CAN identity. Keep both CAN end terminations at the roboRIO/PDH and the isolated former lift on PDH 2/CAN 13. Camera converters retain 6/8/9, slide and grips 3/4/5, radio and roboRIO 21/20, and the inactive-compressor PH remains 7. [Protected circuits](branches.json) · [REV wiring procedure](https://docs.revrobotics.com/ion-build/motion/maxswerve/wiring-maxswerve)

The project proposes 40 A drive and 20 A steer phase caps. At each tick the rail solver charges the voltage the motor needs, the warm phase wire, conversion loss and source/branch droop. The battery calculation uses the magnitude of electrical power in braking as a conservative supply load. It includes eight controller idle draws, the host, both cameras, radio, roboRIO, PH and the stowed receiver. A second force-based calculation increases effective wheel and steering inertia by 50%, doubles steering scrub moment and reduces drive/steer transmission efficiency to 75%/60%. The emitted and adverse demands pass the 12.4 V/25 mΩ, 11.8 V/27 mΩ and 11.8 V/32 mΩ cases on all four openings. Motor cap, feeder current and motor voltage are distinct limits. Breaker time-current, motor heating, charge acceptance, battery dynamics and active compressor use need separate evidence. [Independent equations](electrical.py) · [Emitted phase comparison](trace_power.py)

At 1 m/s with all four steering at 4 rad/s and 25 rad/s², the 65 kg equal-share study allows 3.35 m/s² with a stowed receiver or 1.05 m/s² under the simultaneous full rack/grip envelope. These are independent design corners at 0.025 m/s² resolution. Only the stowed receiver ran on the issued opening clocks. [Acceleration limits](corners.json)

## Place and service eight controllers

Four project-made folded trays sit on the stationary outside rail tops. Each holds two 70 × 35 × 25.5 mm controller cases with two straps per controller. The front pair and rear-left tray center on X±141 mm. Move the rear-right tray to X−99 mm. At X−141, the drive case enters the original camera data trough by 156.6 mm³. The corrected cases and their 40 mm straight removal sweep clear the registered obstacles and camera harness. Loosen the straps, unplug the leads and lift a single controller straight upward. Reserve clearance around the mating plugs and wire clips during assembly. [Reopened project-made trays](controller-trays.step) · [Native service and two-wall probes](controller-fit.json)

The two Ø4.5 mm tray holes per mount sit over a 2 mm existing rail wall at Y±337 mm. A Ø6.2 mm blind-insert preparation removes about 60.38 mm³ from the accessible top wall at each site, with room in the cavity below. Treat the blind insert, finished barrel projection and compatible grip range as receiving conditions. The pierced tray floor has two upright side returns. The static and dynamic robot-field screens do not qualify insert pullout, strap retention or fatigue under carrying/impact loads. [Tray source](controller_fit.py)

Four 16 mm diameter supply corridors and eight 10 mm local phase/sensor corridors have project clearance margins between their explicit endpoints. The four supply centerlines span about 0.096–0.478 m, with 30 mm centerline bend radius where they turn. Across the twelve, the native/registered lower clearance is at least 2.0 mm. Preserve the rear-right separation from the fixed camera data channel. The endpoint rating concerns those free spans. Controller/PDH mating tails, actual terminal orientation, individual encoder plugs, final hold-down clips, wire bundle fill and installed minimum bend are separate receiving checks. The shorter hypothesis allows 0.42 m per supply for tails, breakout and service loops beyond the qualified centerline, and 0.15 m added phase wire per motor. No old wire mass credit enters either budget. [Cable paths](paths.json) · [Stationary occupancy handoff](envelopes.json) · [Wire mass assumptions](harness.json)

## Reconcile inspection weight

The final mount drawing charges 1.496 kg for the eight saddles, bolts, bosses, receiving spread and weld/finish before any chip credit. Its same-stock upper charge is 1.279 kg. The preliminary upper estimates at commit 460607a were 55.648 kg long and 54.628 kg short. The source assembly has 1,199 identities. Retire the six-wheel drive and substitute the seven revised rail/deck parts. Two existing NEOs and their two SPARK MAX controllers remain in the swerve installation. Count the four furnished modules at an incoming 1.634–1.806 kg per module, with both motors included in each weight. Charge only six additional controllers at 108–120 g each. The old two motor mass rows are transferred into the populated-module budget. Credit the named original drive rows, their selected light carriers and axle-retention parts, the 36 wheel-hub screws and the complete formed guard hardware. The 22 camera carrier links and mast ear set remain. Credits for machining use the exact supplier-independent frame/deck volumes under a matched-stock assumption. [Original R103 row exchange](mass.json) · [Installed item partition](installed.json)

| Condition | Upper R103 estimate | Margin |
|---|---:|---:|
| Complete longer harness, including all positive mount/fastener charges | 55.813 kg | −3.650 kg |
| Shorter target, allocated tails and accepted individual leads | 54.792 kg | −2.629 kg |

At the shorter endpoint, the inherited 2–5 kg reserve for unitemized routing, transmission, latches and hardware must stay at or below **2.371 kg** to fit the inspection cap. This is a break-even procurement requirement. The selected shared bill still keeps its full 5 kg upper allowance. With the longer wiring, full bumpers and the 6.17 kg battery, the upper field case is **69.483 kg empty**. Add 0.8165 kg for a pipe. The assembly tool also emits a wider, independently varying interval for the same item exchange. Direct matched source-row endpoints avoid crossing independent maxima and minima of a part with its own retirement. Physical weight, loaded four-pad distribution and mounting qualification remain open. [Mass solver](installed.py)

## Reproduce

Run each script from its folder. Pass the installed Agent FRC directory, the captured-rail selected map, the external primary artifact root and a separate writable artifact directory. The source register identifies each raw motion CSV by exact bytes.

```text
python controller_fit.py ARTIFACTS controller-trays.step controller-fit.json
python cable_routes.py ARTIFACTS paths.json
python envelopes.py
python corners.py
python harness.py
python register_map.py PROJECT CAPTURED_MAP REBOUND_MAP_DIRECTORY
python mass.py PLUGIN REBOUND_MAP_DIRECTORY/selected.json mass.json
python balance.py
python installed.py PLUGIN REBOUND_MAP_DIRECTORY/selected.json OUTPUT_MAPS installed.json
python report.py clocks.json MOTION_RAW routes.json
python ../../../tests/swerve_power_suite.py
```
