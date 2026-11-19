# Build and service the aft stereo bridge

<!-- cad-still -->
[![Shop-duplex assembly in its stowed pose](../../../docs/robot/cad/assembly.png)](../../../docs/robot/cad/index.html)

**Project CAD · Shop-duplex assembly in its stowed pose.** Historical six-wheel CAD reference. The four-module swerve candidate is under development. [Source STEP · 193c7976d1](../design/competition-package/shop-duplex/red/stowed.step) · [Rotate the model](../../../docs/robot/cad/index.html)
<!-- /cad-still -->

The removable bridge locates on a metal round peg at the left foot and a slot at the right. Four captive screws clamp the feet to steel bosses through the shelf. Two more captive screws hold the camera. Students can remove the frame or the device without handling a loose fastener. The package keeps the camera pupils at **X48, Y±37.5, Z842 mm**, on the 560 mm fixed-height end-belt robot at zero slide. [Five printable dimension sheets](drawings.html) · [32-solid engineering STEP](camera-shelf.step) · [Make/buy list](bom.csv)

| Take to the shop | Use |
|---|---|
| [Shelf DXF](shelf-layout.dxf) and [operation list](shelf-operations.csv) | Set units to millimetres. Check the 265 × 420 outline. The DXF locates features in plan. Sheet 1 defines the front R6, depths, counterbores and through operations. |
| [Editable construction](camera_parts.py) and [part register](native.json) | Refer to the named solids for interfaces, machined keeper ears, hex access and pin bores. Threads use major-diameter CAD envelopes. The actual tap, hardware fit and engagement are specified on the sheets. |
| [Independent target pattern](calibration-target.dxf) and [measurement example](metrology-example.json) | Fabricate one target, independently survey it, move it between two known heights and record both eyes. The supplied example is synthetic. |
| [Full placement round trip](native-place.json) and [continuous service check](fit.json) | The complete no-air installation reopens as 603 named solids. The entire frame can lift 535 mm on the defined unpowered service path. Retain the source-owned light STEP locally to regenerate it. |

## Machine, assemble and remove

1. Start from new shelf stock. Identify underside A, square rear face B and robot-right edge C. Machine the ordinary receiver ports and M5 counterbores first and inspect their locations from the same B/C as the new features. The original shelf has Ø4.5 holes that overlap this family's earlier camera pattern. Re-drilling historic stock can leave a merged hole. [Earlier failure](../bridge-tolerance/README.md)
2. Face the two steel saddles in a matched fixture. Install the boss ends up through the new Ø8.20 shelf bores. Fit the independent M3 retaining screws, check head flushness, and mark saddle side and orientation. Measure the four annular clamp lands together. The M4 preload loop runs through each aluminum foot and steel boss. The permanent retainers hold the saddles in the shelf when the four main screws are disengaged.
3. Machine the round locating hole in the left foot and slot the right hole in Y. Leave the slot clear in the direction connecting the two pegs. Fit the 15 × 15 × 2 mm posts and crossbar, then join them using a qualified aluminum process with planned sequence and heat control. Machine or inspect the mounting lands after joining. Keep weld toes inside the service envelope and verify that both feet seat without rocking.
4. Fit all six socket screws before pinning their keeper ears. Select the actual spring pin and match its bore to the supplier's fit. Drive pins flush and inspect all twelve positions. Fit the purchased camera only after measuring its available thread depth. The 6.5 mm upper screws have 3.5 mm nominal engagement and need clearance at the bottom of the housing threads.
5. Lower the left foot onto its round peg and the right onto its slotted datum. Tighten the four frame screws using the preload method that passed the fixture test. Record witness marks. Connect the restrained cable, with slack throughout the 370 mm stroke. For service, support the frame, disconnect the cable and release the four lower screws. Lift vertically. Camera exchanges belong on a supported bench. Inspect the lens pose after reinstallation.

The original untapered foot collided with each grip dashpot by **35.39 mm³ at 25 mm lift**. The replacement removes 1 mm along the entire inboard edge, with an additional 2 mm relieved on its forward 24 mm. The native check covers the 535 mm extraction and an independent 1.5 mm lateral grip allowance. During normal 0–370 mm slide travel it checked 17,701 kit-to-existing-body pairs. The Ø3.60 driver gauge clears its capture pins by 0.20 mm. Each lower screw can rise 5.10 mm and each upper screw 3.60 mm to release its threads. Further extraction drives the head into its crosspins. Cable service, hand reach, wear and actual roof clearance remain physical checks. [Service and fault witnesses](fit.json)

The housing is a dimensioned envelope. The source leaves two connector exits, service loops and strain-relief points unmodeled. Locate those on the selected hardware, check access before lifting and repeat the full stroke with the actual loom installed. Route cable pull into the supported structure.

## Optical replay on the rebuilt frame

The new metal changed 117 grayscale pixels in each moving eye’s empty view. Rendering the rebuilt robot at the two modeled feeds produced 2,924 nominal moving exposures. Every estimated interval contained the corresponding pipe position. All three 5 mm / 2° misregistration controls lost the robot reference. The fixed camera remained outside its custody envelope on all 550 comparison exposures. These images cover the modeled housing, diffuse surfaces and existing optics. The rerender does not measure a real cable, lighting or delivery latency. The lower clearances to the occupied reef ball at the two target sectors remain 16.48 and 24.89 mm under the #71 pose limits. [Native image comparison](optical.json) · [Field and service bounds](fit.json)

## What the datums control

An Ø6.08…6.10 hole against the Ø5.99…6.00 left pin gives at most 0.055 mm radial play per installation. A reseat on the same saddles can span both extremes, **0.11 mm at the left station and 0.032° in yaw** across the 400 mm pin pitch. Adding individual saddle clearance, machining position and pin location gives a conservative 0.26 mm per station and 0.0745° absolute frame clocking. The right slot permits ±1.04 mm of nominal travel along Y. Prove the actual shelf expansion and starting position keep it off both ends throughout the shop's temperature range. [Datum stack](mechanics.json)

The 0.04 mm permitted difference between front/rear clamp lands alone could contribute 0.049° across 47 mm. The upper screws' clearance on the 75 mm camera pattern could allow **0.382°** before tightened. The lower datum scheme can repeat the frame; the housing screws, weld condition and optical axes still require an independent pupil measurement. Inspect both static installation and the final loaded pose. Do not combine a separate static fit and ideal flex result as a measured camera calibration.

## Elastic and joint screening

The project supplied a two-gravity comparison and 69 GPa aluminum modulus. The calculation adds the actual upper fasteners, plate, camera, crossbar, distributed posts and the earlier 115 g upper allowance for moving cable and welds. Metal receives a 5% mass allowance. It represents the posts as cantilevers, the crossbar as simply supported under gravity and restrained at both ends in torsion, and the split plate as two ribbed cantilever lands. It assumes Poisson ratio 0.33 for the shear estimate. This ideal member screen gives **0.364 mm summed tip excursion and 0.113° rotation**. The 1 mm tube section under the revised upper load reaches 0.176°, beyond the earlier 0.126° bend trial. Measure the actual assembled stiffness including the saddle, weld roots and shelf torsion. [Equations and load cases](mechanics.py) · [Prior optical perturbation study](../bridge-tolerance/calibration.json)

One post's 1.59 N·m foot moment can lift the heel with **33.9 N** at the 47 mm screw pitch. A hypothetical 25 N clamp loses seating. The 100 N trial retains 66.1 N under these assumptions. The nominal metal annular land area is 37.7 mm². Assigning all lateral force to one 6 mm locator gives about 1.01 MPa projected bearing demand over its 1.8 mm straight engagement. A 10 N cable pull at the top adds approximately 38 N heel demand with symmetric footing. These are fixture load cases. Select screw material and retention, establish preload from the actual joint, and inspect the heat-affected welds, polymer around the saddles, thread bearing, relaxation and pin retention. The source provides neither an approved torque nor a crash load. [Joint demands](mechanics.json)

The installed Agent FRC checker selects the complete replacement installation once and reports **1.577 kg represented R103 margin**. The budget includes 0.601–0.739 kg for the complete moving camera and fixed power provision on the selected #71 base. Independently subtracting the lightest prior metal and adding the heaviest new metal leaves 1.552 kg. Both estimates give no weight credit for the 4,389 mm³ removed from the shelf. Actual mass, new slide inertia and stow time need remeasurement. This study retains the #71 fixed power configuration. The later power-panel branch needs its own combined selection and interference check. [Installed accounting](accounting.json) · [Mass and purchase assumptions](shop-mass.json)

## Minimal physical qualification

- **Before clamping**, measure left pin/hole and right pin/slot, the 400 mm span, room at both slot ends, all four heights and foot rock. Check the plain clamp and captured-screw go gauges, including full thread disengagement, both eyes' actual camera-thread engagement, driver clearance and whether pin ends can walk. Record instrument uncertainty, actual screw lot and the finished weight.
- **To survey both pupils**, support the empty receiver, locate the same five-mark target independently from shelf A/B/C, and measure its upper surface at Z525 and Z685. At both heights capture an undistorted, time-stamped stereo pair. Keep sensor serials, intrinsics and lens settings pinned. Enter the surveyed mark coordinates, corrected pixel centroids, principal point and estimated ray-intersection uncertainty into the [checker](metrology.py). Four corners determine each plane mapping. Its asymmetric fifth mark checks the independent residual. Two separated rays at the two heights recover pupil position and direction; the lateral ray also observes camera clocking. The fifth mark alone cannot bound a shared fixture or distortion bias.
- **To qualify the small angular budget**, the synthetic example assumes 0.035 mm bounded XY uncertainty per evaluated ray, 0.025 mm plane-height uncertainty, 0.08 mm survey-origin uncertainty and 0.015° angular survey uncertainty. Even ideal observations then consume 0.930 mm pupil position, 0.040° tilt and 0.065° clocking. A shared 2 mm housing displacement, a 3 px asymmetric-target error or a 0.18 mm ray uncertainty fails the example. Establish achievable uncertainty with a known reference and independent hold-out positions before assigning those bounds to real images. [Checked examples](metrology.json)
- **After service and under load**, repeat against the 1 mm position, 0.12583° tilt and 0.1° clocking screen. Record the 0, 185 and 370 mm positions and cable routing, including both signs of the representative fore-aft and lateral two-gravity equivalent load in a restrained fixture. Record repeatability over multiple reseats and the largest residual after unloading. With an independently surveyed loaded pose, check live registration patches, bright pipe/end contrast, loss cases and timestamps. These tests can establish a design-specific calibration envelope. They do not authorize robot operation.

## Reproduce

Use the source-matched signal-light STEP, project Python CAD/optical environment, low-camera trace cache, installed Agent FRC root and the no-air [baseline accounting map](../design/fixed-no-air/README.md). Keep the native whole-robot export with its separately supplied source.

```shell
python camera-datum-shop/camera_parts.py
python camera-datum-shop/drawings.py
python camera-datum-shop/inspection.py
python camera-datum-shop/mechanics.py
python camera-datum-shop/metrology.py
python camera-datum-shop/metrology.py camera-datum-shop/metrology-example.json
python camera-datum-shop/account.py --plugin "$AGENT_FRC_PLUGIN_ROOT" --baseline-map artifacts/fixed-no-air/maps/low-camera-red-no-air.json --out artifacts/camera-datum-shop/maps
python camera-datum-shop/fit.py --lamp "$SIGNAL_LIGHT_STEP" --place-step artifacts/camera-datum-shop/place.step
python camera-datum-shop/optical.py --lamp "$SIGNAL_LIGHT_STEP" --scratch artifacts/fixed-height-observation
python -m unittest discover -s tests -p test_camera_datum_shop.py
```
