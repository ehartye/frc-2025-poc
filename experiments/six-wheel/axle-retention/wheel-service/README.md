# Change a wheel with the welded frame assembled

<!-- cad-still -->
[![Preliminary design · in progress · layout-001](../../../../docs/robot/preliminary-layout-001.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-001.** Red 2 B′x fixed pocket, roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 64 representative solids, with the field fixture, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 1cc8a76119](../../../../docs/preliminary-design/releases/layout-001/receipt.json)
<!-- /cad-still -->

The center wheels have a straight removal path with all six hub screws installed. The front and rear wheels need one screw removed temporarily below the frame. Both procedures work on either side in the registered CAD. Each has a nominal 0.050 mm axial clearance somewhere in the stack. A physical first-article trial with the received screws, measured shims and service fixture is still required. [Six-wheel native check](clearances.json) · [Shaft and retainer sequence](../README.md) · [Screw receiving limits](../../design/selected-shop/articles.json)

Dimensions are in millimetres. Outward means increasing `|Y|` on either side. The axle center is at Z76.2. Wheel center X is +247.65 front, 0 center and −247.65 rear.

## Prepare the stack

1. Isolate robot power and support the chassis on a stable stand. Leave **more than 180 mm below the original tire contact plane** for withdrawal, plus room for the wheel support and hands. Remove **both front and rear detachable bumper U assemblies** and their loose hardware for every wheel change. The eight fixed bumper sleeves remain in the model. The preceding shaft-only check needs just the adjacent bumper at an end station.
2. Secure the sprockets, chain and loose inboard spacers to keep them aligned when the axle comes out. Support the tire on a guided, height-adjustable rest. Mark the wheel, hub, shims, screw phase and spacer order. Take off the caps and selected shims at the accessible shaft ends. On a center wheel, park the washer and sleeve, extract the radial clip along the [proved tool path](../service.json), and take out the thrust washer. Keep the bearing keepers, carrier clamps, gearbox and welded rails installed.
3. Introduce the outboard spacer jaw from below and set its Y stop while the shaft still locates the stack. On a center wheel, return the wheel-side sleeve from its clip-service position to its nominal datum for this operation. Use the source-matched shaft procedure to pull the bare axle 160 mm outward. Keep the loose stack registered while transferring it onto a temporary pilot. The project models the chain as side bands with a representative pin. Check the real chain, tension devices and stand at the robot.
4. Introduce a straight, headless **Ø12.4 × 32** slave mandrel from outside along the cleared bearing and spacer bore. Use a removable pushrod to park its ends at `|Y|278.35–310.35`, then withdraw the pushrod. Reserve an internal extraction thread in the mandrel for reversal. The 6.8502 mm engagement into the inboard spacer, 0.175 mm nominal minimum insertion clearance and 0.3502 mm minimum lowering clearance apply at each wheel. The stub bridges the wheel hub and the nearest inboard spacer. The inboard spacer and mandrel travel **with** the wheel. Guide the group in Y with the tire rest and keep the mandrel from creeping. The mandrel supports the bore radially. Its slip fit alone cannot set axial position. Check fixture stiffness, induced sag, jaw grip and positive stops under the actual wheel weight.

| Station | Lower together with pilot | Hold separately | Withdrawal |
|---|---|---|---|
| Center L or R | Wheel, hub, `spacer-center-2-{side}` | `spacer-center-3-{side}` | Move outer spacer 1.50 mm out, then drop 180 mm |
| Front L or R | Wheel, hub, `spacer-front-1-{side}` | `spacer-front-2-{side}` | Hold at nominal Y, follow the one-screw sequence |
| Rear L or R | Wheel, hub, `spacer-rear-1-{side}` | `spacer-rear-2-{side}` | Hold at nominal Y, follow the one-screw sequence |

## Center wheels

With the clip, washer and shaft out, shift the supported outboard spacer **1.50 mm outward** into the released center-retainer space. Hold it there. Lower the supported wheel, hub, pilot and adjacent inboard spacer together 180 mm. The gap between the screw envelope at any wheel angle and the parked outer spacer is **0.725 mm**. The inboard spacer passes the neighboring sprocket at **0.050 mm**. Reverse the translation before returning the outboard spacer to its nominal position. The front and rear spacers have only 0.050 mm nominal outward room to their bearings.

## Front and rear wheels

1. Clock a screw to six o’clock at the installed height. Hold the outboard spacer at nominal Y and **lower the group 8.80 mm**. Lock the wheel rest at this height.
2. Remove the six-o’clock hub screw outward through the jaw’s Ø13 access port. Its axis is at **Z43.5875**. The Ø11 head gauge clears the underside of the outer rail by **0.9125 mm**. The model covers a straight socket up to Ø9 through `|Y|450` and 38 mm of screw extraction. Measure the received hex size and socket/extension OD, shoulder and entry length. The actual driver body and hand need room beyond this straight corridor.
3. Rotate the empty bolt hole to **twelve o’clock**. The complete screw rotation has **0.5125 mm** to the outboard spacer. Lower the five-screw assembly another **171.20 mm** without changing clock. Support and bag the loose inboard spacer and pilot as soon as the wheel is clear.
4. Reverse the path. Introduce the same group with its empty hole at the top. Stop 8.80 mm below the axle, rotate the empty hole to the bottom, and restore the sixth screw while the tool corridor is open. These are thread-forming screws in the wheel. Verify thread condition, supplier fastening procedure, service reuse and seating before loading the wheel. Account for all six screws before driving. Raise to axis and support all loose parts. Pull the temporary mandrel back outward with its slender retrieval rod, using the same straight corridor. Insert the permanent shaft from outside and reinstall the recorded shims and retainers. Verify full clip seating where applicable, torque, free rotation, endplay, engagement and bumper fastening.

## Service fixture and counterexamples

The outboard jaw envelope is **24 mm wide in X, 1.4 mm thick in Y** at `|Y|312.0–313.4`; its open Ø18.2 seat is centered 0.10 mm above the nominal axle. The Ø18 spacer touches its lower tangent. The same jaw carries a **Ø13 through-port** on the lowest service screw axis at Z43.5875. The calculation fills the port for insertion, rotation and descent as a conservative gauge, then uses the open port for the socket and screw. A solid jaw without this port blocks both. The complete inserted jaw stays 0.5248 mm from the screw at the initial position. Raising the jaw 0.20 mm into the supported spacer produces 4.05 mm³ of material overlap. Use a guided sliding support with a depth stop and gentle side pressure to retain the spacer. Jaw actuation, stand, clamp force, real handle and fixture deflection remain shop qualification items.

Leaving the outboard spacer at nominal Y and dropping the six-screw wheel straight through it produces **35.76 mm³** of overlap in the tested clock. The screw-head gauge overlaps the spacer axially by 0.775 mm. Six Ø11 heads on the Ø47.625 bolt circle leave just **12.8125 mm** between adjacent heads, smaller than the Ø18 spacer. Clocking or jogging the complete ring in the wheel plane cannot pass that spacer at the receiving limit. Keeping one screw out while parked at service height opens a route. A smaller measured head lot may have a different envelope.

## Reproduce the check

Use the project Python/OpenCascade environment. Supply the source-matched 603-body red place exchange under the artifact root. The [camera support instructions](../../camera-datum-shop/README.md) document how to regenerate it from the supplied light STEP. From the repository root run

```sh
python axle-retention/wheel-service/check.py artifacts artifacts/wheel-clearances.json
```

Compare the output with [the issued receipts](clearances.json). Inputs include the [weldment](../../frame-corners/frame-weldment.step), [shop carrier clamps](../../design/installed-procurement/inner-clamps.step), [radial axle kit](../axle-kit.step), [keeper lips and screws](../bearing-keepers.step) and [prior shaft sweeps](../service.json). The checker pins each native input and verifies the six shaft pulls and the #84 screw limits before evaluating the wheels. Tangent prisms and round ends cover every intermediate point of each straight translation. Rings cover all screw angles during clocking, and the other five wheels keep their complete rotating hardware envelopes. Body and screw checks use conservative outer profiles. The receipts document CAD clearances at nominal dimensions and a fixture geometry a team can trial on its actual stack.
