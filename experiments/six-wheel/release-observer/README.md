# Measuring receiver clearance

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 83b3b9c7fe](../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

**The projected stereo pair can distinguish the reference departure from the slow, recontacting release. The entire timing chain currently passes only 15 of 17 synthetic frame phases in the tightest tested pixel-error case.** The checkout contains no physical calibration or vehicle adapter. The existing Seat and Entry ports can both report empty while the pipe still overlaps the shelf.

This observer produces shelf-scoped evidence. The full assembly can extend beyond that shelf. The desktop controller holds the slide on every legacy shelf packet. A separately registered whole-robot measurement exercises one reviewed simulated withdrawal. [Controller and timing evidence](../competition-controls/README.md)

## Camera, independent contact and installation

The [original STEP](stereo-mount.step) opens with the receiver in its place pose. Dimensions use robot millimetres, +X forward, +Y left and +Z up. [Open Cascade checks](mount-check.json) reopened all 14 solids and each of the four 96-solid robot poses. There was zero positive-volume intersection in those positions. Proposed upper guide and rope reserves are separately identified in the ray report. Intermediate motion, battery extraction, service tools, all fasteners and complete cable sweep still need physical checks.

| Assembly | Candidate build and acceptance |
| --- | --- |
| Stereo head | Luxonis OAK-D Lite USB fixed-focus variant, two rectified global-shutter mono eyes. Mount the midpoint at (−315, 0, 970), baseline along Y, 26° down. The 91 × 28 × 22 mm camera proxy faces forward in a slotted, padded 98 mm cradle. Retain with straps, avoid optical windows and check the real connector, eye origins, distortion and actual outer dimensions on the chosen revision. The nominal 640 × 480, 73° horizontal rectilinear model is for design. [Luxonis hardware](https://docs.luxonis.com/hardware/products/OAK-D%20Lite) |
| Fixed frame | Original 20 × 20 × 2 mm wall upright, 798 mm tall, on a 39 × 60 × 4 mm foot with four Ø5.3 mm holes. The cradle lug joins the upright. Fasten through the rear deck into a qualified backing plate, lock fasteners and support the mast against vibration. Measure line-of-sight change under shock. A 1 mm registration bound is a test target. |
| Pre-clearance contact | Three Interlink FSR 400 Short candidate pads at local u=155, y=−70, 0 and +70. They stay on the flat portion before the rounded nose. Head Ø7.2 mm, nominal active Ø5.6 mm. The STEP allocates a maximum 0.8 mm total laminate per sensor and a 30 mm rear-facing tail reserve. Bond to cleaned shelf, taper the wear cover and protect rear/side strain relief. Measure actual compressed and unloaded stack including adhesive, tails, wires and tape. A uniquely attributed loaded pad with stack <1 mm establishes a prior near-contact instant. An unloaded pad supplies no clearance. Test whether the carrier surface and all tails remain within the enlarged shelf enclosure. [Interlink FSR 400 Short](https://www.interlinkelectronics.com/products/fsr-400-short) |
| Force readout | Three 3.3 V divider inputs into ADS1015 AIN0–2. Start with 10 kΩ reference resistors, a known end-of-line resistor at each flex connector and one spare input for the reference rail. Record unloaded, loaded, open, short and stuck responses individually. Select divider, load thresholds and dwell from measured tube, cover preload and contamination. Log each conversion's timestamp and require an observed unloaded-to-loaded transition for this piece while the slide is stationary. The 2 ms three-channel scan budget must include ADC multiplex settling, software and transport. Keep analog harness twisted and shielded, isolate from belt leads, strain-relieve across the stage loop and measure full lift/slide motion. [TI ADS1015](https://www.ti.com/product/ADS1015) |
| Compute and power | Raspberry Pi 4B, USB 3 data to camera and Ethernet telemetry to receiver controller. Reserve separate protected 5 V paths using two Pololu D24V50F5 converters and the camera maker's externally powered USB Y arrangement. Host provision 3 A, camera design point 6 W. At 85% assumed converter efficiency that is roughly 25 W or 2.1 A from 12 V before transients. Verify 5–5.25 V at the camera, loaded voltage drop, backfeed isolation, heat and bus dip. Size upstream conductors/protection to actual wiring and current rules. Use a keyed disconnect and an anchored service loop at the body, keep camera and Ethernet fixed to the frame. Compute and supply reserves sit at the rear right of the deck clear of the four static configurations. [Pi 4B](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) · [Pololu 2851](https://www.pololu.com/product/2851) · [Luxonis electrical datasheet, pp. 3–5](https://raw.githubusercontent.com/luxonis/depthai-hardware/master/DM9095_OAK-D-LITE_DepthAI_USB3C/Datasheet/OAK-D-Lite_Datasheet.pdf) |

A downstream beam can miss the hollow center, a tilted exit, a drop or a reversal. Moving a pair of posts 55 mm beyond the front also puts them past the nominal dock plane during station receive. The stereo-plus-lip design keeps the camera on the chassis and the pressure pads on the shelf. Interlock a wrong, duplicate or obscured piece, raw-frame loss, calibration mismatch, unplugged sensor, brownout, stale network or clock reset. Hold the receiver or latch a fault according to the [controller state machine](../competition-controls/README.md).

## Reconstruct and bracket one departure

Fit each pipe end from synchronized rectified image contours, identify the same physical pipe across both eyes and register the shelf through a surveyed, encoder-checked lift/slide pose. A virtual cap center must come from a qualified exterior-rim fit. Reject bore or background matches, end swaps, missing rim arcs and ambiguous yaw. The code begins **after** feature extraction. The [interval estimator](stereo.py) bounds the two end centers over horizontal, vertical and disparity errors, encloses the whole pipe in a capsule, then subtracts registration and radius allowances. The shelf enclosure includes the full 0.8 mm pad stack. A fresh lower gap bound of at least 1 mm gives the latest crossing time. At the nominal rig, even perfect coincident pipe/shelf image coordinates with ±0.10 px error give a 12.02 mm upper bound on separation, so stereo point proximity cannot prove the prior contact.

The [offline adapter contract](witness.py) requires a prior qualified contact, a later identified positive stereo frame, monotonic common tracking epoch and a signed-off calibration. Clock bounds surround both timestamps. The [Java controller](../competition-controls/src/main/java/org/reefscape/controls/StationCycle.java) matches the attempt/piece, waits until 50 ms after the latest crossing and faults if withdrawal could start later than 150 ms after the earliest. The 150 mm/s ramp begins at the actual command tick. Recontact or tracking loss resets evidence upstream. [Sampled dynamics](../mission/release-latency-grid.json) at 50–150 ms used assumed friction and spin. Recontact appears at 600 ms on both examined approaches. The associated belt cartridges may still overlap the pipe's bounding box for the first 43 ms after shelf separation. Whole-robot clearance and scoring are separate observations.

## Timing and visibility from the 1 ms mission

Through the supported-to-first-trough-contact reference interval, the projected end centers span approximately x=234–406 px and y=235–242 px in the nominal 640 × 480 mono frames. At the stock shelf's first 1 mm gap, halfway to target contact and at target contact, all 24 finite lines from two eyes to two end-axis/upper-rim landmarks clear the reopened receiver package. These lines test robot obstruction. Pipe self-occlusion, untextured PVC, real field walls, exposure, distortion and cropped rectification reduce usable coverage.

| Per-coordinate bound | Positive frame phases / 17 | Capture lag after stock shelf crossing | Worst controller receipt | Phases guaranteeing start in the modeled time window |
| --- | --- | --- | --- | --- |
| 0.10 px | 17 | 35–51 ms | 79 ms | 15 |
| 0.25 px | 17 | 56–72 ms | 100 ms | 0 |
| 0.50 px | 12 | 77–88 ms among those detected | 116 ms among those detected | 0 |
| 1.00 px | 0 | Unknown | Unknown | 0 |

The repeat using the registered 8.95 m field transfer gives the same phase counts and latency bounds. Every slow-release case withheld clearance. The [replay](replay.json) assumes the 1 ms model trace, 17 ms exposure cadence, 8 ms processing/transport, up to 20 ms scheduler phase, 1 ms clock uncertainty and ideal visible rim correspondence. The 0.8 mm protective layer grows the geometry envelope, but mission dynamics still use the stock shelf. The last idealized center-pad tangency is at 14.82131019 s, versus stock 1 mm clearance at 14.84831019 s. A real load under this pad at this time is unverified.

## Reproduce and qualify

```sh
python release-observer/study.py --output replay-local.json
python release-observer/build_mount.py --output artifacts/stereo-mount
python release-observer/plugin_study.py artifacts/receiver-sensors
node "$AGENT_FRC_PLUGIN_ROOT/scripts/sensor-observability.mjs" artifacts/receiver-sensors/reference/request.json
node "$AGENT_FRC_PLUGIN_ROOT/scripts/sensor-observability.mjs" artifacts/receiver-sensors/slow-release/request.json
python -m unittest discover -s tests -p test_release_observer.py
```

The installed 0.15.0 checker accepts only the reference study. The project fixture combines three explicit nominal optical test positions with the actual final mission pose. The finite support encloses the stock R6 and protective layer, and the bore is a 49 mm radius sensitivity assumption. Scripted occupancy and simulated stereo carry no device qualification.

Bench acceptance needs taped ground-truth measurements or an independently calibrated high-speed view, synchronized exposure/ADC/controller receipts, actual registration and outer-radius bounds for all observed poses, and empty/obstructed/stuck/other-piece failures. Compare the force-pad timestamp, stereo estimate and commanded slide motion over every sampling phase. Record the minimum raw data confidence that attains the intended window. If systematic bounds are larger, the present design must hold pending a changed rig, a measured wider timing envelope or more tolerant withdrawal mechanics. The as-built evidence and electrical assignment remain open.
