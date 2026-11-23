# Can the robot locate its receiver at the station?

<!-- cad-still -->
[![Fixed pocket 1.0, preliminary design](../../../docs/robot/fixed-pocket.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Project CAD · Fixed pocket 1.0, preliminary design · in-progress.** Whole-robot blockout on four-module swerve: fixed pocket, held two-roller nip and deploy-once exit lip. Parked pose at the end of the withdraw-park case, lip deployed and pocket empty; field parts, the delivered pipe and the two service hand gauges are hidden. It is not detailed CAD. [Open the design review](https://ehartye.github.io/frc-2025-poc/preliminary-design/)
<!-- /cad-still -->

The current downward-facing release camera cannot see the four mounting bolts around station marker ID2. An upward-looking global-shutter camera on the right mast can see them in the registered model. Start with that single camera for a measurement fixture. A second on the left adds little margin for this task. Both arrangements depend on an independent survey tying the static panel, actual pipe path and loaded receiver together. The physical alignment decision remains **hold**. [Receiver entry windows and student docking lab](../station-supply/DOCKING.md) · [Native visibility](sightlines.json) · [Numerical uncertainty and controls](pose-study.json)

## What the camera measures

Use the red corner station from the January field model. Station +X points from the chute toward the robot. At the 40 mm dock the robot looks back along −station X and robot +Y is −station Y. The landing center in station coordinates is (98.529, 0, 937.15) mm. The same point in nominal robot coordinates is (375.996, 0, 937.15) mm at lift 940 mm and slide 270 mm. It is the rotation pivot for the survey. The two virtual cross-sections are at station X −1.471 and 198.529 mm. At either plane the lateral reading is `dy + (plane offset − dx) × tan(heading)`. Project all eight corners of the bounded depth/side/heading box to obtain both reading intervals. They share a single pose estimate.

The 1 mm outward relocation of both inboard stops gives two tested input windows. Both allow ±5 mm normal error. The tighter one allows ±0.35 mm side and ±0.15° heading, leaving 0.477 mm of rigid clearance. The wider one allows ±0.50 mm and ±0.25°, leaving 0.113 mm. These are budgets for the **relative pipe/receiver state**. Any uncertainty in actual chute delivery consumes part of that same budget. [Native stop and entry sweep](../station-supply/entry-window.json)

Identify the station by its decoded 36h11 marker. Measure the exposed flat bolt faces around it. Their CAD centers have station X 11.0744 mm, Y ±117.475 mm and Z 1368.425 or 1603.375 mm. That 234.95 mm square gives a larger baseline than the printed marker and avoids assumptions about sticker placement. Each face spans about 19–32 pixels along the image axes in the nominal right view. The pre-kickoff notice identifies the marker family and the later manual locates the markers. The field STEP and robot sources are pinned in the receipts. [FIRST pre-kickoff announcement](https://www.firstinspires.org/community-blog/2025-apriltag-information) · [Published manual §5.8](https://firstfrc.blob.core.windows.net/frc2025/Manual/HTML/2025GameManual.htm) · [Native station source](../station-supply/transfer.json)

## Which sight lines work?

| Arrangement | Result at the 40 mm dock |
| --- | --- |
| Existing release stereo, centered at (−315, 0, 970) mm, 26° down | All four bolt centers project above the 640 × 480 image in both eyes. Keep its registered release function. |
| Initial upward mount at (−43, +222.5, 1058) mm | The status light cuts into the camera envelope by 10,712 mm³ and occludes all four bolt rays. [Failed fixture](rejected-mount.json) |
| Right entrance pupil (−43, −222.5, 1088) mm | All 39 tested rays lie inside the image with zero native blockers. |
| Optional left entrance pupil (+30, +222.5, 1088) mm | All 39 tested rays clear. Both original camera reserves and pedestals clear the receiving robot by native volume intersection. [Four original STEP bodies](camera-reserve.step) |

Point each new optical axis at the center of the four faces. Use a 1456 × 1088 IMX296 global shutter with nominal 6 mm lens and 3.45 µm pixels, nominal focal length 1739 px. `pose.py` defines image +u right, +v down. Native checks use all 545 receiving bodies, the two stop replacements and 47 station occurrences. Each camera sees four face centers and nine panel interior samples at (normal, lateral, heading) equal to (0,0,0), (+10 mm,+10 mm,+3°) and (−10 mm,−10 mm,−3°). This is a sampled test. Players, hardware variations, glare, a pipe already in flight and detector failures can obscure these features. [Raspberry Pi camera and lens guide](https://www.raspberrypi.com/documentation/accessories/global-shutter-camera.html) · [Luxonis release-camera specifications](https://docs.luxonis.com/hardware/products/OAK-D%20Lite)

The mounts are original 42 × 42 × 48 mm camera/lens reserves on pedestals seated at Z 1020 mm. Confirm the actual lens barrel, locking screws, focus, cap joint, stiff mounting, strain relief, cable bend/routing and mass. The existing release observer looks downward from Z 970 mm. A single CSI camera needs a compatible available port, supported cable length and measured host power/latency. Two cameras also need two independent inputs, rigid inter-camera calibration, exposure synchronization and a verified common clock. Nominal 60 Hz frame rate alone cannot establish the 20 ms end-to-end age bound. [Manufacturer hardware and software reference](https://www.raspberrypi.com/documentation/accessories/global-shutter-camera.html)

## How small must the image error be?

The six-axis model solves around the **receiver landing center**. The required independent coarse prior is ±15 mm normal, ±35 mm side, ±3 mm height, ±3° heading and ±0.35° pitch/roll. All four faces must be associated and fully measurable. The interval solver accounts for perspective, height and tilt. It bounds the remainder of the rotations over the entire prior and contracts until the solution is enclosed. The total coordinate error per image axis includes feature fitting, focus, distortion, intrinsics, local landmark geometry and any differential camera calibration or timing error. The additional allocation for the shared landing transform is ±0.50 mm normal, ±0.10 mm side and ±0.020° heading. These are required error ceilings. They include mount flex, panel-to-real-chute registration and feed repeatability. At sample age up to 20 ms, the held relative motion allocation adds 0.04 mm and 0.001° at the pivot, assuming at most 2 mm/s and 0.05°/s.

| Total coordinate error | Nominal right camera | Nominal camera pair |
| --- | --- | --- |
| ±0.05 px | ±0.792 mm normal, ±0.254 mm side, ±0.042° heading; tighter window | ±0.712 mm, ±0.246 mm, ±0.040°; tighter window |
| ±0.10 px | ±1.059 mm, ±0.374 mm, ±0.064°; wider window only | ±0.891 mm, ±0.357 mm, ±0.061°; wider window only |
| ±0.20 px | Neither window established | Neither window established |

Those rows use ideal aligned pixel observations, with uncertainty applied around them. For **every** in-budget observation of the aligned robot, the unknown true and observed rays can differ from the nominal rays by twice the individual budget. The sufficient per-axis budgets then fall to 0.0451 px tighter / 0.0744 px wider with one camera, or 0.0485 / 0.0806 px with two. One small nonzero six-axis case passes the tighter window; a larger geometrically valid offset requires hold after uncertainty propagation. Evaluate the whole uncertainty box every time. [Six-axis solver](attitude.py) · [Cases, plane bounds and adverse inputs](pose-study.json)

A synthetic patch exercise integrates an 11.11 mm circular face over individual pixels at three subpixel shifts. Correcting for the projected conic center brings the maximum right-camera error on ideal uniformly lit disks to 0.0037 px. With a 15% linear intensity gradient, the same estimator misses by 0.615 px. Actual bolt heads, reflections, blur, camera noise and illumination need an independent measured residual bound or a better estimator. [Patch generator](feature.py)

## What must be measured before accepting a feed?

1. Use independently calibrated fixtures to register the camera, both loaded jaws and a reference through the **two chute planes** plus the independent normal datum. Measure the actual delivery trajectory distribution using representative pieces, supply conditions and speeds. Record normal, side and heading errors at the landing pivot. Include instrument uncertainty and repeat after removing/replacing a panel or camera. A 0.60 mm lateral shift of both panel and robot leaves every camera pixel identical while the true receiver violates both windows.
2. Collect held-out images against independently measured six-axis poses across both signs of each travel bound. Include angle, distance, lighting/glare, occlusion, physical panel tolerances, distortion, focus changes and mount deflection. Demonstrate a defensible worst-case total residual under the chosen threshold. If the shared survey or feed repeatability uses more than its allocation, recompute the bounds. The logged entry budget covers relative delivery.
3. Measure capture-to-decision age and motion of the loaded receiver; require same epoch, fresh increasing timestamps, all four faces and the correct station ID. For the pair, require both exposure times within 0.2 ms and reject incompatible rays. Hold during feed/contact, on inconsistent data, or when calibration is unavailable. No command output or physical readiness grant is implemented.

## Reproduce the receipts

Use Python with NumPy for the optical study. The optional native run also requires the project OCP environment and the separately obtained, hash-checked January field STEP. It exports only the four original camera reserve and mount bodies. Generate into a scratch folder to compare with the issued files.

```powershell
python -B station-localization/study.py
python -B -m unittest discover -s tests -p test_station_localization.py
python -B station-localization/native.py --field $env:FIRST_FIELD_STEP --step artifacts/station-camera.step --output artifacts/station-rays.json
```
