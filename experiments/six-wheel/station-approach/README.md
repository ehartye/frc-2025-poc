# Opening AUTO with wheel calibration error

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

**Select the reserved south-lane leave and park before the match if station alignment is unavailable.** In the desktop trial the red robot travels 543 mm, settles in 2.56 s and holds with its carriage stowed. This preserves a conditional opportunity for three LEAVE points and two PARK points. It gives up both station feeds and both L1 placements, worth four points on the earlier nine-point clock. A partial stop never authorizes a feed, continuation toward the station or automatic scoring. The earlier field replay shows the same lane and endpoints with its original 1.345 s drive. Select “Stop closer to the tape” to inspect the cages and partner positions. [Open the field replay](../leave-park/replay.html)

The [lower-panel follow-up](../station-landmark/README.md) uses a different landmark on the existing stereo head and checks the actual pipe after the robot stops. At a 250 µs physics step, five of seven signed drive cases achieve both port indications. The [contact refinement and port replay](../station-feed-margin/README.md) found unresolved pickup outcomes under both opposing wheel errors.

## Why the opening station approach is held

The registered drive ends 10 mm from the station in the ideal case. With opposing 0.2% rolling-radius errors its bumper intersects the actual lower station wall at 13.045 s. A 2 mm track error also produces contact, at 13.295 s. The current study stops reading each adverse trajectory before its first observed intersection. [Bumper and station witnesses](../selected-drive/dock.json)

The fixed stereo head sits 315 mm aft of chassis center at Z 970 mm and looks 26° down. Projecting the four surveyed station fasteners into both rectified 640 × 480 eyes at each 5 ms pose gives zero poses with all four visible on each of the three approaches. During the last half-second every corner of the official ID2 panel stays at least 249 pixels above the active image. Body occlusion could further reduce visibility. This excludes the existing four-fastener measurement with this camera orientation. Other lower field features would need their own correspondence, occlusion and physical accuracy evidence. [Source camera model](../sector-alignment/vision.py) · [Surveyed station geometry](../station-localization/sightlines.json) · [Camera replay](results.json)

The native GE-25007 wall face reaches 304.8 mm above the carpet. In a centered-heading projection of its long top edge, shifting the robot 20 mm left or 20 mm right yields the same row, about 360.5 px. In both cases the ends extend beyond the image sides. Even a perfect horizontal-edge measurement cannot settle which lateral placement occurred. Actual contrast, installation, camera pupils and calibration remain unmeasured. [Native wall and bumper check](field.json)

## The station-independent role

Red 1 must reserve the middle start, leaving Red 2 at X 393.375 in, Y 95.5625 in, facing field −X. Red 2 starts empty and remains mechanically stowed. Each motor encoder reports shaft rotations through its 20:1 reduction. The desktop position loop reads paired, timestamped rotations every 10 ms, aims for 0.542925 m, and limits its demanded speed and acceleration to 0.35 m/s and 0.45 m/s². A 5 ms plant advances the robot independently using the selected wheel scale and effective track. This is a software model of feedback and braking. A physical encoder measures wheel rotation indirectly, so slip or two equally erroneous counts can survive the pairing check. [Drive geometry and battery studies](../selected-drive/README.md) · [Controller and plant](station_trial.py)

| Condition | Trial evidence |
|---|---|
| Opposing or common ±0.2% radius error, 578 or 582 mm track, fixed 5/15/25 ms packet delay | All 24 combinations complete. Maximum shaft-distance endpoint error is 0.53 mm. |
| No first sample, stale samples, missing fresh samples, or 40 mm side-to-side count disagreement | Thirteen cases latch a stop. With missing data from enable the chassis stays at rest. A moving robot coasts up to 136.12 mm after inhibition under the assumed braking. |
| Encoder fails during a braking approach | At maximum speed the freshness threshold, one service period and finite braking allow up to 152.17 mm from the last fresh acquisition. A latched fault cannot restart the role. |
| ±5 mm initial placement, ±0.4° starting yaw and the modeled wheel errors | The entire hull stays within 17.90 mm of the reserved corridor. Tape clearance at the confirmed stop exceeds 49.22 mm. |
| Stock, cages and the other five robots | The 24 installed pieces retain at least 2.051 m of clearance. A 3° hanging-hardware cone leaves 45.68 mm and the planning exclusion retains 7.95 mm. Every point in the entire stop corridor clears the five scheduled peers through TELEOP by at least 730.34 mm. |

The two selected red covers reopen from the shop STEP. Eroding the start tape by staging uncertainty still leaves over 9,009 mm³ of cover inside it. Eroding the end zone by the entire tracking bound leaves over 195,600 mm³ inside at the confirmed stop. A stopped robot remains in the reserved corridor after a sensor fault. Its final scoring intersections and published outcome remain unassessed on fault. [Bumper and traffic evidence](field.json) · [Actual final cover and native field source](../leave-park/near-tape.json)

The corrected shop geometry fits the historical 850 × 850 × 1,059 mm box. The route allows an extra 7.8 mm of height for fixed-head connector routing, up to a 1,066.8 mm measured stowed height. Subtracting the whole 7.8 mm from the historical 3-D field and stock distances conservatively covers that enlargement. The full-height cage projection already includes it. Inspect the fitted connector and strain relief, and recheck any change to the footprint. [Corrected stowed inventory](../selected-match/corrected-clearance.json) The selected drivetrain study includes a 65.61 kg empty mass case with this speed and acceleration envelope and an 11.46 V minimum terminal estimate. It uses modeled motor, battery and load parameters. Before a field run, establish wheel-ground scale and track, loaded mass, stop response including a disconnected controller, holding through the match, fixed head registration, cage motion and actual partner commitments. A failure to brake under lost power sits outside this study. [Drive supply receipt](../selected-drive/results.json) · [Cage sweep](../leave-park/cage-motion.json)

## Reproduce

Supply the two issued steering CSVs in the relieved-match trace directory, the registered shop artifact directory, and the authorized FIRST field STEP used in the original native study. Use the project's NumPy, OpenCV and Open Cascade Python environment.

```sh
python station-approach/station_trial.py "$POC_ARTIFACTS/relieved-match"
python station-approach/field_check.py "$POC_ARTIFACTS" "$FIELD_STEP"
python -m unittest discover -s tests -p test_station_approach.py
```
