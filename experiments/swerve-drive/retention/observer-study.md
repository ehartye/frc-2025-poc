# Carrying a station-fed CORAL to the reef

Both signed swerve routes stop with the same identified station-fed pipe still in the receiver in the 31.25 μs simulation. The longer gauge preserves the four-pin capture during transit. The scoring stroke stays inhibited at both endpoints because the observed chassis positions miss the intended 54 mm setback on opposite sides. The earlier independent trough test and the five-point empty fallback remain separate. [Original capture and route](study.md) · [Loaded motion and fault results](held.json) · [Reef registration](reef-fix.json)

## What reaches the reef

The chassis starts at each attained loaded stop from the first carry. Four 8 mm diameter tubes enter the opposing bores, with 35 mm yoke travel. Each end also has a spring-loaded rim feeler. The five-service capture, four wheel states, gyro, pipe momentum, guide modes and 21 shelf modes carry through the original route. The spring stays at 0.45 N/mm. The revised feeler has 8 mm of travel and the same force law throughout the earlier working range. The simulation continues the previous state exactly where its narrower indication first rejected a 4.6 mm reading. The two signs challenge separate calibrations of one robot and each carries its own instance of coral-A. [Original narrow-gauge negative](carry.json) · [Continuation solver](held.py)

A single longer feeler or a closed yoke with two blocked shelf ports can each accept a 5 mm calibration error. The combined check also compares the two measured end positions against the finite 301.625 mm pipe length and 296 mm free-tip separation. It applies two 0.20 mm gauge allowances, up to 3° end-axis tilt, the aperture observations, an earlier verified identity and a 3.5 N axial-force limit. The two independent ports sense occupancy. The yoke travel switches sense closure. The rim gauges measure load and check the fitted length. Vanishing cargo, a stuck pair of ports with empty gauges, missing beam, open yoke, stale report, overtravel or excessive single-gauge bias withholds continuation. Simultaneous coherent sensor failures remain outside these fault cases. [Measurement and controls](retained_report.py)

The open-yoke file contains 26 project-built solids for the two sides, including bored tubes, foot mounting holes, guide and pinion stock, and supplier-based opaque servo envelopes. They move with the outer slide. The included horn and screw are reserved geometrically. The drawing still needs cut rack teeth, servo attachment, sensor electronics, wiring and fasteners. Its estimated 114.89 g plus the cam's 44.53 g uses all but 10.51 g of the earlier 169.93 g upper inspection allowance before these remaining items. The operating case uses the issued 65 kg empty drive model plus one 0.8165 kg pipe. [Open CAD](open-yokes.step) · [Part register and exclusions](open-yokes.json) · [Servo drawing and dimensions](https://www.hitecrcd.co.kr/wp-content/uploads/hitec-products/2019/c/HS-65MG_Specsheet%28v2.1%29_22L.pdf)

At the station, the open part envelopes and their reopened solid geometry were rendered against the original four signed and time-refined pickups. A gray level of 65 on these parts preserved all 24 occupied four-camera observations. A matched-brightness adversary at 215 and an uncalibrated occluder mask both held. Empty scenes granted no pickup. This is a lighting and geometry condition for the proposed installation. The earlier moving-camera dropout remains what first stops the route for physical capture. [Rendered open solids](native-view.json) · [Conservative stock envelopes](station-view.json)

## Why the robot holds at the dock

The current fixed camera decodes surveyed AprilTag 10 in one eye on 29 and 31 approach frames. At a stop the same near bolt on the reef is visible in both pupils, even after adding the bore guides, a full-pipe envelope and all-azimuth wheel masks. The tag establishes the landmark identity upstream. A 5 ms gyro and shaft history transports its planar pose while an independent stopped-bolt observation constrains the final position. The acceptance test includes 0.25 px corner error, a 1 mm translation and 0.05° yaw registration allowance, 0.02°/s gyro drift, 0.1% rate scale and 12 mm wheel-translation error over the last three seconds of accepted images. Lens height, pitch, feature survey and camera intrinsics follow the tested calibration. An incorrect field identity, an ambiguous second centroid, or a missing terminal pupil withholds this specific fix. [Source and geometry](dock_view.py) · [Monocular tag fit](mono.py) · [Bounded bolt association](reef_fix.py)

| Calibration | Measured stop, match s | Reef X enclosure, mm | Target, mm | Release |
| --- | ---: | ---: | ---: | --- |
| Plus | 55.050 | −70.42 to −62.86 | −54 | Hold, too far back |
| Minus | 54.815 | −40.47 to −31.87 | −54 | Hold, too close |

The camera correction must move in opposite directions on these two cases. A second image at the same pose cannot correct the displacement. The next scoring test starts with each attained same-pipe checkpoint, drives to a corrected observed pose and checks the actual bore withdrawal, loaded slide, trough contact, separation and assessment clock. The narrower signed position and heading bounds, finite target clearance, available stock and peer occupancy belong to that run. This result earns zero L1 deliveries. [Independent scoring fixture](../score-stroke/study.md) · [Student film with checked chapter boundaries](https://ehartye.github.io/frc-2025-poc/)

## Reproduce

Use the original study's field, image and wheel artifacts. Paths below are relative to the repository. Set `$A` to the local artifacts directory and `$R` to `experiments/swerve-drive/retention`. The full field and manufacturer originals stay in the local source archive.

```powershell
python "$R/yoke_cad.py" "$R/open-yokes.step" "$R/open-yokes.json"
python "$R/station_view.py" $A "$R/native-view.json" --native "$R/open-yokes.json"
python "$R/station_view.py" $A "$R/station-view.json"
foreach ($sign in 'minus', 'plus') { python "$R/held.py" $A $sign }
python "$R/retained_report.py" $A "$R/held.json"
python "$R/dock_view.py" $A "$R/dock-view.json"
python "$R/mono.py" $A "$A/captive-observer/mono.json"
python "$R/reef_fix.py" $A "$A/captive-observer/mono.json" "$R/reef-fix.json"
```

To regenerate the student film's chapter boundaries, run `python experiments/swerve-drive/score-stroke/film_chapters.py` and then `node <agent-frc>/scripts/replay-chapters.mjs experiments/swerve-drive/score-stroke/receiver-chapters.json .` from the project root.
