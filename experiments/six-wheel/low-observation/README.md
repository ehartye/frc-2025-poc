# What the fixed-height camera can authorize

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 14649626c7](../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

The desktop controller independently corroborated both seated pipes **100 ms after the port indication**. Its evidence stream ends before either drive starts. Neither run establishes the same pipe’s identity through travel, extension and release. The operational result is **zero authorized placements**. The station position checks and the zero-dwell withdrawal trials below identify two separate observations that can be tested on the candidate.

## Follow the two pipes

Times use TELEOP seconds, registered to the original first trough contact. A and B are the nominal −180 and +220 mm sectors. The lower carriage stays at 560 mm. Both attempts use the issued 0.65 m/s counterfeed and the held 0.90 m/s release ramp. [Starting states and original contact model](../design/fixed-height/README.md) · [Frame intervals and time origins](results.json)

| Event | A | B | Evidence |
|---|---:|---:|---|
| Pipe at station lip | 3.671 | 54.073 | Modeled release |
| Ports enter IN_BAND | 5.031 | 55.434 | Actual port state machine operating on simulated samples |
| Independent seated position | 5.131 | 55.534 | Five same-epoch stereo exposures after IN_BAND, spanning at least 68 ms |
| Drive begins | 9.032 | 59.434 | Scheduled path, custody signal already stale |
| Drive stops at target | 25.509 | 75.599 | Last moving route sample |
| Extension starts | 27.009 | 77.099 | Slide moves 0–370 mm over three seconds |
| Release belt starts | 30.009 | 80.099 | Hypothetical upstream release |
| Pipe leaves powered shelf | 30.176 | 80.266 | Modeled last powered support |
| Entire finite pipe clears shelf by 1 mm | 30.199 | 80.289 | Modeled shelf geometry |
| Pipe first contacts trough | 30.283 | 80.373 | Integrated planar contact |
| Slide held, receiver touches pipe again | 30.850 | 80.940 | 566.4 ms after trough contact |

The camera and pad quantities stay unchanged. The two registered empty scenes rebuild the 545-body red source, machine six original parts, and add 32 lock bodies. Both have 577 bodies. The test renders the full pair of mono frames from the finite tube and robot meshes, then supplies only grayscale images, the matched empty scene, and optics to the estimator. The station clock accepts only the retracted low configuration’s mesh digest. Each control receives fresh frames from the wrong attempt, the older high pose, an empty scene or an uncalibrated sensor as appropriate. With samples interrupted, A and B lose corroboration on local application ticks 1620 and 1640 ms. [Issued sensor and uncertainty model](../stereo-custody/README.md) · [Actual WPILib desktop replay](hal.json)

**Drive and extension hold.** The low port run stops at local 1700/1720 ms, 3.7605 s before scheduled chassis movement. A changed field behind the moving robot invalidates its static empty-scene reference. Using only rows with the complete silhouette and a two-pixel guard on robot-backed rays retained just two positive samples in each 250 ms survey (66 and 65 samples over 16.477 and 16.1645 seconds). That evidence cannot establish continuous identity. A second three-second blind interval spans slide extension because only the endpoints have registered images. At the fully extended start of release the fitted bottom interval reaches −16.65 mm, outside the ±12 mm limit, on both attempts. No elevation, drive, slide extension or belt release is enabled by this study.

## A measurable withdrawal condition

Once the entire pipe is independently observed beyond the most forward robot material, retraction can begin without waiting for a second geometric clearance. The front of the 577-body model in robot millimetres is the maximum of the bumper at 425, middle stage at 160 plus half the stroke, and the receiver shelf at 165 plus the full stroke. The calculation includes the rotating pinion bounds, draw wire and all machined and added lock parts. At 370 mm extension the front is 535 mm. The proof requires the measured lower centre X minus the projected pipe half-width to exceed this front by 1 mm with another 1 mm for front registration. It assumes the transverse-axis error stays within 1°, a 0.5 mm radius error, and the issued estimator’s 1 px contour error. The centered trial has negligible yaw at release. A new orientation or systematic camera error needs a new silhouette bound.

| Window on first proved pre-contact exposure | A | B |
|---|---:|---:|
| Paired image proves clearance, local ms | 26695–26711 | 26402–26418 |
| Deadline tick under a 50 ms wait and zero actuator lag, local ms | 26740 | 26440 |
| First command tick with 50 ms wait, local ms | 26760–26780 | 26460–26480 |
| Eligible 50 ms phases | 0 / 17 | 0 / 17 |
| Eligible immediate phases | 11 / 17 | 12 / 17 |
| Eligible immediate command ticks, local ms | 26720 | 26420 or 26440 |

The oldest possible prior contact comes from a hypothetical healthy pad edge at the final finite-pad opportunity, with 2 ms for scanning and 1 ms for clock error. The 17 phase variants sample paired exposures every 17 ms with 15 ms delivery. The isolated immediate gate accepts the first pre-trough-contact proof only, using the earlier 401-body trial’s 7–29 ms packet age and allowing up to 10 ms before motion within the 150 ms contact deadline. It binds the lower robot’s geometry separately. An interrupted track disables every phase. Applying these cases through the real 20 ms WPILib scheduler reproduced 23 timing-only windows and zero permissions with identity unavailable. [Phase timings](immediate.csv) · [Historical controller window](../competition-controls/src/main/java/org/reefscape/controls/WithdrawalTrial.java)

The coupled model reruns every integer actuator lag from 0 through 10 ms at the three eligible command ticks. All 33 onsets maintain at least 32.97 mm modeled forward clearance and two seconds of paired-floor/lip observation. The front comparison runs at every 0.2 ms contact step and from motion onset through first target contact when that onset precedes it. Retraction follows the same 150 mm/s stroke as the settlement integrator. The 110 ms truth-triggered baseline separately achieves sustained support, and holding the slide produces recontact on both attempts. [All rerun onsets and native front limits](immediate.json) · [Held-slide and 110 ms controls](contacts.json)

For a physical trial, the independently observed pressure-pad edge, installed camera uncertainty, surveyed front, pipe dimensions and orientation, delivery bound, actuator onset and continued identity all require measurements. The current image backgrounds omit field structures, other robots and students. A matching stationary field reference or independently segmented calibrated scene is necessary even at the station. The single camera pair cannot distinguish identical pipe substitutions across an unobserved interval. No field points or hardware permissions follow from the counterfactual clock.

## Reproduce

Use the project’s OCCT/VTK, NumPy, Pillow, desktop JDK and cached Gradle dependencies. Set `JDK` and `GRADLE_HOME_CACHE` to the local installations. The scratch folder contains reconstructible grayscale/depth planes and submillisecond plant trajectories.

```sh
python low-observation/prepare.py --scratch artifacts/fixed-height-observation --java-home "$JDK"
python low-observation/evaluate.py --scratch artifacts/fixed-height-observation
python low-observation/contact.py --java-home "$JDK"
python low-observation/immediate.py --java-home "$JDK"
python low-observation/verify.py --java-home "$JDK" --gradle-home "$GRADLE_HOME_CACHE"
python -m unittest discover -s tests -p test_low_observation.py
```
