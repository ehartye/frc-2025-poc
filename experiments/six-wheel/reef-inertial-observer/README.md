# Field approach with one visible fastener

<!-- cad-still -->
[![Team 1780 · stowed swerve robot](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · stowed swerve robot.** Red alliance R4 with numbered bumper covers. Purchased parts use project-created representations in this public assembly. [Source STEP · abbd8ab838](../../../cad/full-swerve-r4/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

The intact guide arm can stay on the robot. In two signed desktop wheel trials the existing fixed stereo camera and chassis gyro brought the stowed robot to sector A. The three-frame decision bounded the smaller-left-wheel robot within **8.02 mm and 0.697°** and the smaller-right-wheel robot within **6.81 mm and 0.572°**. The corresponding match clocks were 47.862 s and 47.792 s. Both robots stopped at 48.162 s. [Open the student replay](viewer.html) · [Controller receipts](results.json)

| Sensitivity | Smaller left wheel | Smaller right wheel |
| --- | --- | --- |
| Registered field and fresh sensors | Approach accepted | Approach accepted |
| Missing gyro, stale camera, missing fasteners, duplicate detection or wrong tag | Brakes and holds | Brakes and holds |
| 12 mm sideways disturbance | Brakes and holds | Approach accepted |
| Common 14 mm lateral error in the field map | False alignment at +14.18 mm | False alignment at +13.78 mm |
| Station custody on the recovered two-stage carriage | Five guarded observations at each time refinement | Zero guarded observations at either refinement |

**Neither trial authorizes a placement.** A shift shared by both field references can move the true goal while leaving the sensor-to-feature relationship unchanged. Gyro rate cannot observe this common translation. The field grant requires an independent survey of the installed targets and camera relative to the robot and real field datums. Both branches also require continuous pipe observation on these wheel histories. The right branch lacks the preceding guarded station handoff. The separate south leave-and-park role retains its own five-point opportunity. [Joined decisions](decisions.json) · [Conditional south role](../station-approach/README.md)

## How the robot measures its pose

The fixed camera first reads AprilTag 10. Two separately identified bolt faces then anchor heading. Their surveyed heights turn the visible centroid patches into bounded horizontal rays. The observer intersects heading intervals from different landmarks. For the following 1.8 s, it transports each anchor using the existing camera's onboard IMU. At the target, NAUO1187 is visible in both fixed pupils. A fresh view of that one fastener bounds the robot center only while the previous heading enclosure remains available. Single-pupil observations can steer the moving robot. The final interlock needs both pupils and three distinct stationary observations. [Unchanged installed hardware](../mounted-guide-vision/README.md) · [Camera and gyro estimator](observer.py)

The desktop plant samples chassis yaw at 100 Hz independently of the 35 ms camera exposures. The gyro delivers quantized relative rotation after 20 ms. The camera delay is 15 ms. At camera service the estimator extrapolates from the last two available gyro samples to the capture time. The interval includes quantization, the sensor-age bound, a 1.5 rad/s² angular acceleration cap, ±0.02°/s residual bias and ±0.1% scale uncertainty. Each selected wheel trajectory stays within the angular acceleration cap. The 0.001° integral quantum is an assumed interface target. The fitted camera axes, BMI270 availability on the actual unit, timing, optical error, tire contact and loaded brake performance require measured acceptance. Camera dependence and coarse registration come from the prior station study. [IMU clock model](rate.py) · [Historical camera and IMU assumptions](../station-landmark/README.md)

The extraction keeps the guide arm, the other 55 added guide articles, the opaque original masts, fixed harness bounds and pipe enclosure. A complete known window around each visible bolt face guards against a biased centroid. The optical interval admits ±0.18 pixel plus 0.0005 pixel for recording the centroid. The pre-existing interlock adds the 1.85 mm and 0.14° project allowance and the measured wheel transport from capture to the decision. The native guide and established conservative receiver envelope leave at least 134.86 mm to the mounted ball, 375.85 mm to other installed stock, 41.16 mm to barriers and 1,298.15 mm to moving peers across the two nominal approaches. This sweep covers the final stowed path, including the earlier rectangular guide bounds. The placement extension and downstream interactions require separate qualification. [Field occupancy](clearance.json) · [Finite drive controller](../sector-alignment/dock.py)

## Reproduce

Use the registered project artifacts and the existing Python OpenCV, NumPy and Open CASCADE environment. The native body study reads the tracked guide exchange and the independently registered wheel/field data.

```sh
python reef-inertial-observer/replay.py "$ARTIFACTS"
python reef-inertial-observer/clearance.py "$ARTIFACTS"
python reef-inertial-observer/decision.py
python reef-inertial-observer/student.py "$ARTIFACTS"
```
