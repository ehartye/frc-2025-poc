# Four-module opening to the red station

<!-- cad-still -->
[![Four-module swerve frame and mounting hardware](../../../docs/robot/cad/assembly.png)](../../../docs/robot/cad/index.html)

**Project CAD · Four-module swerve frame and mounting hardware.** Original robot structure for the swerve candidate. The project-local Fusion assembly adds four manufacturer module occurrences. [Source STEP · f8b75f2df4](../packaging/swerve-frame.step) · [Rotate the model](../../../docs/robot/cad/index.html)
<!-- /cad-still -->

[Open the eight-trial student replay](index.html) to see the achieved wheel directions, field path and competing reservations from kickoff through the 21.10 s first-feed slot. The 65 kg trials confirmed the station at 20.285 and 20.330 s. A 72 kg test with higher yaw inertia also reached the station on both signed calibration cases.

| Case | Station confirmation | Achieved normal / lateral / yaw at 21.10 s | Minimum barrier / stock / peer clearance |
| --- | --- | --- | --- |
| 65 kg, positive errors | 20.285 s | +0.45 mm / −0.06 mm / −0.001° | 9.88 / 72.12 / 39.04 mm |
| 65 kg, negative errors | 20.330 s | +0.66 mm / −0.13 mm / −0.009° | 10.14 / 57.30 / 41.30 mm |
| 72 kg, positive errors | 20.195 s | −1.35 mm / −0.09 mm / +0.016° | 7.94 / 71.58 / 39.46 mm |
| 72 kg, negative errors | 20.245 s | −1.41 mm / −0.05 mm / +0.009° | 7.94 / 56.89 / 41.64 mm |

The field animation stops at the 21.10 s feed slot. A separate simulation starts coral-A there and carries the achieved signed 65 kg chassis poses into a stationary receiver. The actual port controller reaches IN_BAND at 22.911673 s and holds through 27.031673 s at both contact time steps. The earlier positive calibration run faulted at Java 1,020 ms under the same controller. The selected inlet still needs independent pipe imagery and a loaded swerve departure before delivery can be assessed. [Four physical/contact clock records](first-feed.json)

## How this drivetrain moves

The motor/absolute contract applies to four horizontally separated steering axes in FL, FR, RL, RR order. At each corner a 76.2 mm wheel turns through a 5.50:1 drive train. The azimuth ratio is 9424:203. The installed locations have a 564 mm wheelbase and 554 mm track. The controller converts field velocities into robot +X forward and +Y left, selects the shorter steering rotation and reverses wheel speed where necessary. A wheel at the chosen center of rotation holds its previous azimuth. The four wheel targets share one speed limit.

The drive pinion lies on the steering axis. In the model a locked wheel makes one motor input revolution per carrier revolution. The per-corner logical signs and arbitrary absolute zeros are simulated commissioning inputs. Independent polarities and steer-induced rotor turns are removed before applying the drive ratio. The checker exercises simultaneous rolling/steering, an offset pivot and a reversed wheel. The portable vector checks in Agent FRC 0.23.0 and 0.24.0 gave a maximum residual of 0.00221 m/s. A deliberately inverted front-right drive signal failed all three exercises, with residuals of 1.42–1.55 m/s.

A 5 ms field controller requests chassis translation and yaw. A 1 ms plant evolves four steering inertias, drive-wheel inertia, finite lateral tire forces, yaw and translation. The same gear coupling participates in steer torque and rotor speed. Each substep passes achieved chassis acceleration to four constrained tread springs and uses their reactions for the following substep’s tire-force caps. The 65 kg / 5 kg·m² and 72 kg / 8 kg·m² cases share a 380 mm CG height, 26/24/26/24% static shares, front/rear tread rates 90/110% of 150 kN/m, friction coefficient 0.80 and a 9.5 V modeled actuator terminal ceiling. The inner motor loops assume ideal velocity feedback. The odometry channel independently quantizes absolute azimuth to 1,024 counts per turn and rotor turns to 1/4,096 of a revolution as a reporting assumption. Results depend on these hypotheses and on the separately protected auxiliary loads.

## Why the route slows down

The first four-wheel route changed direction sharply near the waypoint. A modeled, statically balanced robot with a 310 mm CG and 90/110% tread rates demanded 59.37 N at its right rear while the available 0.80 tire coefficient yielded 56.82 N. The registered first route remains a failed traction control in the power study. A separate 72 kg trial exposed 3.077 mm of normal-position error after its first lock. The revised controller slows inside the last 35 mm and waits for wheel speeds below 6 mm/s. The selected route limits the field-speed change to 0.62 m/s² and tapers speed as the robot approaches the corner. Its signed 65 kg traces passed the four-tread and pack sensitivities. A minimum 22% static share per corner and the assumed CG and tread bounds leave at least 69.14 N of traction margin across both runs. The start and dock use release-day field and stock geometry. The five independently programmed peers retain their reservations. During the three-second disabled interval the applied motor voltage is zero and finite residual motion stays below 0.004 mm in the four successful runs.

## How the station is observed

The controller combines the ten-bit absolute angles, drive rotor deltas, a separately sampled gyro and 25 Hz rectified images of the two upper holes in GE-25008. Both eyes must yield both holes. The estimator checks the residual, transports the delayed measurement using wheel odometry, and requires at least three consecutive accepted frames plus slow wheels at the target. The gyro and initial station survey remain distinct sources of possible bias. The camera model reuses the existing straight-guide robot depth image and finite panel-hole geometry. The modeled upper-hole ray bundle stays above 821.6 mm, after a 14-pixel association allowance. Compared with the new modules across their full azimuth sweeps, frame, controllers and represented wiring, the smallest vertical separation is 515.5 mm. All registered changed bodies remain inside the 850 mm bumper square. Unmodeled connector tails, physical camera pose, exposure contrast, clutter, wheel diameter and encoder zeros remain commissioning inputs. [Original geometry and ray registration](visibility.json)

The revised rear mounts raise the observer host and its two USB connector envelopes. Their 30 measured native bounds top out at 227.058 mm. The padded station ray bundle clears them by 594.541 mm. The wiring remains the highest registered new obstruction, with 515.539 mm of clearance. [Revised rear geometry replay](rear-visibility.json)

| Injected failure | What the controller observed | Result |
| --- | --- | --- |
| Both panel holes lose contrast | Zero valid fits | Holds 407 mm from the station target |
| All packets delayed to 100 ms | 103 fits arrive beyond the 65 ms age gate | Same wheel trace and hold as loss of contrast |
| Front-left steering mechanically locks | Measured azimuth stalls at a sustained angular error | Latches a stop at 3.190 s |
| Initial surveyed heading is wrong by +0.8° | The expected bilateral landmarks cannot be matched | Holds 367 mm from the target |

## Reproduce the opening

Run these commands from this study directory. The registered camera artifacts are regenerated by the project’s station/guide optics studies or supplied as a local art directory. The program checks the raster SHA before using it. Generated CSVs contain one FL, FR, RL, RR record per service time and carry shaft speeds, tire reactions, both actuator voltages and phase currents. NumPy, OpenCV and the existing project geometry dependencies are required.

```powershell
$cases = 'plus','minus','heavy','upper-minus','blind','late','jam','misaligned'
foreach ($case in $cases) { python opening.py $artifacts $rawOutput --case $case }
python publish.py $rawOutput --out $reviewOutput
python check_module_map.py $normalizedInput
node "$env:AGENT_FRC_PLUGIN_ROOT/examples/swerve/check-encoders.mjs" $normalizedInput
python check_module_map.py $rawInput --raw
node "$env:AGENT_FRC_PLUGIN_ROOT/examples/swerve/check-encoders.mjs" $rawInput
```

With Agent FRC 0.24.1, the raw check reads two synchronized 20 ms shaft and absolute-encoder frames from each corner. The maximum module-vector error was 0.001662 m/s during diagonal translation, 0.002966 m/s at a front-left pivot and 0.002126 m/s during sideways travel. The tolerance is 0.003 m/s. Inverting the front-right drive polarity with `--wrong-polarity --raw` makes the checker fail. [Raw encoder fixture](raw-encoders.json) · [Installed positive and reversed-channel results](raw-review.json)

[All eight clock receipts](opening.json) · [Project encoder input](encoders.json) · [Finite contact plant](plant.py) · [Controller and field registration](opening.py)

