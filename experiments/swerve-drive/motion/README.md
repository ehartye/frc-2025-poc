# Four-module opening to the red station

<!-- cad-still -->
[![Team 1780 · stowed swerve robot](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · stowed swerve robot.** Red alliance R4 with numbered bumper covers. Purchased parts use project-created representations in this public assembly. [Source STEP · abbd8ab838](../../../cad/full-swerve-r4/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

## First loaded swerve departure

The receiver follows the corrected signed opening poses into the same first station feed at 21.10 s. The field robot is held stationary during feeding. Its last incoming wheel trace bounds the residual chassis and wheel rates at 10⁻⁵ m/s or rad/s. At 27.035 s the five-frame stereo decision can enable a short combined reverse and lateral motion on the four real module centers. The target ramps to 0.15 m/s reverse and 0.025 m/s lateral in 250 ms, then commands a stop at 600 ms or sooner if custody expires. Wheel-radius error, absolute encoder quantization, yaw scale and bias remain signed. The chassis carries coral-A's 0.816466 kg at its attained center. Tire/contact and bus calculations assume the same supported tread conditions as the opening.

The moving simulation preserves tube pose, velocity and momentum, 21 shelf/carriage coordinates and rates, both plungers and both five-mode guides. The stowed slide and zero belt command stay engaged. The contact solver takes the attained four-module chassis acceleration, yaw rate and yaw acceleration on 5 ms intervals. The effective chassis mass and inertia include the piece as a lumped load. Small cargo motions change its force distribution within the receiver. The drive does not feed those internal transients back into the wheel model. Stereo uses the registered public stowed body, two calibrated optical windows, the actual field station and a 0.05 mm guarded facing. The public wheel artwork shows the modules pointing forward. The [all-angle visibility check](wheel-visibility.json) removes those six depictions per corner, projects all 300 supplier sweep envelopes and marks any pixel that might see moving material. A five-pixel neighborhood also excludes adjacent edge and registration samples. The fixed estimator drops any scanline whose uncertain region reaches the observed pipe or its three-pixel edge witness. The roof estimator masks 5,792 left and 6,601 right pixels in the images and their reference depths. Actual pupil calibration, surface reflectance and untested lighting remain assumptions.

Each 17 ms exposure must describe the same pipe on both cameras. Delivery takes 15 ms and the controller runs every 5 ms. An invalid frame or one more than 40 ms old latches a stop. The simulation reruns the actual wheel and pipe braking path from the first rejected service time. The matched-brightness and late station controls withhold the initial permit. The missing-eye challenge tests a packet arriving after motion has begun.

With the same pipe aboard, the chassis moved 71.79–71.80 mm during the nominal 600 ms command and finite brake. Each of the four simulated trials retained optical custody in all 88 moving exposures. Relative to the receiver, its center moved at most 0.249 mm in the four contact runs. Independently integrated wheel rotors, absolute azimuth and gyro measured field displacement within 0.087 mm of chassis truth. Losing one fixed eye on the first moving capture commanded braking at 27.065 s, after 30 ms. Total displacement from departure, including the coast, was at most 0.225 mm. The nearest modeled field barrier, stock and peer stayed 9.86 mm, 354.46 mm and 4.04 m away over the departure. The next decision is whether the moving receiver can retain an independently observed pipe over a registered transit to the reef.

[Loaded wheel, camera and cargo results](loaded-departure.json) · [Corrected station feeds](loaded-feed.json) · [Both stereo heads and matched controls](loaded-witness.json)

To reproduce the departure, place the eight corrected wheel CSVs under `$wheels` and use a working directory for the local contact traces, masks and images. The public register contains source hashes. The field STEP and the local JDK are supplied separately.

```powershell
foreach ($side in 'plus','minus') {
    foreach ($us in 31.25,15.625) {
        python feed.py $jdk $wheels $raw $side $us --opening energy.json --out "$raw/$side-$us.json"
    }
}
python loaded_departure.py collect $raw $feeds --split $work
python wheel_visibility.py "$work/wheel-visibility.json"
python receiver_witness.py $raw $field $witness --opening energy.json --feed-register $feeds --wheel-mask "$work/wheel-visibility.json"
python receiver_witness.py $raw $field "$work/whole-row-control.json" --opening energy.json --feed-register $feeds --wheel-mask "$work/wheel-visibility.json" --whole-scanlines
```

For each signed contact timestep, the service checkpoint precedes the wheel, contact and image calculations. A zero-duration wheel command reproduces the denied departure. The 30 ms lease reproduces the missing-eye stop. The 600 ms command is the nominal trial.

```powershell
python loaded_departure.py service $side $us $raw $feeds $witness $work
foreach ($ms in 0,30,600) {
    python loaded_departure.py wheels $side $us $raw $feeds $witness $work --wheels $wheels --lease-ms $ms
    if ($ms -gt 0) {
        python loaded_departure.py pipe $side $us $raw $feeds $witness $work --lease-ms $ms
    }
}
python loaded_departure.py vision $side $us $raw $feeds $witness $work --field $field --wheel-mask "$work/wheel-visibility.json"
```

Rebuild the illustrated-forward control using the per-case subset made by `--split`, then collect the complete register once every case has run.

```powershell
$trial = Join-Path $work "$side-${us}us"
python receiver_witness.py $raw $field "$trial/station.json" --opening energy.json --feed-register "$trial/feed.json"
python loaded_departure.py vision $side $us $raw "$trial/feed.json" "$trial/station.json" $work --field $field
python review_departure.py $work $feeds $witness $result
```

The compact register preserves the separate native-image and attained shaft, wheel, tube and structure hashes.

## Signed motor and transmission replay

The [primary match qualification](match-qualification.md) checks the actual bumper covers on the starting line, the attained LEAVE score and the evidence required for two deliveries and a park.

The current opening controller uses the [reciprocal motor and four-corner energy replay](energy.json). At 65 kg it reached the station at 20.285 and 20.330 s for the two signed tire and gyro variations. At 72 kg it reached at 20.180 and 20.245 s. Missing or late observations, a stalled steer axis and a misaligned start each withheld the feed. The closest represented field barrier in the four reached cases was 7.93 mm away. All eight paths used the original stock, other five robot reservations, stowed receiver loads and 21.10 s feed-slot deadline.

Both motor types use one SI constant for back EMF and torque, winding resistance from the catalog stall current, and viscous drag fitted at free speed. This predicts 2.083 N·m drive and 1.027 N·m steer stall torque versus catalog 2.60 and 0.97 N·m. The curves between these points, gearbox efficiencies, phase limits and tread support are prebuild assumptions. A backdriven gearbox now dissipates energy when the wheel powers the motor. Drive-to-azimuth reaction uses the same signed coupling as rotor speed. The earlier 0.2 N·m, 100 rad/s drive example had an unexplained 3.973 W surplus. The corrected run accounts for shaft work, winding heat and drag at each of eight motor ports every millisecond. Across the four signed openings the minimum protected drive or steer voltage was 11.28 V on the 11.8 V, 32 mΩ pack. The supply screen charges the magnitude of braking electrical power and takes no regeneration credit. The drive simulation uses its fixed 9.5 V authority level.

The historical student film and custody study below use the original motor revision. The new 65 kg positive terminal moved 0.0165 mm in station-normal position. Each new pickup must consume its own signed wheel trace and actual pipe trajectory before it can support loaded departure or a scoring decision. To reproduce the corrected register, run from this directory:

```powershell
foreach ($case in 'plus','minus','heavy','upper-minus','blind','late','jam','misaligned') {
    python opening.py $artifacts "$raw/reciprocal" --case $case
}
python opening.py $artifacts "$raw/historical" --case plus --model historical
python energy_review.py $raw $pluginRoot $outputJson
```

[Open the eight-trial student replay](index.html) to see the achieved wheel directions, field path and competing reservations from kickoff through the 21.10 s first-feed slot. The 65 kg trials confirmed the station at 20.285 and 20.330 s. A 72 kg test with higher yaw inertia also reached the station on both signed calibration cases.

| Case | Station confirmation | Achieved normal / lateral / yaw at 21.10 s | Minimum barrier / stock / peer clearance |
| --- | --- | --- | --- |
| 65 kg, positive errors | 20.285 s | +0.45 mm / −0.06 mm / −0.001° | 9.88 / 72.12 / 39.04 mm |
| 65 kg, negative errors | 20.330 s | +0.66 mm / −0.13 mm / −0.009° | 10.14 / 57.30 / 41.30 mm |
| 72 kg, positive errors | 20.195 s | −1.35 mm / −0.09 mm / +0.016° | 7.94 / 71.58 / 39.46 mm |
| 72 kg, negative errors | 20.245 s | −1.41 mm / −0.05 mm / +0.009° | 7.94 / 56.89 / 41.64 mm |

The field animation stops at the 21.10 s feed slot. A separate simulation starts coral-A there and carries the achieved signed 65 kg chassis poses into a stationary receiver. The actual port controller reaches IN_BAND at 22.911673 s and holds through 27.031673 s at both contact time steps. The earlier positive calibration run faulted at Java 1,020 ms under the same controller. The R4 camera study below follows those four feeds through the decision to begin loaded travel. [Four physical/contact clock records](first-feed.json)

## Can the robot leave the station with coral-A?

At **27.035 s**, the first eligible swerve controller tick, the fixed and roof stereo heads agree that coral-A is seated in both simulated 65 kg approaches. Each decision uses five synchronized image pairs per head. The two contact solver steps, 31.25 and 15.625 microseconds, repeat each approach. These are two starts. The checks use the straight guides, fixed-height receiver and the existing gyro-referenced dock poses. [Camera records](receiver-witness.json) · [Mechanical state at first service](first-service.json)

| Condition at station | Positive approach | Negative approach |
| --- | --- | --- |
| Contrasting shelf, 15 ms packet delay, nominal geometry | 5 of 5 paired captures qualify | 5 of 5 qualify |
| Same delay and a 0.05 mm shelf perturbation bound | 5 of 5 qualify | 5 of 5 qualify |
| Shelf region with the same brightness as the pipe | Hold because the roof loses the rear edge | Hold because the roof loses the rear edge |
| 45 ms delay on the guarded case | Hold because only 3 of 5 arrive in time | Hold because only 3 of 5 arrive in time |

Each image comes from an attained pipe pose, the actual signed station transform and the separately reopened R4 public robot, including its 975 registered solids. Each head sees the other camera's housing. Rays originating at the two registered pupils omit that head's own opaque public housing envelope. The fixed estimator measures the pipe's position. The roof estimator independently bounds its position and yaw. Field-only rays supply no robot reference for the roof. The five observations at Java 5372–5440 ms are available by the decision tick with 15 ms transport. The next image at 5457 ms becomes available at wall 27.045 s. A missing eye, an empty shelf, an 18 mm sideways displacement and a 7 degree yaw challenge all withhold permission at both refinements on both starts.

For the 3.327 ms between the saved feed state and first wheel service, the contact solver retains the same finite pipe, its linear and angular state, both guide modal histories, the 21 carriage and shelf modes and both plungers under the held zero-belt command. Across the four restarts, the largest center displacement is below 0.00032 mm. The chassis is stationary in this interval. The outgoing checkpoints can seed the next loaded-motion experiment.

The evidence ends before wheel actuation. It credits zero deliveries. It assumes surveyed pupils, modeled surface brightness, the represented stowed geometry, 15 ms camera transport and one uninterrupted delivery of coral-A. The cameras measure shape and position; the station simulation supplies the piece identity. Conditioning on finish, timing and the stationary chassis keeps this result separate from departures under power and from inspection or as-built behavior.

From the repository root, replay the original swerve feeds into a local directory, then run:

```powershell
python experiments/swerve-drive/motion/receiver_witness.py $feedDirectory $officialFieldStep $cameraResult
python experiments/swerve-drive/motion/first_service.py $feedDirectory $checkpointDirectory $serviceResult
```

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
foreach ($case in $cases) { python opening.py $artifacts $rawOutput --case $case --model historical }
python publish.py $rawOutput --out $reviewOutput
python check_module_map.py $normalizedInput
node "$env:AGENT_FRC_PLUGIN_ROOT/examples/swerve/check-encoders.mjs" $normalizedInput
python check_module_map.py $rawInput --raw
node "$env:AGENT_FRC_PLUGIN_ROOT/examples/swerve/check-encoders.mjs" $rawInput
```

With Agent FRC 0.24.1, the raw check reads two synchronized 20 ms shaft and absolute-encoder frames from each corner. The maximum module-vector error was 0.001662 m/s during diagonal translation, 0.002966 m/s at a front-left pivot and 0.002126 m/s during sideways travel. The tolerance is 0.003 m/s. Inverting the front-right drive polarity with `--wrong-polarity --raw` makes the checker fail. [Raw encoder fixture](raw-encoders.json) · [Installed positive and reversed-channel results](raw-review.json)

[All eight clock receipts](opening.json) · [Project encoder input](encoders.json) · [Finite contact plant](plant.py) · [Controller and field registration](opening.py)


[Compare the loaded transit paths, the two observed stops and the retention decision](../transit/README.md).
