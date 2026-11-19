# Carrying the first pipe toward the reef

<!-- cad-still -->
[![Team 1780 · stowed swerve robot](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · stowed swerve robot.** Red alliance R4 with numbered bumper covers. Purchased parts use project-created representations in this public assembly. [Source STEP · abbd8ab838](../../../cad/full-swerve-r4/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

[Compare the three wheel paths on the match clock](index.html). The four-module robot begins at 28.535 s with coral-A on its stowed receiver. Each trial starts at the true and estimated poses reached in the [first loaded departure](../motion/loaded-departure.json), with the attained steering angles, rotor turns, motor targets, gyro and residual wheel rates. The sign identifies the two opposing wheel-radius and gyro error patterns.

| Wheel controller | Distance traveled | Smallest structural clearance | Smallest stock clearance | Smallest peer clearance |
|---|---:|---:|---:|---:|
| Advance on crossing each waypoint | 9.52–9.68 m | −96.1 mm | −150.9 mm | −13.4 mm |
| Settle before advancing and stop upstream | 8.04 m | 80.7 mm | 188.3 mm | −58.6 mm |
| Settle and shift the north waypoint 450 mm south | 7.71 m | 80.7 mm | 172.7 mm | 150.4 mm |

In the first controller, the robot crosses the northern waypoint at roughly 0.61 m/s and keeps moving as the next command ramps in. The second controller slows with distance and advances only after 50 ms of encoder-derived velocity falls below 18 mm/s. Its original lane still intersects red3's reservation. The shifted lane clears the five other robots, registered floor and mounted stock, station planes, reef and barriers throughout the two wheel trajectories. Its 425 mm half-width and 1,066.8 mm height bound move with the robot. The field checker pads between successive samples and allows for peer motion.

The shifted path brakes through 50.880 s, 300 mm upstream of the original scoring dock. Odometry errors reach 30.5 mm on one signed path and 20.3 mm on the other. The 11.8 V, 32 mΩ battery model, with both cameras and the occupied receiver, leaves at least 11.352 V at the eight drive and steer controllers. The path includes the modeled pipe mass and the original tire-load, inertia and friction assumptions. A distinct end approach must register against the scoring face and the actual shelf clearance. [Wheel paths, pose histories and per-object clearances](corridor.json)

## Cargo and observation

### Should the robot require both cameras throughout transport?

**Keep the brake. The current observations support testing a dedicated mechanical retention witness before tuning the fixed camera further.** The proposed collision-clear wheel route extends past both observed stops. Each branch still has three modeled points for leaving the starting line. Neither branch has delivered a pipe or reached a verified parking pose.

| Signed wheel error | First invalid exposure | Brake command | Measured stop | Pipe movement before that exposure | Fixed estimate for height above shelf |
|---|---:|---:|---:|---:|---:|
| Positive | 31.041 s | 31.060 s | 31.960 s | 0.456 mm | −12.275 to +7.725 mm |
| Negative | 29.205 s | 29.220 s | 30.120 s | 0.229 mm | −12.525 to +7.725 mm |

Times are on the shared match clock. At the 31.25 µs contact resolution the first invalid images are compatible with a seated pipe. The true bottom is within 0.037 mm of its nominal height on both exposures, the roof stereo still bounds the pipe, and the fixed stereo crosses the −12 mm acceptance limit. Its measured silhouette uses only rays backed by known R4 material, after removing pixels that moving wheels might obscure. Numerics matter near that threshold. On a separately propagated 15.625 µs positive branch the optical brake comes at 30.240 s. These outcomes make a repeatable case for a visibility problem in the simulation. [Four-pupil observations and counterexamples](decision-evidence.json) · [Onboard sensor and cargo stops](stops.json)

A different mechanism explains a later failure. If the negative chassis keeps following the proposed wheel path after losing custody evidence, the coupled receiver predicts 20.35 mm of travel before the pipe hits a hard stop at 38.981 s. Changing the camera threshold would leave that physical event intact.

| Architecture | Result on the same two losses | What would select it |
|---|---|---|
| Fixed and roof stereo | The roof continues seeing the pipe. The fixed height interval causes both brakes. A stationary image cannot distinguish a working grip from one that has just failed with the pipe resting in the same place. | Independent height and retention estimates under motion, background changes and a real simulated grip fault. |
| Robot-owned matte background in the fixed view | Two 105 mm side strips and one 550 × 235 mm sheet at chassis X355 mm fit clear of all 975 stowed body boxes. Ray casting each candidate in both real pupil frames leaves the two height intervals unchanged. | A different viewing or backing geometry would have to change the discriminating rays, retain a clear slide and field path and clear the same false-grant cases. |
| Non-imaging occupancy and travel limits | Three finite transverse beams fit clear of the stowed body boxes. The rear-limit beam detects the attained hard-stop pose in all nine ±1 mm X/Z registration samples. The three-beam pattern accepts a 24 mm sideways displacement and a failed grip at its old pose. | Exercise an independently sensed retainer and a separate occupancy/exit witness through the same two loads, an empty receiver, axial and lateral slip, a jammed latch and reconfirmed release. Require retention for the bounded transport loads before enabling motion. |

The beam comparison uses the finite hollow tube, including its open bore. Testing a solid cylinder would change the answer. The ray and panel checks model stowed packaging. Their brackets, wiring, full-stroke sweep and load capacity remain open. The existing 7.2 mm pressure-pad active areas lie forward of the seated pipe. A rotating grip idler witnesses actuator response, while a static image witnesses position. Neither measures available traction.

**The selected one-shot stationary retry is containment.** The two simulated stops meet the encoder, odometry and gyro limits over a 50 ms window. The planned retry would discard pre-stop images, require five fresh paired observations of the same pipe within 200 ms, and spend one retry allowance for that pipe across its entire trip. This study ends at the stop. Reacquisition, identity through a dropout, remaining-maneuver feasibility and the ability to drive on or retreat from either stopped pose remain unverified.

The 31.25 µs receiver simulation resumes the actual tube, 21 shelf/carriage coordinates, two plungers and both five-mode guide states. During the first three seconds the tube moves at most 0.736 mm or 1.098 mm relative to its departure center in the two signed runs. The chassis moves approximately 290 mm from its starting pose during that interval. Its speed at the end is 0.402 m/s. Camera measurements use the original pair of fixed eyes, the roof pair, the dark shelf sensitivity, uncertain wheel pixels and the station collision geometry at each attained field pose. Farther along the field, additional field objects, peer appearance and establishing the empty visual reference from a sensed pose matter. The collection of wheel paths awards zero deliveries.

## Reproduce

Supply the raw four departure registers and source-bound tube checkpoints from the previous study. Use separate output directories for each sign, timestep and policy. From this directory, run:

```powershell
python field_transit.py $departure $run plus 31.25 --policy south-offset
python cargo.py $departure $stationContact $run $cargo plus 31.25 --until 3
python custody_clock.py $departure $run $cargo $field $clock plus 31.25
```

The alternative policies are `crossing` and `settled`. Each optical exposure preserves four image identities on the original 17 ms cadence. The 5 ms control service receives images after 15 ms. A missing, inconsistent or more than 40 ms old capture triggers the brake. The occupancy and wheel-power checks apply on the attained clock. The historical six-wheel route supplies field coordinates and peer reservations.

For the onboard-only comparison, build the stereo cache with the field-backed reader first and run `custody_clock.py` with `--internal` and a separate output. Keep one directory per signed case beneath the common artifact directory containing `native-scene/`. The minus and plus 31.25 µs decisions occur 0.685 s and 2.525 s after departure. Replay each cut with `field_transit.py --cut <elapsed>` and with `--hold 1.1` in separate output directories. Propagate contact against the first of those two wheel paths. The compact [stop register](stops.json) checks that the longer path reproduces every sampled state through the measured stop. The [comparison script](premise.py) reprojects the two actual invalid exposures through each background and runs the finite-wall beam checks. The later hard-stop control uses `cargo.py --resume` on consecutive three-second segments of the wheel-only proposal.
