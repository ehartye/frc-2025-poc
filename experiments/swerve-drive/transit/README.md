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

The 31.25 µs receiver simulation resumes the actual tube, 21 shelf/carriage coordinates, two plungers and both five-mode guide states. During the first three seconds the tube moves at most 0.736 mm or 1.098 mm relative to its departure center in the two signed runs. The chassis moves approximately 290 mm from its starting pose during that interval. Its speed at the end is 0.402 m/s. Camera measurements use the original pair of fixed eyes, the roof pair, the dark shelf sensitivity, uncertain wheel pixels and the station collision geometry at each attained field pose. Farther along the field, additional field objects, peer appearance and establishing the empty visual reference from a sensed pose matter. The collection of wheel paths awards zero deliveries.

## Reproduce

Supply the raw four departure registers and source-bound tube checkpoints from the previous study. Use separate output directories for each sign, timestep and policy. From this directory, run:

```powershell
python field_transit.py $departure $run plus 31.25 --policy south-offset
python cargo.py $departure $stationContact $run $cargo plus 31.25 --until 3
python custody_clock.py $departure $run $cargo $field $clock plus 31.25
```

The alternative policies are `crossing` and `settled`. Each optical exposure preserves four image identities on the original 17 ms cadence. The 5 ms control service receives images after 15 ms. A missing, inconsistent or more than 40 ms old capture triggers the brake. The occupancy and wheel-power checks apply on the attained clock. The historical six-wheel route supplies field coordinates and peer reservations.
