# Straight-guide station and reef rehearsal

<!-- cad-still -->
[![Shop-duplex assembly in its stowed pose](../../../docs/robot/cad/assembly.png)](../../../docs/robot/cad/index.html)

**Project CAD · Shop-duplex assembly in its stowed pose.** Historical six-wheel CAD reference. The four-module swerve candidate is under development. [Source STEP · 193c7976d1](../design/competition-package/shop-duplex/red/stowed.step) · [Rotate the model](../../../docs/robot/cad/index.html)
<!-- /cad-still -->

[Open the student field replay](replay.html). Choose a signed wheel configuration and move the match clock to see the study robot, installed stock and five other robots.

The straight-arm robot uses the upper two holes on the station panel to dock. Its existing fixed camera, a surveyed pair of reef fasteners and the modeled chassis gyro guide its approach to reef sector A. Both signed wheel errors reach the station before the first feed at 21.10 seconds. [Opening, stock and traffic](issued-opening.json) · [Reef observations](anchored-route.json)

## Match result

Both coarse signed simulations carry the same coral-A from their own simulated human feed to reef sector A. The left case delivers 1,249 independent roof captures to the tracking pipeline and stops at 48.257 s. The right case delivers 1,248 and stops at 48.232 s. Maximum modeled shelf deflection stays below 0.032 mm on both trips. The final pipe centers are 74.960 and 74.936 mm fore of the receiver datum. The two finite wheel paths, camera streams, contact histories and final generalized shelf states have separate source hashes. These trials use 31.25 µs feeder and 62.5 µs loaded-carry steps.

| Trial | Next permitted action | Demonstrated points |
| --- | --- | --- |
| Fresh station and reef observations, functioning idlers | Hold stowed at sector A pending extension support | 3 LEAVE |
| Station camera packets delayed beyond their freshness limit | Brake at the station approach and hold | 3 LEAVE |
| Both idlers seize after the observed seat | Brake at the first idler disagreement and hold | 3 LEAVE |
| Reserved south role chosen before the match | Follow its separately checked leave-and-park route | 3 LEAVE + 2 PARK |

The arrival on the previous calibration and guide images remains available for comparison. Its coarse left carry reached A, while its fine left carry lost the roof height interval at wall 27.197 s. The new edge bound tightens the same finite pixel-error enclosure and the current trials continue from their freshly attained cargo. [Current left capture and carry](cargo-leftSmall-31.25.json) · [Current right capture and carry](cargo-rightSmall-31.25.json) · [Fine previous control](left-15.625.json)

## Field reservations

The ten pairs formed by the other five programmed robots retain at least 1.340 m separation through the full 153 s field clock. Their minimum fixed-stock clearance is 1.170 m. Their tightest barrier is the north cage at 14.44 mm. The study robot's approach to A comes within 7.90 mm of installed stock on the smaller-left-wheel route and 8.64 mm on the smaller-right-wheel route. Along those trips the nearest other robot remains at least 130.78 and 161.25 mm away. [Other five robots](peer-traffic.json) · [Left wheel](field-leftSmall.json) · [Right wheel](field-rightSmall.json)

## What students can change

Keeping only the upper station holes removes two partial lower-hole centroids from the estimator. The original panel detector stopped the left robot 192 mm away and the right robot missed the human feed deadline. The upper-hole detector reaches stationary confirmation at 20.35 s and 20.59 s, respectively. At 15 s both robots have cleared the start line with their complete bumper footprint. They remain still through the disabled interval. On the original 20:1 drive model the observations depend on the registered starting survey and modeled traction and axle loading. [Original detector](opening.json) · [Upper-hole observations and image timestamps](revised-opening.json)

As a separate check, we rendered the assembled native station and complete match-drilled guide scene at the first exposure and stationary endpoint of both runs. The initial images contain eight to eleven eligible dark components; the terminal pupils contain four or five. The broad search selects the upper outer holes in all four stereo pairs without a pose input. A separate local-window check seeded at the registered pose finds the same correspondence. The simplified full-clock panel renderer includes four outer holes and the actual robot occlusion. The four native checks establish the correspondences at those poses. [Native image association](native-panel.json)

For the next leg the driver uses its last stationary camera estimate to replan to fixed field waypoints. With the left wheel small, zeroing the plan at the ideal station instead stops the robot 127 mm short. Under the revised policy, both stationary grants are within the simulated sector target. Removing fresh exposures inhibits both. A 14 mm shared survey error fools the right estimate. The projected supplier field and robot solids, lighting contrast, camera calibration and gyro model are simulation inputs.

## From a station lock to a pipe

A confirmed dock schedules a new simulated human feed of coral-A. The pipe evolves under gravity, the two idler surfaces, flexible shelf, two guide stages, folded guide backing and two stow dampers. The Java program observes two interrupted beams. At the station, a second camera views the pipe from above. Its geometric estimate uses every available secant of the concave projected end to bound the hidden crown. Its retained tests check false inputs, independent yaw and visible edge support. [Predecessor incoming pipe controls](stations.json)

The opposite wheel calibrations also expose an ambiguity. Holding normal distance fixed, +3 mm lateral and +0.270° yaw can share the two upper-hole image centroids at ±0.074 pixel on the left and ±0.087 pixel on the right. The −3 mm and −0.271° alternatives need ±0.229 and ±0.118 pixel, respectively. The signed left negative control faults on unresolved incoming edges at Java 1020 ms. It withholds custody, even though stationary repetitions of the paired centroids would agree. A numerical tolerance in this table is the maximum difference to a shared midpoint in either axis in either eye. [Pixel challenge](station-uncertainty.json) · [Port fault](offset--3mm-leftSmall-31.25.json)

With the actuators free, both signed dock poses seat coral-A through Java 5460 at both feeder timesteps. Five independent camera pairs are delivered in time on each trial. The smaller-right-wheel run first reaches IN_BAND at 1360 ms on the coarse step and 1340 ms on the fine step. The final pipe center moves from 77.08 to 74.09 mm in the receiver. The next simulation starts separately from each signed, derived mechanical state. With seized idlers, the independent encoder disagrees with the requested surface travel. The left wheel case stops at wall 27.222 s after traveling 0.538 mm during braking. The right stops at 27.137 s. The reached stops remain over 2.33 m from peer footprints through match end. [Left seized-idler trial](match-leftSmall-31.25-idler-jam.json) · [Right seized-idler trial](match-rightSmall-31.25-idler-jam.json)

The estimated empty field mass is bounded at 65.833 kg and the loaded drive adds 0.816 kg for one pipe. On the fully applied prebuild mass estimate and 11.8 V / 27 mΩ pack the signed openings pass through 92.5% load on the two outer axles. At 95% they inhibit. With the 32 mΩ pack, both pass at 77.5% and inhibit at 80%. These are discrete samples of the opening; the nominal full loaded path uses 75%. [Independent wheel-load replay](../wheel-loads/README.md)

The reservation, surveyed station coordinates and delivered camera time are declared inputs. Future physical commissioning would measure traction, axle loading, camera survey, grip and shelf damping. No physical measurements are required for the simulation.

## Why the robot holds the rack

The stowed receiver has a different support condition from the scoring extension. A separate dynamic trial finds the fixed plunger reaches a rib at 21.375 mm of travel, about 0.5994 s into the prescribed 370 mm extension. Removing the plungers still gives a negative required normal reaction under the earlier signed pipe states. This project keeps the rack stowed at a reached A approach while a successor bearing fixture is evaluated. Coral-B stays in stock. The five programmed peers continue through TELEOP while the study chassis holds its attained pose. The last reported pipe observation bounds cargo evidence. A credited L1 requires positive material support and the applicable scoring observation. [Extension contact and adverse cases](../extension-support/dynamics.md)

## Reproduce

The commands below use the project artifact directory, sourced field STEP and a JDK 17. Set the project's existing CAD/vision Python environment first. Earlier project receipts describe how to generate the shared engineering inputs. Run each signed wheel case and both feeder steps, 31.25 and 15.625 microseconds.

```powershell
python straight-guide-match/opening.py $Artifacts
python straight-guide-match/opening.py $Artifacts --revised
python straight-guide-match/opening.py $Artifacts --revised --fault late
python straight-guide-match/issue.py $Artifacts
python straight-guide-match/station_uncertainty.py $Artifacts
python straight-guide-match/route.py $Artifacts --anchored
python straight-guide-match/feed.py $Artifacts $Jdk leftSmall 31.25
python straight-guide-match/attained_views.py $Artifacts $FieldStep leftSmall 31.25
python straight-guide-match/stations.py $Artifacts
python straight-guide-match/carry.py $Artifacts --side leftSmall --arrival-us 31.25 --step-us 62.5 --attained --mode guarded-all-edges --output straight-guide-match/cargo-leftSmall-31.25.json
python straight-guide-match/shared_clock.py $Artifacts leftSmall 31.25 --output straight-guide-match/field-leftSmall.json
python straight-guide-match/match.py $Artifacts leftSmall 31.25
python straight-guide-match/peer_traffic.py
```

For the finer carry, use the independently generated 15.625 µs feed and a 31.25 µs travel step. The denominator is two signed wheel configurations. Timestep comparisons and injected failures are controls on those starts. The raw trajectory, images and per-frame timestamps live in the project artifact directory under the case IDs in each report. To reproduce a signed ambiguity control, pass `--offset -3` or `--offset 3` to the feeder. Use the signed receipt as its own case. The student viewer embeds its inputs and opens directly from a checkout.

```powershell
$records = Get-ChildItem straight-guide-match/match-*.json | Sort-Object @{Expression={if ($_.Name -match 'nominal'){0}elseif ($_.Name -match 'idler-jam'){1}else{2}}},Name
python straight-guide-match/visual.py $records.FullName
```
