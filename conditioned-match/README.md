# Does the first real pickup lead to two scores?

The revised opening drive reaches the station before the 21.10 s feed under both signed wheel calibrations. The 2025 shop robot still lacks a qualified two-placement mission. The shelf and cameras can confirm a seated pipe under one material model, but the coupled shelf and the subsequent reef observation can each stop the scoring chain. The earlier nine-point replay used a different chassis and observer scene. [Student match replay](replay.html) · [Original two-trip control](../relieved-match/README.md) · [Preselected south alternative](../station-approach/README.md)

## Start, drive and stop

The issued wheel path first exceeds the 38 A phase limit at the first 4.470 s turn transition on the modeled fresh battery. At an empty field mass of 65.833 kg it demands 49.66 A (left wheel small) or 51.45 A (right wheel small), with required tire friction coefficients 0.994 and 1.026 against the 0.90 assumption. The screened supply first fails at 1.060 s. The completed-path forecast is a counterfactual beyond that first inhibit. [Issued-path power](power.json)

Running the original gyro reference at 94% pace and the terminal visual wheel ramps at 290 mm/s² reaches stationary stereo lock at 20.35 and 20.51 s. The visual guard uses the same acceleration for the braking-distance allowance, transition stop and actual wheels. A 230 mm/s² terminal ramp at the same initial pace loses the right-side view. The successful trials retain zero motion throughout the three-second disabled interval, whole-bumper clearance of the start tape, and at least 12.30 mm to barriers, 36.70 mm to installed stock and 84.17 mm to any moving peer through the scheduled release of the first pipe. The 5 ms source interpolates wheel samples at their physical endpoints. [Controller trials](tuning.json) · [Achieved poses, disabled motion and clearance](opening.json)

The model joins grip hold, radio, two cameras, host, roboRIO and drives on the common supply. At a modeled 11.8 V open circuit and 27 mΩ common resistance both corrected paths retain at least 9.934 V at the right drive. The corresponding lower bound at 32 mΩ is 9.595 V. The 34.65 A maximum phase current uses a 20:1 drive, 75% maximum outside-axle loading, 44% minimum lighter-side loading, at most 0.38 scrub friction and at least 0.90 available longitudinal grip. Final component masses, individual wheel loads, carpet traction and battery resistance require physical checks. The mass allowance uses the R103 limit, a 6.17 kg battery and one 7.5 kg bumper set. [Installed Agent FRC power review](power-review.json) · [Drive mechanics](../selected-drive/README.md)

## Pipe identity and the first impact

Both runs start with coral-A and coral-B in human station stock. A leaves at wall 21.10 s. Its physical lip is at 21.671673 s, equal to Java 100 ms. The existing Java intake receives the new, signed station coordinates. Under the 400 kN/m, 4 s/m effective contact law, both approaches enter the dual-port seat state at Java 1,400 ms and hold through Java 5,460 ms at both 62.5 and 31.25 µs numerical steps. The 1 s/m damping controls fault under both wheel calibrations. Those cases stay at the station. The modeled robot in either stationary hold clears the five scheduled peers by at least 2.33 m through the end of TELEOP. The loose failed pipe has a separate, unresolved field trajectory. [Attained feed inputs and Java outcomes](feeds.json) · [Constitutive assumptions and earlier controls](../shelf-contact/README.md)

The registered shop camera renders the actual attained pipe pose against the native station, existing sensor apparatus and both opaque guide faces. Fixed stereo and the moving roof each supply distinct pixel fits. At the modeled 15 ms delivery, the last pre-departure capture at Java 5,440 ms can reach the control loop at 5,455 ms. The raw state transferred to the carry model includes position, axis, linear velocity, angular velocity, identity, original clock and zero commanded belt speed. Translation and rotation of the reached chassis introduce accelerations in the receiver frame. [Fixed stereo](seat-images.json) · [Roof camera](roof.json) · [Achieved loaded motion](corrected-cargo.json)

Three native pressure-film tails are exposed to the arriving pipe. An interior point of each reopened film intersects the nominal finite pipe wall by 1.29–1.37 mm on the earlier opposing #105 trajectories if the 0.8 mm-high laminate stays undeformed. The first interval begins near Java 317 ms. This is an elastic-interface question. A rigid tail cannot simply be added after a calculation that assigns all penetration to a single pipe/shelf spring. The separate plate trials move the surface modes and the sensor ray together. The original plate loses both signed pipes. The insert without dampers reaches the cartridge stop on the left and loses the pipe on the right. Coupon force, rebound, shell ovalization, pressure-film strain, guide anchorage and as-built clearances remain unmeasured. [Actual native material witnesses](pad.json) · [Coupled plate and damper cases](elastic.json)

## Reef stop and the reserved second pipe

A completed plan from the older 597-body assembly released A at 48.010 s and B at 98.101 s. To test the strongest conditional continuation, the current shop scene includes the fixed head, the two opaque guides and the carried-pipe enclosure at the reef. Repeating the original wheel targets from the earlier attained station puts the two reef handoffs about 150 mm off laterally. Reusing sampled gyro heading reduces the starting error enough for the older camera to acquire the marker. On the shop scene the two-eye detector never establishes ID10 at either of the two newly powered A arrivals. The controller brakes with the rack stowed. A one-eye tag fit recovers 30/27 initial registrations and 48/42 paired fastener observations, but both trials lose the registered field view and settle 98/108 mm short of the A sector. Five moving robots, installed floor and reef stock and the stowed hull were swept over both projected continuation paths. [Camera decisions](navigation.json) · [Stock and peer bounds](occupancy.json) · [Alternative viewpoints](views.json)

No reached case supplies the A placement, independent forward release witness, rack withdrawal and confirmed stow needed to arm the return. Coral-B stays in its distinct human stock slot at 71.501 s. The 156 s assessment includes no confirmed L1 placement and no park on the attempted station route. The independently reserved south start retains a conditional LEAVE/PARK opportunity worth five points, compared with three conditional LEAVE points on the powered station hold. Choosing that south role occurs before the match start. [Event ledger and the historical controls](match.json)

## Reproduction

Use the project's registered field STEP and CAD/vision assets, desktop JDK 17, installed Agent FRC and the project's Python dependencies. Set the numerical library to a single BLAS thread per process for the elastic trials. The raw frame, motion and contact arrays stay in the supplied artifact directory.

```sh
python conditioned-match/tuning.py "$ARTIFACTS"
python conditioned-match/opening.py "$ARTIFACTS"
python conditioned-match/navigation.py "$ARTIFACTS"
python conditioned-match/views.py "$ARTIFACTS"
python conditioned-match/occupancy.py "$ARTIFACTS"
python conditioned-match/supply.py "$ARTIFACTS"
python conditioned-match/plugin.py "$ARTIFACTS" "$AGENT_FRC_PLUGIN_ROOT"
python conditioned-match/acquire.py "$ARTIFACTS" "$JDK17"
python conditioned-match/pad.py "$ARTIFACTS"
python conditioned-match/seat_images.py "$ARTIFACTS" "$FIELD_STEP"
python conditioned-match/roof.py "$ARTIFACTS"
python conditioned-match/transport.py "$ARTIFACTS"
python conditioned-match/transport.py "$ARTIFACTS" --fine
python conditioned-match/transport.py "$ARTIFACTS" --corrected
python conditioned-match/elastic.py shelf-structure "$JDK17" "$ARTIFACTS"
python conditioned-match/ledger.py
python conditioned-match/viewer.py "$ARTIFACTS"
python -m unittest discover -s tests -p test_conditioned_match.py
```
