# Does the first real pickup lead to two scores?

<!-- cad-still -->
[![Shop-duplex assembly in its stowed pose](../../../docs/robot/cad/assembly.png)](../../../docs/robot/cad/index.html)

**Project CAD · Shop-duplex assembly in its stowed pose.** Historical six-wheel CAD reference. The four-module swerve candidate is under development. [Source STEP · 193c7976d1](../design/competition-package/shop-duplex/red/stowed.step) · [Rotate the model](../../../docs/robot/cad/index.html)
<!-- /cad-still -->

The revised opening drive reaches the station before the 21.10 s feed under both signed wheel calibrations. The 2025 shop robot still lacks a qualified two-placement mission. The shelf and cameras can confirm a seated pipe under one material model, but the coupled shelf and the subsequent reef observation can each stop the scoring chain. The earlier nine-point replay used a different chassis and observer scene. [Student match replay](replay.html) · [Original two-trip control](../relieved-match/README.md) · [Preselected south alternative](../station-approach/README.md)

## Start, drive and stop

The issued wheel path first exceeds the 38 A phase limit at the first 4.470 s turn transition on the modeled fresh battery. At an empty field mass of 65.833 kg it demands 49.66 A (left wheel small) or 51.45 A (right wheel small), with required tire friction coefficients 0.994 and 1.026 against the 0.90 assumption. The screened supply first fails at 1.060 s. The completed-path forecast is a counterfactual beyond that first inhibit. [Issued-path power](power.json)

Running the original gyro reference at 94% pace and the terminal visual wheel ramps at 290 mm/s² reaches stationary stereo lock at 20.35 and 20.51 s. The visual guard uses the same acceleration for the braking-distance allowance, transition stop and actual wheels. A 230 mm/s² terminal ramp at the same initial pace loses the right-side view. The successful trials retain zero motion throughout the three-second disabled interval, whole-bumper clearance of the start tape, and at least 12.30 mm to barriers, 36.70 mm to installed stock and 84.17 mm to any moving peer through the scheduled release of the first pipe. The 5 ms source interpolates wheel samples at their physical endpoints. [Controller trials](tuning.json) · [Achieved poses, disabled motion and clearance](opening.json)

The model joins grip hold, radio, two cameras, host, roboRIO and drives on the common supply. At a modeled 11.8 V open circuit and 27 mΩ common resistance both corrected paths retain at least 9.934 V at the right drive. The corresponding lower bound at 32 mΩ is 9.595 V. The 34.65 A maximum phase current uses a 20:1 drive, 75% maximum outside-axle loading, 44% minimum lighter-side loading, at most 0.38 scrub friction and at least 0.90 available longitudinal grip. At 11.8 V and 27 mΩ, concentrating the load on the four outer wheels challenges this assumption. The companion inverse-drive replay first inhibits the signed #108 commands at 3.295 s at 100% outer load, with peak demands of 39.48/39.34 A and minimum terminals 9.369/9.367 V. A differently geared approach changes the dock poses and requires separate pipe trials. Final component masses, individual wheel loads, carpet traction and battery resistance require physical checks. The mass allowance uses the R103 limit, a 6.17 kg battery and one 7.5 kg bumper set. [Installed Agent FRC power review](power-review.json) · [Drive mechanics](../selected-drive/README.md)

## Pipe identity and the first impact

Both runs start with coral-A and coral-B in human station stock. A leaves at wall 21.10 s. Its physical lip is at 21.671673 s, equal to Java 100 ms. The existing Java intake receives the new, signed station coordinates. Under the 400 kN/m, 4 s/m effective contact law, both approaches enter the dual-port seat state at Java 1,400 ms and hold through Java 5,460 ms at both 62.5 and 31.25 µs numerical steps. The 1 s/m damping controls fault under both wheel calibrations. Those cases stay at the station. The modeled robot in either stationary hold clears the five scheduled peers by at least 2.33 m through the end of TELEOP. The loose failed pipe has a separate, unresolved field trajectory. [Attained feed inputs and Java outcomes](feeds.json) · [Constitutive assumptions and earlier controls](../shelf-contact/README.md)

The registered shop camera renders the actual attained pipe pose against the native station, existing sensor apparatus and both opaque guide faces. Fixed stereo and the moving roof each supply distinct pixel fits. At the modeled 15 ms delivery, the last pre-departure capture at Java 5,440 ms can reach the control loop at 5,455 ms. The next capture at Java 5,457 ms is drawn from the last stationary pipe trace and arrives after departure. It preserves the existing 17 ms phase across the handoff. The raw state transferred to the carry model includes position, axis, linear velocity, angular velocity, identity, original clock and zero commanded belt speed. Translation and rotation of the reached chassis introduce accelerations in the receiver frame. [Fixed stereo](seat-images.json) · [Roof camera](roof.json) · [Achieved loaded motion](corrected-cargo.json)

Three native pressure-film tails are exposed to the arriving pipe. An interior point of each reopened film intersects the nominal finite pipe wall by roughly 1.3 mm on the earlier opposing #105 trajectories if the 0.8 mm-high laminate stays undeformed. The first interval begins near Java 317 ms. This is an elastic-interface question. A rigid tail cannot simply be added after a calculation that assigns all penetration to a single pipe/shelf spring. The separate plate trials move the surface modes and the sensor ray together. The original plate loses both signed pipes. The insert without dampers reaches the cartridge stop on the left and loses the pipe on the right. Coupon force, rebound, shell ovalization, pressure-film strain, guide anchorage and as-built clearances remain unmeasured. [Actual native material witnesses](pad.json) · [Coupled plate and damper cases](elastic.json)

The alternative machined insert relocates the two stow plungers to X118/Y−66,+42 mm, leaves 0.10 mm running clearance and recesses the three pressure laminates 0.8 mm. At 62.5 and 31.25 µs, the two powered arrivals enter the Java seat window at 1,360 ms and stay through 5,460 ms. Each damper absorbs 0.408–0.534 J in the finer runs. The finite wall and moving tail still overlap by as much as 1.108 mm under the effective contact law. On the preceding damper geometry both 0.20 mm running-gap controls fault. Physical damping, tail deformation, attachment strength and production tolerances remain open. [Four machined arrivals and earlier controls](elastic.json) · [Registered receiver and native geometry](https://github.com/ehartye/frc-2025-poc/tree/784134ea442501957b079f95fbff63f6e14ed641/shelf-structure)

The 998-body native stowed raster and registered field supply five of five position witnesses for each signed arrival on the nominal finish. With the entire changed shelf, both cartridges, the 629 fixed-lead bounds and ±2 mm vertical uncertainty included, five final fixed stereo frames per arrival enclose the simulated pipe on a dark backing. Matching the backing to the pipe intensity yields zero position witnesses on both arrivals. Blank and one-eye controls also withhold custody. The machined run stays at the station in this match ledger. A real finish, moving-roof transfer and elastic loaded travel need their own evidence. [Machined fixed-eye controls](machined-images.json)

## Reef stop and the reserved second pipe

A completed plan from the older 597-body assembly released A at 48.010 s and B at 98.101 s. To test the strongest conditional continuation, the current shop scene includes the fixed head, the two opaque guides and the carried-pipe enclosure at the reef. Repeating the original wheel targets from the earlier attained station puts the two reef handoffs about 150 mm off laterally. Reusing sampled gyro heading reduces the starting error enough for the older camera to acquire the marker. On the shop scene the two-eye detector never establishes ID10 at either of the two newly powered A arrivals. The controller brakes with the rack stowed. A one-eye tag fit recovers 30/27 initial registrations and 48/42 paired fastener observations, but both trials lose the registered field view and settle 98/108 mm short of the A sector. Five moving robots, installed floor and reef stock and the stowed hull were swept over both projected continuation paths. [Camera decisions](navigation.json) · [Stock and peer bounds](occupancy.json) · [Alternative viewpoints](views.json)

The issued loaded reference first exceeds the screened-pack limits at 28.420/28.435 s, long before either projected reef endpoint. With 85% outbound reference pace and the same delayed gyro service, both paths instead fit the 75% outside-axle power and current bounds. The two-eye field gate still brakes 186/151 mm short of A at 48.162 s. The slow stowed route retains at least 12.28 mm to barriers, 159 mm to installed stock and 151 mm to the five peers; a chassis stopped at either final modeled pose has over 1.33 m to those peers through TELEOP. These clearance calculations remain explicitly conditional wherever independent pipe custody or its continuation fails. [Loaded supply](carry-power.json) · [Slower wheel route](slow-navigation.json) · [Slower supply](slow-power.json) · [Carried state and camera](slow-cargo.json)

The two signed carry cases diverge under the slower powered route. Left-small loses coral-A from the flat shelf at 44.673/43.348 s with 125/62.5 µs integration. The conservative pressure-tail separation bound also falls below zero in both projections. Right-small retains the simulated pipe until the 48.162 s wheel stop at both resolutions, with at least 12.25 mm to the tail envelope and 10.71 mm to the guide. At 15 ms camera delivery and 5 ms service, right-small loses the ongoing roof-axis witness by wall 45.407 s and left-small by 38.557 s. Physical and wheel paths after those instants are forecasts. The first failed image alone does not establish a braking trajectory. [Slower cargo continuation](slow-cargo.json) · [Independent moving roof](slow-roof.json)

The gyro route on the earlier registration also retains its timestep controls. On the earlier wheel path the left-small case reaches the cartridge stop at 42.726/42.651 s at 62.5/31.25 µs while the right-small case reaches the end of the modeled chassis path. At the newly powered station pose on the original outbound pace the left-small case reaches the stop at 42.590/42.591 s under 125/62.5 µs. The electrical inhibit precedes those projections. [Earlier refinement](fine-cargo.json) · [Original-pace continuation](corrected-cargo.json)

No reached case supplies the A placement, independent forward release witness, rack withdrawal and confirmed stow needed to arm the return. Coral-B stays in its distinct human stock slot at 71.501 s. The 156 s assessment includes no confirmed L1 placement and no park on the attempted station route. The independently reserved south start retains a conditional LEAVE/PARK opportunity worth five points, compared with three conditional LEAVE points on the powered station hold. Choosing that south role occurs before the match start. [Event ledger and the historical controls](match.json)

## Reproduction

Use the project's registered field STEP and CAD/vision assets, desktop JDK 17, installed Agent FRC and the project's Python dependencies. Set the numerical library to a single BLAS thread per process for the elastic trials. The raw frame, motion and contact arrays stay in the supplied artifact directory.

```sh
python conditioned-match/tuning.py "$ARTIFACTS"
python conditioned-match/opening.py "$ARTIFACTS"
python conditioned-match/navigation.py "$ARTIFACTS"
python conditioned-match/views.py "$ARTIFACTS"
python conditioned-match/slower.py "$ARTIFACTS"
python conditioned-match/occupancy.py "$ARTIFACTS"
python conditioned-match/supply.py "$ARTIFACTS"
python conditioned-match/supply.py "$ARTIFACTS" --corrected
python conditioned-match/plugin.py "$ARTIFACTS" "$AGENT_FRC_PLUGIN_ROOT"
python conditioned-match/acquire.py "$ARTIFACTS" "$JDK17"
python conditioned-match/pad.py "$ARTIFACTS"
python conditioned-match/seat_images.py "$ARTIFACTS" "$FIELD_STEP"
python conditioned-match/roof.py "$ARTIFACTS"
python conditioned-match/transport.py "$ARTIFACTS"
python conditioned-match/transport.py "$ARTIFACTS" --fine
python conditioned-match/transport.py "$ARTIFACTS" --corrected
python conditioned-match/transport.py "$ARTIFACTS" --slow
python conditioned-match/roof.py "$ARTIFACTS" --slow
python conditioned-match/elastic.py shelf-structure "$JDK17" "$ARTIFACTS"
python conditioned-match/machined_images.py "$ARTIFACTS" "$FIELD_STEP" shelf-structure
python conditioned-match/ledger.py
python conditioned-match/viewer.py "$ARTIFACTS"
python -m unittest discover -s tests -p test_conditioned_match.py
```
