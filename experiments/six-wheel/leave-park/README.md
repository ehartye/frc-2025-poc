# A match with no station supply

<!-- cad-still -->
[![Preliminary design · in progress · layout-001](../../../docs/robot/preliminary-layout-001.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-001.** Red 2 B′x fixed pocket, roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 64 representative solids, with the field fixture, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 1cc8a76119](../../../docs/preliminary-design/releases/layout-001/receipt.json)
<!-- /cad-still -->

[Open the interactive leave and park replay](replay.html). Red 2 begins empty on its starting tape. Its receiver stays stowed at the issued height throughout. This role uses only the drivetrain. A team deciding before the match can use it when the station, pipe capture or release observer is unavailable. Switching into it from an arbitrary mid-match pose would require a new route.

## Trade tape clearance for cage clearance

Choose **Stop closer to the tape** in the replay. Keep Red 1 in the reassigned middle slot and Red 2 in the south lane. Red 2 drives straight for **0.543 m in 1.345 s** and holds at field X 372 in. Compared with X 370 in, the shorter drive gives the hanging hardware another **47.71 mm** of movement allowance. At the assumed 3° sway, the robot still clears its starting tape by **32.29 mm** after tracking error and keeps **28.74 mm** of clearance to the hanging hardware. Its front cover overlaps the parking zone through the buzzer.

| Red 2 stop | Hanging hardware movement available after tracking | Remaining clearance with 3° assumed sway | Tape clearance after tracking |
|---|---:|---:|---:|
| Reserved lane, X 370 in | 62.89 mm | −18.96 mm | 80.16 mm |
| Reserved lane, X 372 in | 110.60 mm | 28.74 mm | 32.29 mm |
| South partner keeps slot, X 370 in | 29.74 mm | −52.11 mm | 80.16 mm |

Move the **Sway allowance** slider to compare the entire drawn paths. The native field has six fixed eye nuts, six cages, four chains and ten carabiners. A 131.39 mm radius encloses each hanging assembly about its registered axis at any twist. For an assumed angle θ, the screen adds 1,561.84 mm × tan(θ) horizontally in every direction, using the highest fixed eye and the floor. For field use, the centerlines of the hanging cage, clips and chain must stay within that cone below the eye throughout the reservation. Survey the anchor positions and bound the entire assembly, including its bottom during tilt. A single cage-face angle cannot establish that condition. The full-height projection covers either team-selectable cage depth. At 4.1° the new route exhausts the allowance. The kickoff planning polygons still reserve the wider lane, and the robot's actual tracking, cage motion and partner commitments require field evidence.

An exhausted allowance leaves the outcome unresolved. A separate solid test shows a real intersection for the partner-retained route. Tilting its adjacent deep cage 6° toward the parked robot, about a trial pivot in the vertical overlap of the upper clip and eye, puts **665.02 mm³** of cage material inside the front bumper cover. The same two bodies have zero common volume at 0° and 4°. These are specified poses. The study does not predict field swing amplitude or contact forces. [Hanging inventory, route bounds and material trials](cage-motion.json)

The high, double-pinned carriage uses the same outer footprint. The check reopens the stowed robot and replacement kit, measures the resulting 583-body assembly, and repeats the bumper, stock and wheel checks on the X 372 in route. It clears all 24 installed pieces. Running the two 49.68–65.22 kg mass endpoints through the wheel model requires a tire friction coefficient of 0.272, up to 11.72 A phase current per drive and at least 11.83 V modeled bus voltage. The installed Agent FRC body-event review remains registered to the 545-body source revision. [Locked assembly](../design/fixed-height) · [Installed route check](near-tape-review.json)

Rebuild the separate X 372 in route with `python leave-park/native_park.py FIELD_STEP --near-tape`. Then run `python leave-park/cage_motion.py FIELD_STEP` and `python leave-park/view_data.py`. Use `python leave-park/body_route.py PLUGIN_ROOT artifacts/park-cages --near-tape` to replay its bumper event. The earlier four recorded requests retain their own source revision.

## Stop five inches sooner

In the replay, choose **Shorter straight AUTO** and toggle **Static cage bounds**. Red 2 starts at the same point in the reserved south lane and stops at field X 370 in. It travels 0.594 m in 1.407 s. Its bumper clears the starting tape by 117.93 mm and still overlaps the scoring zone at the end of the match.

| Stowed route | Clearance to static STEP hardware | Clearance to planning exclusion | Minimum AUTO lateral gap to a red partner |
|---|---:|---:|---:|
| Original stop, X 365 in and Y 95.5625 in | 26.61 mm | 25.85 mm | 748.6 mm with slot reassignment |
| Shorter stop, X 370 in and Y 95.5625 in | 113.41 mm | 25.85 mm | 748.6 mm with slot reassignment |
| X 370 in and Y 105 in, partner keeps south slot | 105.99 mm | 0 mm | 166.0 mm |

The program reopens all 545 bodies and encloses the straight, stowed motion in one prism. It traverses the official field assembly hierarchy and measures distances to the nearby hardware. A 350 mm search radius covers every part that could determine these minima. The limiting parts are cage members. The floor and two tape markings support the wheels. The STEP supplies static cage positions. The kickoff polygons reserve space for motion. [Native distances and geometry registration](native-park.json)

With independent wheel-ground travel errors bounded by ±1%, an initial 5 mm position error and a 0.4° heading error, the shorter path consumes 37.76 mm of clearance. **75.65 mm remains** against the static hardware for cage displacement and other uncertainty. The conservative planning exclusion still allows only 25.85 mm. The partner-retained route has positive separations in the static geometry and six-robot model but enters that exclusion. Measure installed positions, tracking and cage motion before narrowing the reservation.

Agent FRC 0.21.0 verifies the bumper event against the local hardware and one registered partner. Keeping Red 1 in its original south slot for the issued approach produces the same front-cover overlap at the finish. Both covers intersect the partner envelope at the start, so the checker rejects that route. Whole-field qualification retains its separate stock, partner and traffic checks. [Installed route decisions](body-route-review.json)

Rebuild the measurements with `python leave-park/native_park.py FIELD_STEP`, then run `python leave-park/view_data.py`. To replay the project mappings, run `python leave-park/body_route.py PLUGIN_ROOT artifacts/park-route`. Supply the official field export matching the recorded digest. [FIRST field drawings](https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf)

## Reserve the starting position before choosing the role

**The straight AUTO is the supported five-point candidate.** Red 1 takes the middle slot. Red 2 starts directly in the south cage gap at Y 95.5625 in, drives 0.721 m toward the BARGE and holds that pose through the end of TELEOP. One straight segment takes 1.550 s. The robot clears its own starting tape and stays entirely on its own side of the field centerline. Its actual bumper covers overlap the BARGE ZONE at the finish. If Red 1 keeps its usual south slot, the two starting envelopes overlap by 73.7 mm. Assign both starts before attempting this version. G303 requires bumper overlap with the starting line and the other start conditions. G403 and G405 govern AUTO interactions. [Scoring and drivetrain receipt](study.json)

The stowed, empty assembly mass interval is 49.461–64.941 kg. The route uses the project's conservative yaw inertia, 170.5 mm CG radius, minimum side load of 26.85%, full outer-axle loading bound and the documented gearing, scrub and rolling assumptions. Replaying the complete signed wheel demand at both mass extremes gives **0.265 required tire friction** for the straight role, **11.69 A** peak phase current per drive, and an **11.79 V** minimum modeled bus. The middle-slot turn route reaches **1.351 required friction** against the assumed 0.90. Its field geometry works with a clear lane. Its turn performance requires tighter weight-distribution evidence or a different path. The electrical comparison uses the drivetrain auxiliary-load model and its assumed battery state. Wheel slip, stowed mechanical retention, actual loads and carpet tracking still require measurement. [Assembly mass and balance](../design/competition-package/shop-duplex/mass.json) · [Drive model](../design/drivetrain/route_check.py)

The 25.85 mm cage margin also sets a drivetrain acceptance target. With a 5 mm initial position error radius, 0.4° initial heading error and each wheel's **actual ground travel** within ±0.25% of commanded travel throughout the drive, the complete footprint can deviate by up to 22.02 mm. That leaves 3.83 mm against the modeled field. Expanding each wheel bound to ±0.50% consumes 29.83 mm, so this test cannot guarantee clearance. An encoder count alone cannot establish ground travel under slip. The calculation integrates the differential heading error over distance and includes rotation about the farthest chassis corner. The eroded start tape and final park zone retain actual bumper material. A match plan also needs the alliance to reserve the occupied cage gap for the entire match; a partner needing the same space to climb changes the choice. [Wheel-error cases](study.json)

## Starting in the middle slot

| South-lane condition | Observed behavior | Conditional match contribution |
|---|---|---|
| Partner has cleared and reserves the lane | Red 2 reverses in AUTO, begins the parking route at TELEOP 118.000 s, comes to rest at 126.431 s and holds through the buzzer. | 3 LEAVE + 2 PARK |
| Partner holds its starting slot | Red 2 completes the same AUTO and stays out of the lane. | 3 LEAVE |
| Clearance arrives at TELEOP 129 s | The remaining six seconds are shorter than the 8.431 s route. Red 2 holds. | 3 LEAVE |
| Proceed past the occupied slot anyway | The programmed footprints first overlap at TELEOP 121.81 s. The replay freezes the rejected route there. | LEAVE was already assessed. PARK is withheld. |

The parking departure deadline for these exact wheel limits is TELEOP 126.569 s. That is a mathematical boundary with zero delay allowance. The selected 118 s departure leaves 8.569 s of slack. The modeled partner drives clear from the start of TELEOP. A real driver needs a confirmed reservation covering the entire approach and final footprint. Another match could have different partners, routes or disturbances.

## Occupancy and scoring for the middle-slot trial

The program reopens all 545 red bodies from the issued stowed STEP and checks the actual bumper covers against the starting tape and red BARGE ZONE. The complete hull is 850 × 850 × 1,059 mm. At AUTO start the front cover intersects the vertical tape prism by 23,574 mm³. At AUTO 1.636 s both covers have cleared, with 378 mm between hull and tape. The robot remains at rest through AUTO 15 s and the three-second disabled interval. Both covers intersect the park prism at the reserved endpoint. The wheels are stationary, the model stays floor-supported, and there is no cage attachment. The holding pose has zero cover volume inside the zone. [Exact intersections and times](study.json)

All 15 robot pairs are screened on the 135 s TELEOP clock with bounded motion between 20 ms samples. The minimum pair bound is 499 mm for the reserved trial. The focal robot is checked through its complete turns and straights against the registered field model and 24 installed game-piece occurrences. Its closest installed stock stays more than one metre away. The other five robots use prescribed 850 mm square envelopes, including checks on their paths through the field and stock. Partners hold during AUTO. The replay interpolates 100 ms visual samples. Assessment and clearance use the wheel models, independent stock receipt and solid geometry. [Field registration](../mission/field-registration.json) · [Installed field inventory](../full-match/installed-stock.json) · [Issued red CAD](../design/competition-package/shop-duplex/red/review.json)

## The parking bottleneck

The south approach passes 25.85 mm from the modeled cage exclusion. A 10 mm bound on total planar tracking error plus a 1° heading bound consumes 20.49 mm at the most distant chassis corner. About 5.36 mm remains. A 20 mm translation bound plus the same heading error consumes 30.49 mm. Shifting the final straight 30 mm toward the cage also produces zero clearance in the registered planar model. Positive bumper material remains inside the zone after erosion by either error allowance. The approach limits this design. The other robot envelopes, carpet grip, steering, field setup and cage motion need field measurements. No game piece, climb or ranking point is credited. [Tracking and displaced-route controls](study.json)

The comparison uses the competition edition of the FIRST manual, §6.5.2 for LEAVE and PARK and Table 6-2 for their values. Its SHA-256 is `dc6aa9ddbeba25c679c58bae746bde14dba2af79a61a65d8c5f575ca6cdfd523`. Field drawing FE-2025 sheet 3 supplies the tape and zone geometry. The kickoff exercise retains its own release-day cutoff. Official scoring remains with field personnel. [FIRST manual](https://firstfrc.blob.core.windows.net/frc2025/Manual/HTML/2025GameManual.htm) · [Field drawings](https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf) · [Kickoff rule register](../kickoff/rule-register-v0.json)

From the project root run `python leave-park/rehearse.py` in the project OpenCascade environment, then `python leave-park/view_data.py`. Open `leave-park/replay.html` locally.
