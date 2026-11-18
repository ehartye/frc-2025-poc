# How much room does the route use?

A robot occupies an area during travel. We swept the neutral 30 × 30 in box continuously along the field-lab centerlines while holding heading at zero. A second trial extends its front face 8 in along global +X. The obstacle footprints are the two regular hexagons in the game model. A simultaneous 2D overlap is a candidate for a height and contact check. The tests do not model forces, validate bumpers or certify field legality.

| TELEOP route from 0.12.0 | Base-box finding | With front projection |
| --- | --- | --- |
| Station → reef for CORAL | Minimum reef gap 7.000 in. The start also needs a station-wall check. | First reef footprint overlap at 137.833 in of the 139.647-in route. |
| Reef → processor for ALGAE | Minimum reef gap 4.684 in. | Same minimum gap with the assumed +X orientation. |
| Reef → net for ALGAE | First reef footprint overlap at 10.144 in of the 171.863-in route. | Same first overlap. |

For the net trial, replace the modeled intermediate point with (228,101) in. The resulting 187.165-in route clears the reef footprints by 4.684 in and costs another 0.255 s at the assumed 60 in/s. Check bumpers, cargo, turns and all other structures before using the result for an actual cycle. This resolves a specific planar overlap without choosing a scoring strategy.

## Field boundary and overhead constraints

The V0 field has nominal 690.875 × 317 in extents. Its walls include four diagonal station faces. The [Jan 4 WPILib tag layout](https://github.com/wpilibsuite/allwpilib/blob/b65f159c3fdfdfce9ec126cf79dfd9a16d1e66e9/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape.json) supplies observed drawing coordinates and orientations for station tag planes. We approximate the physical wall with each tag plane and screen the entire square against all eight planar boundaries. At A=(45,42) in the closest corner lies 1.092 in beyond the tag-12 plane. Moving the center to (48,45) in gives 3.098 in of nominal margin. Confirm the physical panel offset, robot heading, bumpers and receiver reach at that pose.

The barge has a central support, horizontal truss, elevated nets and suspended cages. The model must distinguish the height of each obstruction. At a cage footprint, a 24-in-high ground-based box overlaps a deep cage and fits under a shallow cage in this nominal column screen. A solid 30-in-high box overlaps both. The 62-in truss reference and 76-in net low point do not describe center support clearance. Processor and station openings apply only in their local wall planes. [V0 manual, §§5.1–5.6, 5.8 and 6.3.5](https://manuals.plus/m/e98a748723b248f34e890accab1c49c809276fe44de713321834e30736d1e04a.pdf)

## Two robots requesting the same passage

Assume two adjacent deep cages and a straight crossing. The published center spacing and cage widths leave a 35.5-in cross-section. One 30-in-wide box fits with 5.5 in to spare. Two will not fit abreast. A 36-in occupied width misses by 0.5 in even before a margin. These tests use square cage faces facing the lane. The installed pose, swing and other structures can change the passage.

Reserve the 44-in nominal barge depth exclusively for each opposing traversal. At a constant 60 in/s, a 30-in-long box occupies the reservation for 1.233 s from front entry to rear exit. A robot requesting entry at AUTO 12.600 s clears at 13.833. A second requesting at 13.000 waits 0.833 s and would clear at 15.067. The second completion misses the 15-s active deadline. A scoring assessment interval after the buzzer cannot extend the travel budget. The same requests at TELEOP elapsed times 132.6 and 133.0 s would miss its 135-s deadline for the second robot. A six-robot model can apply these reservations to the phases and resources each robot actually uses. [V0 manual, §§6.4–6.5](https://manuals.plus/m/e98a748723b248f34e890accab1c49c809276fe44de713321834e30736d1e04a.pdf)

## Replay

### Does this alliance need a faster loader?

Run `python scripts/kickoff_partner_timing.py` to compare the *same* 2.5 s partner reservation at two times. This study uses the revised station approach (48,45) in and the original reef destination. The field lab returns a 135.493-in route with potential overlap from the same two modeled partner slots. For a team discussion, assume a single active receiving lane, four pieces allocated to our robot, another piece supplied independently to the partner, mean loaded/empty transit of 80 in/s, 1.0 s to receive and 0.5 s to place. The lane serves each pickup without interruption. The global inventory still supports both allocations. The direct route, lane exclusivity, exact timing, successful scoring and robot speed are trial assumptions.

| Partner's use of our receiving lane | Our first pickup begins | Our third delivery completes | L1 deliveries by 15 s |
| --- | --- | --- | --- |
| No reservation | 1.000 s | 13.968 s | 3 |
| From 0.5 until 3.0 s | 3.000 s | 15.968 s | 2 |
| From 2.1 until 4.6 s | 1.000 s | 13.968 s | 3 |

The uncontended plan can absorb 1.032 s of total waiting before its third completion misses the deadline. The early reservation consumes 2.000 s of that budget. Shifting it into our loaded transit preserves both the partner's reserved duration and our third delivery. Student decision: measure and coordinate station access before requiring a faster mechanism. If team coordination cannot hold total delay to the budget, compare a different alliance role or measure what service/drive improvement is realistic. Timing alone leaves the retention and post-contact test open.

From the repository root, run both scripts. The first uses a continuous separating-axis test and convex swept footprint. The second independently applies stage boundaries, cube height and an exclusive-resource clock.

```sh
python scripts/kickoff_route_clearance.py
python scripts/kickoff_field_occupancy.py
python -m unittest discover -s tests
```

[Field reference inputs](field-constraints.json) pin the source commit and separate source dimensions from scenario assumptions. The [nominal opportunity study](README.md) records the earlier throughput calculation. Its original station pose and net centerline should be revised before interpreting cycle times as achievable. Unknown load magnitude and actual traffic remain separate experiments.
