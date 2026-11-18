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

From the repository root, run both scripts. The first uses a continuous separating-axis test and convex swept footprint. The second independently applies stage boundaries, cube height and an exclusive-resource clock.

```sh
python scripts/kickoff_route_clearance.py
python scripts/kickoff_field_occupancy.py
python -m unittest discover -s tests
```

[Field reference inputs](field-constraints.json) pin the source commit and separate source dimensions from scenario assumptions. The [nominal opportunity study](README.md) records the earlier throughput calculation. Its original station pose and net centerline should be revised before interpreting cycle times as achievable. Unknown load magnitude and actual traffic remain separate experiments.
