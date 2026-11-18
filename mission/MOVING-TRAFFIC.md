# Moving traffic and finite station stock

Open [the offline field rehearsal](moving-field.html). Select a departure, play it, or jump to the next conflict, feed or supported pipe. Red 2 is the station-to-L1 candidate. Red 3 and Blue 3 follow independent differential-drive programs across its route. The other three robots hold their prescribed starting poses.

| Candidate | Predicted intersection or minimum separation | Supported by 135 s | Empty recovery |
| --- | --- | --- | --- |
| Start immediately | Red 3 and Red 2 overlap at 3.87 s | Departure withheld | — |
| Wait 10 s | Blue 3 and Red 2 overlap at 15.73 s | Departure withheld | — |
| Wait 16 s, stock at 0 and 35 s | All pairs clear by at least 99.9 mm | A at 62.60 s, B at 113.84 s | 132.75 s |
| Wait 16 s, stock at 0 and 111 s | Same traffic clears | A at 62.60 s | 159.88 s, after cutoff |
| Wait 16 s, one pipe available | Same traffic clears | A at 62.60 s | No second trip |

The first invalid separation intervals center on 3.85 and 15.71 s. The later sampled overlaps independently confirm the two no-go decisions. The reserved two-delivery model leaves 2.25 s to the buzzer after empty recovery. The later stock case still feeds B before the buzzer, but cannot deliver it. These trials compare schedules students can negotiate. They establish neither a minimum start delay nor a preferred match role.

## What couples the trials

- Red 2 consumes exact registered approach, capture/seat, transport, hold, withdrawal, empty return and station recovery intervals. The unloaded lift uses the existing 2 s station-height recovery. Clearance and retention come from the [registered two-pipe model](REGISTERED-FIELD.md) and its 96-part assembly. The lateral sector offsets remain −180 and +220 mm.
- The two additional paths use the same differential wheel/acceleration limits. Red 3 holds the first corridor intersection for 5 s. Blue 3 starts at 8 s and traverses the north cage gap. Field obstacles and perimeter planes are checked along these paths. Every 20 ms the maximum separation along the four footprint axes is reduced by the combined corner-motion bound over half the interval, including acceleration. All 15 pairs must clear for the 135 s trial. A stock or occupancy failure withholds the affected delivery.
- Each assumed robot occupies an 850 mm square. During Red 2 placement/recovery the check encloses the extended assembly in a 1070 by 850 mm rectangle. Opposing and partner intakes, localization error, slip, unmodeled field objects, contact deflection, driver decisions and delays remain unmeasured.
- Stock moves through one serial station lane with one identified pipe per slot. Its putative operator spends 1.1 s inserting a pipe after both the robot and stock are ready. A hollow cylinder with 50.8/57.15 mm inner/outer radii then rolls at rest down an effective 42.11 mm length on a 35° incline. Ideal no-slip acceleration is 2.968 m/s², giving 0.16845 s to the 0.50 m/s modeled lip state. This short effective length joins the existing measured-in-simulation feed pose to a possible handoff. It is a testable hand/loading assumption. Changing the measured lip state requires a new custody run.

## Which scoring condition changed

The [native red reef receipt](branch-exclusion.json) identifies 12 complete GE-25301 scoring pipe weldments. Each pipe at its sustained floor-and-lip pose is enclosed by a **filled** 59.15 mm radius, 305.625 mm long cylinder. Filling the pipe also covers its bore. The nearest scoring weldment is 160.24 mm from this volume for A and 161.39 mm for B. A probe placed on actual weldment material gives zero separation. All 12 top bounds register at 1828.66 mm. These measurements use the pipe orientation and displacement in the registered experiment. The 2 mm expansion is an explicit model margin.

The official field drawing identifies the weldment’s pipe sections and caps. The archive manual requires a branch inside the volume of the CORAL for L2–L4 and separately excludes an already-scored other level from L1. The positive full-volume separation supplies this missing later-edition witness. At the two supported poses, each edition's L1 predicate returns **yes**. The focal robot’s departure is covered by the registered mechanism/body test. The other five robot footprints stay at least 1.342 m from either settled location for the full clock. Both poses are instantaneous simulation assessments. Official points, complete inventory and referee assessment remain unresolved. [FIRST manual §§5.3 and 6.5.1](https://firstfrc.blob.core.windows.net/frc2025/Manual/HTML/2025GameManual.htm) · [FIRST official field STEP](https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025Field-STEP.zip) · [Field drawings, GE-25301, p. 163](https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf)

## Reproduce

Run from the repository root. The first two commands use standard Python and checked-in receipts. Supply the original archived field assembly to independently regenerate the native branch distances with the CAD dependencies already used by this project.

```sh
python mission/moving_traffic.py --matrix mission/moving-traffic.json
python mission/render_moving_traffic.py
python mission/check_branch_exclusion.py path/to/FE-2025.STEP --output mission/branch-exclusion.json
python -m unittest discover -s tests -p test_moving_traffic.py
```

The [scenario matrix](moving-traffic.json) stores LF-normalized SHA-256 input identities, earliest occupied intervals, actual overlap samples, feed state, per-specimen observations and edition-specific evaluations. The branch witness requires the archived FE-2025 assembly hash recorded in its receipt.
