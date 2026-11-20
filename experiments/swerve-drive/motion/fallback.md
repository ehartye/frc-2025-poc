# Empty-start swerve fallback

Reserve the south starting slot for Red 2 and assign Red 1 the middle slot before the match. Use this branch when the team has decided to give up station supply and both scoring attempts. The stowed robot starts with its bumper over the starting tape. It drives toward its own alliance end in AUTO, stops, waits through disable and returns toward midfield in TELEOP. Its park target is X 9.600 m, Y 2.4272875 m, heading 180°. [Park geometry](../../../cad/full-swerve-r4/design.md#end-of-match-park-target) · [Watch the replay](https://ehartye.github.io/frc-2025-poc/#media)

| Attained case | AUTO stop | Tape clearance at AUTO end | Park stop on wall clock | Largest X error during assessment | Smallest peer gap |
|---|---:|---:|---:|---:|---:|
| 65 kg, positive calibration error | 3.100 s | 180.1 mm | 22.060 s | 6.32 mm | 746.9 mm |
| 72 kg, negative calibration error | 3.070 s | 176.6 mm | 22.030 s | 1.98 mm | 743.2 mm |

Both cases earn **five modeled points** under these reservations. Three come from LEAVE and two from PARK. FIRST evaluates bumper overlap at the starting line for LEAVE and in the alliance barge zone for PARK. The scoring review uses the competition manual, §6.5.2 and Table 6-2. Its edition differs from the kickoff worksheet. [FIRST competition manual](https://firstfrc.blob.core.windows.net/frc2025/Manual/HTML/2025GameManual.htm)

| Wall time | Controller and independent observation |
|---|---|
| 0–15 s, AUTO | Encoder distances, four absolute azimuths and a delayed gyro control a red-side target at X 10.60 m. The controller latches a stop below 6 mm estimated position error and 12 mm/s measured wheel speed. The scoring check uses actual robot pose. |
| 15–18 s, disabled | All eight motor outputs are disabled. The previously attained chassis state continues with the modeled motor braking and tire forces. The bumper clears the line throughout. |
| 18–153 s, TELEOP | The controller drives back along the reserved lane to the preselected park target and latches its stop. Drive authority comes from the enabled phase. The fallback uses no camera readings or scoring mechanism commands. |
| 153–156 s, assessment | The plants continue with outputs disabled. Both robots stay on the floor, with bumper cloth in the zone through the entire three-second window. |

The 156 s traces each contain 31,201 chassis states and 1,248,000 component power observations. The 1 ms wheel integration and 5 ms controller service run through the full hold. The 72 kg test combines the opposite wheel and gyro errors with the higher mass. These two combined cases do not isolate a mass effect or establish a match success rate.

The full-interval CAD screen encloses all attained poses and the motion between samples. The numbered R4 and its full steering envelope clear registered static field hardware by at least 251.5 mm and all 24 original stock locations by 1,421.7 mm. The separate six-degree cage cone leaves 53.8 mm. The five prescribed peers clear the attained footprint in each case. Leaving Red 1 in its usual south slot instead produces 72.6–76.6 mm of starting overlap and invalidates the plan. The team must secure the lane reservation. An opponent entering the lane, movement of the stock, a larger cage excursion, or switching to this routine from an arbitrary match pose requires another review.

## Reproduce

Use the project Python environment with NumPy, OpenCV and Open Cascade. Supply the official field STEP matching the digest in the existing park study. From this directory run the following commands with a project-owned output location.

~~~sh
python fallback_match.py 1 runs/plus.json
python fallback_match.py -1 runs/minus.json
python fallback_clearance.py path/to/FE-2025.STEP runs/review.json runs/plus.json runs/minus.json
~~~

Each solver run writes a JSON summary and its full numerical trajectory beside it. The native review checks those bytes, compares all controller ticks to the motion and assessment conditions and emits the CAD/field receipt. The issued review includes display samples from the same raw traces. The film uses that receipt and the orbit viewer's colored R4 mesh. Wheel detail uses the registered stowed CAD posture. The on-screen clock indicates where playback speeds up during a hold. [Issued review and source hashes](fallback-match.json) · [Film renderer](../../../scripts/render-fallback-film.py)
