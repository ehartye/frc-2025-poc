# Two pipes with one receiver

<!-- cad-still -->
[![Four-module swerve frame and mounting hardware](../../../docs/robot/cad/assembly.png)](../../../docs/robot/cad/index.html)

**Project CAD · Four-module swerve frame and mounting hardware.** Original robot structure for the swerve candidate. The project-local Fusion assembly adds four manufacturer module occurrences. [Source STEP · f8b75f2df4](../../swerve-drive/packaging/swerve-frame.step) · [Rotate the model](../../../docs/robot/cad/index.html)
<!-- /cad-still -->

The 597-body replay demonstrates the controller sequence. At its two extended scoring poses, each inner slide rail intersects the official reef wall by about 1,387 mm³. The revised shop channels and position sensor clear the wall through the modeled stroke. Station docking, powered motion and the physical build still determine whether the robot can score. [Field contact and revised receiver](../selected-match/README.md)

The [second-placement packet recovery](../packet-recovery/README.md) uses the roof camera to preserve one delayed fixed exposure. Its own run recovers the nominal park. The three original cases below keep their recorded controller policies.

The same 597-body red robot leaves the starting line, takes two distinct human-fed pipes, scores each in L1 and parks at **110.878 s** in the nominal desktop match. Both rule editions count both pipes at the **156 s** wall-clock assessment. The robot earns nine modeled role points. Stopping after A earns seven and finishes parking at 60.478 s. A late fixed-camera packet during B leaves the rack extended. Three AUTO points remain credited in that run. A had reached L1 before the fault, and the final interaction of both pipes is unresolved. [Open the interactive match](replay.html) · [Read the assessment](match.json)

| Wall time | First pipe | Second pipe |
|---|---:|---:|
| Human feed | 21.100 s | 71.501 s |
| Port and independent fixed stereo confirm custody | 23.080 s | 73.460 s |
| Loaded drive armed | 27.032 s | 77.434 s |
| Belt release requested | 48.010 s | 98.101 s |
| Whole-front image permits rack withdrawal | 48.295 s | 98.375 s |
| Linear and rotary feedback establish motion | 48.330 s | 98.410 s |
| Rack reaches stowed endpoint | 50.765 s | 100.845 s |

Both captures use the same low receiver, machined cheeks, shims, fixed and shelf-mounted cameras, and bounded reverse probe. A begins at the registered AUTO dock. Its achieved reef location drives the biased return to the station. B arrives with a −0.5722 mm normal error, −0.0008 mm lateral error and +0.00375° yaw. Those errors enter B's actual feed and next wheel path. Stopped clearance to the official station is 10.000 mm on A and 10.545 mm on B. A separate +3 mm human-feed displacement reaches a mechanical stop before the first accepted seat on either trip. [Station and field CAD](native-register.json) · [First feed](first-feed.json) · [Second feed](return-feed.json) · [Reef navigation](navigation.json)

## What changed the scoring result

The force sensors sit **0.8 mm above the shelf**. At the apparent pressure reading, a flat-shelf model put 43.35–43.87 mm³ of each real sensor inside the pipe wall. That model generated successful controller events. Reopening the actual pipe and sensor material exposed the contradiction. [Native material witnesses](native-register.json) · [Flat-shelf control](flat-control/events.csv)

The corrected dynamics follow the same finite hollow cylinder from its acquired position, velocity, axis and angular momentum. The accelerating and rotating receiver applies belt traction, spring forces and finite shelf contacts. At release the three circular pressure heads and their raised tails support the pipe before it drops back toward the shelf edge. Resolving corner impulses adds 0.0064–0.0166 N·s across the four integrations. The downstream free-flight datum sits 30 mm past the final flat support. The enclosing capsule clears the shelf and sensor fronts continuously from this datum through first field contact. Fore-aft robot separation after field contact stays above 32.60 mm through assessment. The two finite pipe envelopes clear each other by 93.27 mm during B’s final extension and both placements after the 4.5 mm allowance. [Pad contact model](pad_contact.py) · [Tube dynamics](carry.py) · [Separation](free-flight-clearance.json) · [Trough support at the measured rack clocks](landing-observed.csv)

The pressure prior requires any one active disk to remain above the assumed **1 N** for two milliseconds. Both 0.25 ms and 0.50 ms integrations produce a prior on each pipe. Requiring 8 N for that dwell removes the prior in all four trials. Per-pad load sharing varies across the two step sizes. Real cover compliance, threshold and scan timing require bench characterization. [Threshold sweep and clock](controller-register.json)

The first pipe reaches field support before its first qualifying whole-front image. Ending the image stream at contact makes the desktop controller time out and hold A. Continuing the same article through registered trough contact supplies the fixed capture at local 26,707 ms. B qualifies at local 26,401 ms. The renderer includes the installed reef and the first pipe at the second sector. Both robot-camera registrations and the exact downstream rack traces have independent hashes. The trial uses 17 ms capture periods, 15 ms delivery, a 5 ms rack service loop, and the project’s calibrated capture phases for each acquisition epoch. The 36 full-robot roof checks cover stowed, midstroke and extended poses at three cartridge displacements and both ends of each measured carry envelope. Additional holes, wet material, ambient lighting, clock phase and real USB jitter require new evidence. [Original cutoff failure](pre-contact-control/events.csv) · [Camera registrations](camera-registration.json) · [Complete-robot image checks](roof-challenge.json) · [Native desktop HAL events](events.csv)

## Field and build limits

All six scheduled robots clear one another by at least 104.90 mm. Stowed-field barrier clearance is 4.11 mm and the smallest stowed stock bound is 1.034 mm. At the two scoring docks the 597-body sweep clears the occupied reef ball by at least 16.48 and 24.89 mm under the registered ±10 mm and ±1° perturbations. The nominal final bumper overlaps the parking sector by 335.68 mm. The modeled cage can swing 109.43 mm beyond the position allowance before reaching the parked chassis. These figures use the project’s six prescribed robot paths. They do not predict other teams’ behavior. [Route and scoring assessment](match.json) · [Installed material](native-register.json)

The competition role remains provisional. The model assumes a centered first station arrival and bounded feed distribution, nominal lighting and calibrated opaque material, the registered mass/traction and power bounds, and sustained floor/lip contact under the stated friction model. The force model uses three transverse sections per pad, rigid contact with negligible restitution, and 0.15 pad-cover friction. The earlier belt-indentation adverse case remains relevant outside these two centered feeds. Both earlier starting-error and alternate shop configurations retain separate qualification. [Earlier five return paths](../return-station/README.md) · [Cartridge design](../station-relief/README.md) · [Receiver power and traction gates](../README.md)

## Reproduce the desktop study

Use a populated Java 17 / WPILib desktop cache, Python with NumPy, OpenCV contrib, VTK and OpenCascade, the official field and lamp STEP files, and the registered wheel/optical cache from the [sector study](../sector-alignment/README.md) and [station study](../return-station/README.md). The nominal 1 ms rack curve is the historical HAL trajectory with SHA-256 beginning 62c69e83. The final HAL regenerates that same curve independently for A and B. Supply these paths locally and keep third-party CAD with its source.

```sh
python relieved-match/first_feed.py "$OUT" "$JDK17"
python relieved-match/navigation.py "$OUT" "$SECTOR_CACHE" "$STATION_CACHE"
python relieved-match/first_feed.py "$OUT" "$JDK17" --piece coral-B
python relieved-match/station.py native "$OUT" "$LAMP_STEP"
python relieved-match/station.py images "$OUT" "$FIELD_STEP" --prior "$STATION_CACHE"
python relieved-match/cameras.py "$OUT" "$LAMP_STEP"
for piece in coral-A coral-B; do
  for dt in 0.00025 0.0005; do
    for law in coupled pads pads-edge; do
      python relieved-match/carry.py "$piece" "$OUT" --dt "$dt" --mode "$law"
    done
  done
done
python relieved-match/edge_clearance.py
python relieved-match/hardware.py "$OUT" "$LAMP_STEP" "$FIELD_STEP"
python relieved-match/sight.py "$OUT"
python relieved-match/roof_check.py "$OUT" "$LAMP_STEP"
python relieved-match/landing.py "$OUT" "$JDK17" "$RACK_CSV"
python relieved-match/landing.py "$OUT" "$JDK17" "$RACK_CSV" --field-trace
python relieved-match/reef_scene.py "$OUT" "$FIELD_STEP"
for piece in coral-A coral-B; do
  for head in fixed moving; do
    python relieved-match/exposures.py "$OUT" "$piece" "$head"
  done
done
python relieved-match/control.py "$OUT" "$JDK17" "$GRADLE_CACHE"
python relieved-match/landing.py "$OUT" "$JDK17" "$OUT/path-nominal-coral-A.csv" --issued relieved-match/events.csv
python relieved-match/field_trial.py "$OUT"
python relieved-match/viewer.py "$OUT"
```

The controller’s published output is limited to this desktop study. Physical calibration, handoff timing, pad characterization, sensor calibration, power, wheel-load and inspection measurements remain open.
