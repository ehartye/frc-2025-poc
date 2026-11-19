# Correcting the loaded reef approach

The existing camera, gyro and four swerve modules corrected both signed approaches while retaining the identified station-fed CORAL. At the measured stops, 57.120 s on both match clocks, the simulated center was within 1 mm of the intended reef position. The sensor bounds span about 8 mm fore–aft and 30 mm laterally. Both yokes remained closed, both occupancy beams stayed blocked, and the end probes continued to agree with the measured pipe length. These two development runs deliver the original receiver and cargo to sector A. They earn zero L1 deliveries. [Measurements and states](results.json)

## How the robot corrects its pose

The previous study used the existing fixed camera to decode a surveyed tag in one eye upstream. At the stopped robot, its two pupils see one identifiable reef bolt. The gyro transports heading between the two observations. Four wheel encoders propagate the 15 ms camera delivery delay. A new image arrives every 50 ms during correction. The control caps translation at 20 mm/s and 80 mm/s² and verifies the final wheel and gyro stop. Adding an indistinguishable blob, losing range to the field marker, or losing a packet inhibits further correction. In the four mid-motion beam or packet faults the braking travel was 2.02–2.09 mm. [Camera input audit](camera_input.py) · [Drive and stop controller](approach.py)

| Development input | First stop, s | Measured starting reef X, mm | Final true error X/Y, mm | Final observed reef X, mm |
| --- | ---: | --- | --- | --- |
| Plus | 55.050 | −70.42 to −62.86 | −0.50 / +0.46 | −58.48 to −50.26 |
| Minus | 54.815 | −40.47 to −31.71 | +0.66 / −0.74 | −57.39 to −49.07 |

The target is (−54, −180) mm and 0° yaw in the surveyed local frame. The minus-case initial interval corrects the preceding publication. Two recent minus images exceeded its assumed 0.25 px corner allowance. One earlier frame also exceeded it. An observable 0.18 px reprojection residual rejects those three images. A separate comparison of each retained detected corner against its rendered physical location found maxima of 0.204 px across all 29 accepted plus frames and 0.232 px across 28 accepted minus frames. The discarded frames reached 0.580 px. This test covers these rendered images and calibration models. The old conservative hold remains valid at the original two stops.

At the correction speeds the stowed robot clears defined field barriers by at least 97.80 mm, installed stock by 45.35 mm and scheduled peers by 1330.44 mm. The two measured cargo trajectories continue all 414 and 461 wheel services from their respective previous state hashes. The 31.25 μs substeps include flexible shelf, finite CORAL, guide, closed pin and probe reactions. The original pipe’s velocity, axis and angular momentum are saved alongside the shelf's modes for a subsequent action. [Cargo continuation](cargo.py)

## Which receiver can score

Projected over the complete observed docking uncertainty, the previously simulated cam motion leaves 4.53 mm or 3.21 mm at the nearest stock separating plane. The screen includes both inner slide rails, all 26 retainer pieces, the possible pin/probe positions, 6,601 rack poses and an additional 1 mm interpolated-motion allowance. This screen addresses the reef-wall side of the approach. Tube release, full observation, competing pieces and scoring support require the scoring robot's own trajectory. [Placement screen](clearance.py) · [Independent cam stroke and landing](../score-stroke/study.md)

The original shelf in these two corrective runs hits the stationary cap near 21.375 mm of extension. The machined continuous-land receiver acquired both calibration inputs, but changed the plate stiffness and produced different station trajectories and terminal modal states. The results register pins both versions. The scoring design continues from the cam’s actual station capture, with the outgoing pipe, wheel clock and mechanism kept together. The competition target remains nine simulated points in all eight frozen qualification starts, including the assessment at 156 s.

## Replay

Supply the local field and contact artifacts from the [previous custody study](../retention/observer-study.md). The per-eye image generator needs OpenCV's AprilTag detector. The cargo solver uses NumPy with single-threaded BLAS.

```powershell
$R = 'experiments/swerve-drive/reef-correction'
python "$R/camera_input.py" $A
foreach ($sign in 'plus','minus') {
    python "$R/approach.py" $A $sign
    python "$R/cargo.py" $A $sign
    python "$R/clearance.py" $A $sign
    foreach ($fault in 'missing-eye','lost-packets') {
        python "$R/approach.py" $A $sign --mode $fault
    }
}
python "$R/collect.py" $A "$R/results.json"
```
