# Can two optical ports seat the station pipe?

<!-- cad-still -->
[![Team 1780 · captive swerve receiver](../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Team 1780 · captive swerve receiver.** The public Fusion capture candidate at rest, with 1,001 robot solids and the hollow practice CORAL. Project-drawn representations occupy the supplier interfaces. [Source STEP · 0dad9e862e](../../../cad/swerve-captive/public-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

On the issued 545-body receiver, a desktop controller brought the centered simulated pipe to **75.67 mm** against the **72–88 mm** seat window. It used two thresholded optical returns and a clock. A zero-speed catch allowed the pipe to hit both drive pulleys. A short outward belt command kept the pipe farther forward, then a slow outward scan located it again. The competition controller still requires physical calibration and an independent custody decision before retracting. [Controller](PortAcquisition.java) · [Results and conditions](results.json) · [Historical controller comparison](../coupled-belt-clock/README.md)

## What the eyes can tell us

The red receiving STEP has 545 named bodies. The shelf and mounts each have matching Ø12 mm vertical bores at local +100 mm (Entry) and +40 mm (Seat). The selected Pololu QTRXL-HD-01RC 4501 boards sit 22 mm below the shelf top at DIO0/1. Their RC discharge depends on reflectance. Establish the relation between discharge time, pipe finish, orientation and height on the actual assembly. [Reopened port geometry](ports.json) · [Electrical selection and manufacturer references](../design/electrical-inspection/README.md)

The model asks what a calibrated pair of digital thresholds *could* resolve. It assumes a vertical ray anywhere through each aperture, the centered transverse pipe radius 57.15 mm, maximum detectable underside height **3.7–4.2 mm** above the shelf at Entry and **17.2–17.8 mm** at Seat, and ±0.25 mm port registration. These are proposed acceptance bounds. An activation height of 4 mm above the shelf corresponds to 26 mm above the sensor plane. For upper bounds on false-positive coverage, a seated tube can return a signal up to `6 + sqrt(2 R h − h²)` sideways from the port center. Requiring both highs narrows the center to **72.246–87.695 mm** under the stated assumptions. Wide uncalibrated returns could cover both ports at +49 or +91 mm.

## Acquisition on a 20 ms clock

1. **Wake.** Require an enabled, docked, calibrated and valid empty frame. Every later input must have the current tick’s timestamp. Pre-spin both belts outward at 0.45 m/s while waiting for Entry. A seat-only first observation faults.
2. **Brake.** Hold 0.45 m/s for 160 ms after the first entry observation, then hold stationary belts. An incoming pipe can light both ports on its first sampled tick. This grants no completion signal. Require a later seat-only sample followed by a clear pair.
3. **Probe.** After at least 260 ms of braking, move the belts outward at 0.16 m/s. Require a fresh seat-only sample and then both ports high. Immediately request stationary belts.
4. **Confirm.** All synchronized samples must remain high on both ports for 240 ms. The resulting `IN_BAND` signal maintains active zero belt speed. A contradictory edge, missed clock, timeout, lost dock or missing calibration latches neutral output. The controller issues no drive or slide commands. [All digital inputs and emitted belt commands](ticks.csv)

The plant and Java controller exchange frames during the run. Pipe position, velocity, spin and source release time are private to the plant. Independent WPILib scheduler replay verified **11,113 ticks in 160 trials** with exact 20,000 µs FPGA steps. The two tested integration steps were 0.5 and 0.25 ms. [Desktop verification](hal.json)

| Changed input | Observed outcome |
|---|---|
| Axial depth −5, 0 or +5 mm, each with chute release on every integer phase from 0 through 19 ms | All 60 cases reached the seat. The earlier requirement for an isolated incoming Entry sample would reject valid simultaneous samples in this clock sweep. |
| High/low optical ranges and opposing port registration endpoints | All four additional cases seated. Across all 64 successful cases, centers were 74.79–76.67 mm at first confirmation, 1.149–1.209 s after chute exit. |
| Stationary early belts | At −5 mm depth the pipe reaches −33.32 mm. Its actual annular wall intersects each 22-tooth grip drive pulley by 3.91 mm³. The counterfeed’s worst modeled rearward center is −25.94 mm. [Continuous supported-travel sweep](sweep.json) |
| Grip 2 N/side with friction 0.08, alternate sliding-ramp friction 0.15 | Weak grip loses the pipe across the rear edge and eventually times out. The sliding-ramp input produces an unresolved inward edge sequence and a neutral fault. A clock-only fallback could resume with no established piece. |
| Missing optical calibration, stale frame, disabled, lost dock, stuck channels, interrupted dwell | Neutral output, with the observed reason in the trial. Even an empty receiver fed the complete nominal electronic sequence reaches the identical `IN_BAND` command. Two correlated false signals can defeat the electrical proof. |

The station-space +5 mm case moves the incoming pipe axially by +5 mm in robot coordinates on the original 10 mm standoff / 240 mm receiving stroke. It leaves the shelf and mechanism fixed. Lateral offset, nonzero arrival yaw, contact compliance, an obstructed port and tube-end effects need separate characterization. The flight and supported sweeps inspect the centered robot against the exact issued red assembly. The alternative station mounting and cheek-relief work has its own geometry and evidence. [Station localization](../station-localization/README.md)

Before using this as a possession signal, measure each optical channel’s turn-on and turn-off ranges and latency over the intended PVC surface, light and contamination range. Verify the true pipe center and velocity during a catch against an independent instrument. Check the 160 ms early drive, motor tracking, measured preload and rear clearance on the same assembly. The desktop phase sweep fixes the fresh-frame transport latency at zero; it establishes no sensor bandwidth or real trigger margins.

The centered, continuously swept outer envelope from −27 to +110 mm cleared all 545 robot bodies at the 0.05 mm³ reporting threshold. Before touchdown, the padded chord envelopes intersected only the receiving shelf, where the model places its known impact. These checks cover the issued rigid robot solids and leave real belt deformation, field-side contact and dimensional tolerances open. [Inbound sweep](flight.json) · [Supported scan](sweep.json)

## Reproduce

Use the project’s existing CAD/science environment, JDK 17 and populated WPILib 2026.2.1 Gradle cache. All experiments run on the desktop.

```powershell
python observable-acquisition/native_ports.py
python observable-acquisition/study.py --java-home <JDK17>
python observable-acquisition/sweep.py
python observable-acquisition/flight.py
python observable-acquisition/verify_hal.py --java-home <JDK17> --gradle-home <populated-cache>
python -m unittest discover -s tests -p test_observable_acquisition.py
```
