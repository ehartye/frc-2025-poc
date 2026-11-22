# Can the receiver tell whether both belts actually moved?

<!-- cad-still -->
[![Preliminary design · in progress · layout-002](../../../docs/robot/preliminary-layout-002.png)](https://ehartye.github.io/frc-2025-poc/preliminary-design/)

**Preliminary design · in progress · layout-002.** Red 2 B′x fixed pocket, held roller nip and exit lip on four-module swerve. Parked pose at the end of the withdraw-park case, with the exit lip deployed and the pocket empty. Whole-robot block-level layout from the layout-checked review scene: 67 representative solids, with the three field fixtures, the delivered pipe and two service hand gauges hidden. It is not detailed CAD. [Open the review](https://ehartye.github.io/frc-2025-poc/preliminary-design/) · [Layout-checked receipt · 14649626c7](../../../docs/preliminary-design/releases/layout-002/receipt.json)
<!-- /cad-still -->

At 580 ms the receiver asks its two belts to help seat the pipe. On the original 850 mm guide, a good left feed and a completely stalled drive gave the same Entry and Seat readings until 1,100 ms. The two **passive front idlers** provide an earlier observation. Each turns when its own timing belt advances, even though neither sensor sees the pipe. On the desktop, both locked-belt trials went neutral at **640 ms**, 700 ms before the issued controller's timeout. [Original port evidence](../split-guide-acquisition/README.md) · [Replayed results](qualification.json)

| Station condition | Idlers at 640 ms | Last station decision |
|---|---|---|
| Left arrival, both integration steps | Both advanced | IN_BAND at 1,340 ms |
| Right arrival, both steps | Both advanced | IN_BAND at 1,360 ms |
| Left, both belts held locked, both steps | No advance | Latched FAULT at 640 ms |
| Left, grip traction lost, both steps | Both advanced | Seat went clear. FAULT at 700 / 880 ms |
| Empty receiver supplied with both passing port and angle recordings | Both appear to advance | False IN_BAND at 1,340 ms. Independent custody remains required |

The held-lock cases command zero at the fault and both actual belt speeds are already zero. The good histories reproduce every 20 ms port-controller state and belt command at 31.25 and 15.625 µs mechanical integration through the existing 5,460 ms handoff. The original camera exposure states and mechanical restart remain applicable to these four issued trials by command identity. The detector supplies no chassis, carrying or scoring permission. The four separate 813 mm folded-guide histories reproduce every issued state and command through the same handoff, using their own published port transitions. The locked and traction-loss physical faults remain scoped to the original guide. [Folded guide inputs](../guide-impact/results.json) [Handoff](../split-guide-acquisition/restart.json) · [Independent optics](../selected-optics/README.md)

## Where the measurement comes from

Each grip has a 20-tooth, 3 mm pitch passive timing pulley, a round 5 mm shaft and an axial Ø5 × 1 mm magnet. One revolution corresponds to 60 mm of belt travel. The shaft magnets center at robot X95.5 mm, Y±161.562 mm with the shelf stowed. Their tops are at Z576 mm. Each 12 × 12 × 4 mm PCB reserve starts at Z577 mm. Reopening the 144-body native drive STEP reproduced the ten selected parts' bounds in the 991-body scene, with a 1 mm magnet-to-board-envelope gap on each side. The round pulley bore touches the shaft. Rotation of one part relative to the other is geometrically possible. Sensor die height, attachment and transmitted torque need first-article evidence. [Native fit](fit.json) · [Drive source](../design/receiver-drives/README.md)

The selected circuit reserves a separate ADS1015 at address 0x49 for both idlers and two slide synchronization pickups. The existing 0x48 pad converter retains its own 2 ms triplet. With full-turn AS5600 analog output on 3.3 V and a ±4.096 V ADC range, one turn spans about 1,650 positive converter codes at 2 mV/count. The Java wrapper takes signed motion from each sensor with opposite polarities and unwraps across zero. The fit charges zero incremental sensor mass against these existing reservations. Final board, retainers, cable, mounting and scale weight must be reconciled with the bought articles. [Selected circuits](../design/electrical-inspection/circuits.json) · [ams sensor](https://look.ams-osram.com/m/3ac60b27c91075b5/original/AS5600-FS000133-2-00.pdf) · [TI converter](https://www.ti.com/product/ADS1015)

## How the veto works

1. Before enabling the bounded probe, both channels must provide calibrated, recent data. Separate per-channel acquisition timestamps are required. A repeated stamp, missing channel, unexpected half-turn, impossible step or stale transfer latches neutral.
2. The original 20 ms controller commands 0.16 m/s. At 60 ms, the two latest readings on **each** idler must show more than 1.50 mm forward over a preceding 40–45.25 ms window. The comparison repeats every 20 ms while probing. Early dual port returns without the prior two-idler observation fail closed.
3. Once the port controller enters DWELL it commands zero. Belts can then be stationary. The original 240 ms dual-return dwell, recalibration, docking and enable interlocks still apply. A short stall after the last accepted rolling comparison can fall across the transition to DWELL. The independent custody check still owns what happens after acquisition.

| Timing and uncertainty | Conditional qualification |
|---|---|
| Acquisition | One sample per idler every 5 ms, maximum gap 5.25 ms, transfer ≤4 ms, age ≤10 ms |
| Installed angle uncertainty | ≤3° per reading including mounting, full-turn scale, supply drift, wrapping and analog system error, plus the two stated quantization terms |
| Mechanical response | ≥0.14 m/s rolling, ≥16 m/s² initial acceleration and ≤0.40 mm differential lost motion; issued nominal model uses 0.16 m/s and 26.25 m/s² |
| Timing error | Effective angle lag ≤3 ms and exposure clock error ≤1 ms |
| Separation | Stationary worst pair reports ≤1.0511 mm. Healthy lower bound is 2.8364 mm even with opposed angle errors, worst permitted delay, lash and start ramp. Threshold 1.50 mm lies between them. |

A stationary newest comparison already vetoes motion, even if the older comparison still saw the belt moving. The 960 healthy and 960 locked fixtures exercised sample phase, shaft angle including wrap, opposed errors, acquisition delay and sensor lag. Each decided at 60 ms. Of 480 trials that stopped later during a continuing probe, the first neutral command followed physical idler stop by 39–61 ms. A continuous upper bound of **79.5 ms** includes the 40 ms window, up to 5.25 ms on either end, 4 ms transfer, 3 ms sensor lag, 2 ms differential clock error and one 20 ms service. These bounds apply after the idler itself stops. Contact force, stored energy and motor heating need their own limits. A rolling idler with a slipping pipe can pass this test. [Controller](IdlerWatch.java) · [Uncertainty calculation and sampling fixtures](qualify.py)

## What students would need to measure

- Witness-mark pulley, shaft, magnet and belt. Index one turn each way against 20 teeth on both sides with the cartridge under its applicable preload and load. Check for slip, lost motion and loose magnet before and after the test. Record full-turn angle against an independent index through the seam, both signs and all tested approach directions. Measure the installed die position and supply range. A magnet fixed to a slipping shaft would invalidate the detector.
- Timestamp both real conversions and the arrival of the complete packet at the controller. At a configured 3,300 samples/s and the specified ±10% data-rate tolerance, four sequential conversions take up to 1.347 ms before configuration, bus wait, host and transmission. Test all four inputs concurrently with the independent pressure converter. Measure actual sensor-filter lag at start and stop, acquisition clock error, reference distortion and rail clipping. The analog signal alone does not carry per-sample magnet validity. [Converter timing and input ranges](https://www.ti.com/lit/ds/sbas473f/sbas473f.pdf) · [Sensor evaluation guide](https://look.ams-osram.com/m/4f8342513a447495/original/AS5600_UG000254_2-00.pdf)
- Compare shaft travel with an independent timing reference during the declared probe and separately evaluate the safe force/current/time limit with a representative fixture. Record true stationary, healthy, one-side-only and no-traction trials. Preserve the independent optical handoff for the pipe. The checked-in report leaves installed calibration empty.

## Reproduce on a desktop

Use the project's Python environment and JDK 17. The qualification compiles the exact released acquisition source with the watchdog, runs packet-level failures and writes the raw converter-fixture trial summary and station port inputs in the specified working directory. The native check uses the registered scene from the fixed-camera-route workflow. [Scene reproduction](../fixed-camera-route/README.md)

```text
python grip-motion/qualify.py JDK17 WORK/grip-motion
python grip-motion/fit.py WORK/fixed-camera-route/scene.json
python -m unittest discover -s tests -p test_grip_motion.py
```

