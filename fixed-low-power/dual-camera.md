# Power for the two-camera low robot

The 593-body fixed-height robot can carry the proposed electrical load in the desktop match with an 11.8 V pack and 27 mΩ pack-plus-trunk resistance. The lowest motor-controller terminal reaches **9.577 V** at 90.38 s with a 62.75 A peak battery demand. The moving camera has only **17 mV of modeled inlet margin**. Pack resistance, the hot moving harness and the actual regulator waveforms determine whether this design works on the robot. [Replay results](dual-camera.json) · [Robot configuration](../design/fixed-no-air/README.md)

## Panel and harness proposal

Mount three MEAN WELL DDR-30G-5 converters vertically on a retained TS35 rail on the fixed base. Route dedicated positive and return conductors from each PDH channel and keep each output independent. Leave the lift isolated, compressor absent and PH on its original branch. Preserve the existing RIO and radio supplies. Each camera plugs into its own Luxonis Y-adapter. Route its data port to the Pi and its power port to its assigned fused 5 V supply. Provide a properly terminated USB-C power source and short compatible plug leads. Verify USB-C attachment, orientation, load capability and idle/reverse current in each power sequence. The adapter guide identifies the two ports but gives no backfeed limit. [MEAN WELL specification, pp. 2–4](https://www.meanwell.com/Upload/PDF/DDR-30/DDR-30-SPEC.PDF) · [Luxonis connection guide](https://docs.luxonis.com/hardware/products/Y-Adapter)

| PDH / upstream | Load and output fuse | One-way input / output routing limit | Complete hot output loop target | Predicted inlet at maximum load |
|---|---|---|---:|---:|
| 6 / 10 A REV ATO | Pi 4B and sensor allowance, 5 A MINI | 0.80 m 18 AWG / 0.20 m 16 AWG | 31.59 mΩ | 5.019 V at 3.20 A |
| 8 / 10 A REV ATO | Fixed OAK-D Lite, 3 A MINI | 0.95 m 18 AWG / 1.30 m 14 AWG | 81.93 mΩ | 5.022 V at 1.20 A |
| 9 / 10 A REV ATO | Moving OAK-D Lite, 3 A MINI | 1.10 m 18 AWG / 1.50 m 14 AWG | 86.07 mΩ | 5.017 V at 1.20 A |

The modeled loop includes both conductors at 1.25 times their room-temperature resistance. The remaining series budgets are 25 mΩ for the host and 55 mΩ per camera, covering fuse, holder, splices, connector contacts, USB tails and adapter. Place output protection adjacent to the converter and qualify all smaller downstream conductors. The Littelfuse 0297005 and 0297003 typical cold resistances are already 17.75 and 33.75 mΩ. Temperature, contact wear and flex can consume the difference. Confirm the smallest protection compatible with measured startup and actual wire sizes, plus fault clearing with the converter in current limit. Check device-supplied pigtails and the historical Section 8 V11 rules R621–R625 at inspection. Keep signal ground references consistent with each USB data link. [Fuse data and opening curves](https://www.littelfuse.com/assetdocs/littelfuse_datasheet_297_mini32v.pdf?assetguid=42c9dd21-a88e-4328-8e67-2f832444faf1) · [Electrical inspection sources](../design/electrical-inspection/SOURCES.md)

The DDR-30G-5 adjusts from 4.5 to 5.5 V and accepts 9–36 V. Its 30 W rating derates at low input. Using the lower 85% load line gives a 25.5 W ceiling across the 9–12 V band at room temperature. The largest proposed output is 16.67 W. The published ±2% output tolerance and 60 mV peak-to-peak ripple leave insufficient room to guarantee this design from catalog limits alone. Set each unit near 5.16 V and require the instantaneous screw-terminal voltage to stay within 5.13–5.20 V over the actual load range, input range and enclosure temperature. The calculations allow another 10 mV instrument error. All three no-load ceilings are 5.210 V. An extra 50 mΩ in the moving leg takes that head to **4.957 V**, below its 5.00 V minimum. Output current limiting and internal overvoltage shutdown at 5.75–7 V do not establish camera protection at 5.25 V. Qualify or separately protect the endpoint for the intended fault cases. [Voltage and thermal curves](https://www.meanwell.com/Upload/PDF/DDR-30/DDR-30-SPEC.PDF)

The proposed panel fits beside the Pi in a 135 × 85 × 156 mm space allowance for the retained robot and ventilation. The 1.050 m fixed-camera centerline supports the revised 1.30 m copper allowance. [Panel layout and cable route](../design/fixed-no-air/converter-panel.md) The proposed gross mass budget is 0.36 kg for converters (the vendor lists 0.12 kg under packing), 0.14 kg for retention and protection and 0.25 kg for wiring and adapters. Carrying the full 0.75 kg without credit for existing supplies leaves **1.061 kg** under the conservative fixed-low two-camera inspection estimate. The 593-part assembly predates this retrofit. Finish the rail retention, fuse layout and moving cable sweep, weigh the actual parts, and isolate the moving share within its existing 0.440 kg allowance. Flex-rated conductors, a strain-relieved 370 mm service loop, cable bend radius and connector locking must survive full stroke. A change to moving mass or drag calls for new actuator and visibility trials. [Installed inventory](../design/fixed-no-air/inspection.json) · [Moving camera](../moving-observer/README.md)

## What the match establishes

The 156-second replay uses all 7,800 original 20 ms drive, grip, controller, radio and PH load bins. It replaces the shared observer allowance with three separately wired branches at 5.21 V times each maximum current. The nominal efficiency assumption is 85%. The 67 kg chassis scenario and 25 N slide drag carry forward. Extension includes the added camera mass. Withdrawal has two independent checks. One bounds the complete range of 17 camera phases using the desktop rack trials. The second uses the actual 1 ms phase current and motor voltage from the two issued withdrawals, aligned to 48.295 s and 98.390 s. It computes power from paired samples before taking each bin maximum. Both checks put the weakest electrical point on the right drive during transport. [Clock and input hashes](dual-camera.json) · [Actuator trials](../slide-feedback/README.md)

| Open-circuit pack / common resistance | Efficiency | Weakest terminal | Time below 9.5 V |
|---|---:|---:|---:|
| 12.4 V / 25 mΩ | 85% | 10.482 V | 0 s |
| 11.8 V / 27 mΩ | 85% | 9.577 V | 0 s |
| 11.8 V / 27 mΩ | 75% | 9.561 V | 0 s |
| 11.8 V / 32 mΩ | 85% | 9.145 V | 2.82 s |

On the screened pack, the converter inputs stay above 10.017 V. The calculation assumes the regulators are running. Their cold start, coincident inrush, USB enumeration, fuse clearing, battery recovery and processor load with two simultaneous streams need evidence on the assembled system. Record both ends of each rail through start, the two rack pulls, drive acceleration and a loaded-to-idle step. Repeat with each USB power source removed in turn, then with all power removed, checking for backfeed and loss of the corresponding identity stream. A lost image makes its scoring evidence unavailable. Verify the 5.00–5.25 V interval at camera pins, stream continuity, resistance after repeated travel and actual pack-plus-trunk sag before accepting a robot for this role.

During the two issued pulls the minimum slide terminal was 11.462 V on the screened pack, with 32.36 W peak modeled slide-controller input from the 1 ms samples. The independent all-phase envelope gave 11.347 V over the same wall windows. The archived inputs make the calculation reproducible without operating a robot.

Use the registered grip trace and the integrated desktop HAL output folder containing `events.csv` and both `path-nominal-coral-*.csv` files. The replay checks their exact bytes before joining the clocks.

```sh
python fixed-low-power/dual_camera.py artifacts/fixed-low-power-pr63/grips.json artifacts/integrated-match
```
