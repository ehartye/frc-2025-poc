# Can the carried CORAL reach the low trough?

The Red 2 robot carried its station-fed CORAL to two measured stops within 30 mm of the planned reef datum. Both signs kept the same pipe, synchronized wheel state and both rim-probe signals. The camera fixture, capture yokes and slide each do a different job. The comparison starts from the [three frozen inputs](cohort.json) and the [earlier cam and capture study](../cam-scoring/study.md).

## Drive and observe

|Calibration|Start correction|Stopped|Dock X/Y in mm|Minimum pin engagement|Driven 5 ms samples|
|---|---:|---:|---:|---:|---:|
|Positive|53.130 s|56.100 s|−29.86 / −178.68|21.69 mm|594|
|Negative|53.140 s|55.210 s|−29.58 / −179.11|21.69 mm|414|

Station stereo establishes which pipe entered the receiver. Both latch switches, both rim probes and both occupied ports subsequently give the controller a fresh retained-cargo decision every 5 ms. At the reef, the controller uses recorded field-tag exposures, the gyro and two fixed views of a surveyed bolt to refine its dock. Removing one view or withholding camera packets on either sign makes the controller brake, with 1.73–2.16 mm of travel after the inhibit. Those four stops need a separate path before another placement can begin. [Wheel, cargo and camera receipts](results.json)

For the held-out minus calibration, a 0.20 mm station plunger gap changes the initial floor and pipe response. Six new field-backed exposures pass the dark-facing gate and none pass the matched-brightness gate. The empty control also holds. Both ports and the physical yokes confirm the loaded pipe through departure at 27.935 s. [Station and route records](results.json)

## Change the scoring stroke

The frozen command was 345 mm. The closest dock pose consistent with the actual camera histories, the SP1 transducer's ±2 mm travel allowance, rail pitch up to ±0.04 rad and ±5 mm of heave leave insufficient clearance against the front wall at that setting. A 337 mm command keeps the signed whole-robot separating plane positive by 4.33 mm and 3.21 mm. An 8 mm overtravel at the shorter setting again loses that guarantee. The check uses all 1,268 installed bodies of the reopened assembly, the stowed jack and actual low wall in the red field. A negative bound indicates where an exact solid check could improve the estimate. [Geometry and tolerance cases](clearance.json)

The original 23-link power and USB tails first intersect the fixed right climber motor at sampled strokes of 308.75 and 319.75 mm. The alternate 22-link layout shifts the moving end 95 mm rearward. For a 345 mm sweep, the entire data and power centerlines retain at least 24.87 and 26.94 mm to the motor at maximum jacket diameter. The branch's project-authored STEP contains 19 installed support and connector shapes and two approach gauges. The stationary power feed has its own route and service-removal clearance. Carrier end brackets, cable connector strain relief and protection of the return bend still need an installed design. Carrying these candidate shapes through this study leaves the issued robot and its inspection mass unchanged. [Wire geometry and sources](candidate-wiring.json) · [Continuous clearance](wiring-clearance.json)

## Reproduce the study

All lengths in the tube and rail solvers are meters. CAD registrations use millimeters. Set `ARTIFACTS` to the shared baseline artifact folder, including the source R4 stereo meshes, signed cam feeds and both signed cam route states from the preceding study. Each command below runs in a fresh interpreter to keep its registered modal basis.

```text
python low-placement/dock.py ARTIFACTS plus
python low-placement/dock.py ARTIFACTS plus --cargo
python low-placement/dock.py ARTIFACTS plus --mode missing-eye
python low-placement/dock.py ARTIFACTS plus --mode lost-packets
python low-placement/rack.py ARTIFACTS plus --mm 337
python low-placement/delivery.py ARTIFACTS plus --stroke-mm 337 --dt-us 62.5
python low-placement/clearance.py ARTIFACTS low-placement/clearance.json
python low-placement/wiring.py
```

Repeat the signed commands with `minus`. From `experiments/swerve-drive`, regenerate the held-out first service with `python score-stroke/cam_service.py ARTIFACTS/r4-score-stroke minus 31.25 --source-default-blas --gap-mm .2 --destination ARTIFACTS/low-placement/holdout-service`; run `low-placement/heldout.py ARTIFACTS --stage STAGE` for `optics`, `station-wheels`, `station-cargo`, `route-wheels`, and `route-cargo` in that order. Collect the actual runs using `python low-placement/collect.py ARTIFACTS low-placement/results.json`.

The strategic Red 1 volunteer delivery rate of 15 in 135 seconds describes another player and field program. Count a Red 2 delivery only when its own named pipe settles in the target after separating from the robot. An independent parked run and independent placements from the old release fixture remain separate results.
