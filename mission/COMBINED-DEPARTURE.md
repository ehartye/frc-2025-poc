# Can the assembled robot let go and leave?

The 361-part short-belt assembly clears the registered route and supported pipe. Its 67 kg candidate makes the two deliveries in the 16-second traffic reservation with 1.53 s left after return and recovery. A separate failure involves knowing when to move. The original 50 ms dwell after a whole-robot clearance observation misses the prior-contact timing bracket in every tested camera phase.

| Registered observation | 537 mm belt | 420 mm belt |
| --- | ---: | ---: |
| Positive material overlap at nominal shelf-clear event, per inward-loaded belt | 143.40 mm³ | 0 mm³ |
| Overlap per belt after 30 ms | 103.39 mm³ | 0 mm³ |
| Last interval with potential whole-robot contact, including tracking error | +58 ms | +47 ms |
| Conservative bound after the supported pipe settles, 127 ms onset case | 37.98 mm | 35.13 mm |
| Minimum clearance when approaching the second sector at +220 mm | 39.17 mm | 39.17 mm |

Both long-belt risers stay fixed laterally. The counterexample comes from actual displaced belt material envelopes. The short-belt model uses the passive cartridges and unpowered coast through the last part of the shelf. The nominal replays retain custody of each identified pipe through travel and reach separate target positions. All five registered travel legs clear field and parked-body obstacles by at least 10 mm. The merged robot is 1059 mm tall, has an 850 mm square travel footprint, and reaches 535 mm ahead of chassis center when placing. At +180 mm, the original neighboring-sector control still cuts 436.65 mm³ into the prior pipe. [Long-belt native receipt](combined-departure-long.json) · [Short-belt native receipt](combined-departure-short.json)

## Reloading the actual design

Each run reopens all named solids in **all four supplied STEP files**. The analysis matches identities, validates the supplied motion declarations against the poses, checks the registered receiver by native solid intersection, and stores raw STEP hashes. It sweeps rigid frames, full rotating pinion bounds, straight variable-length return ropes and the stroke-dependent draw wire. The conservative bounds permit 15 mm axial outer-slide tracking error, half that for the middle stage and 1.5 mm independent lateral grip motion. Native positive-volume counterexamples at placement use the authored compliant model, 2 mm forward endpoint error and both cartridges 1.5 mm inward. Fixed hardware and springs follow their own kinematics. A sampled zero overlap can still have tangency or an unresolved interval between samples.

The historical [96-part registered study](REGISTERED-FIELD.md) remains independently reproducible. The [203-part comparison](combined-departure-203.json) and [345-part intermediate comparison](combined-departure-345.json) preserve the supplied inventories. The source field model, hold reference, tuned contact coefficients, provisional drive dynamics and route assumptions are shared inputs. They cannot establish the timing or geometry of an unmeasured competition robot.

## Whole-robot clearance and timing

The stereo measurement reconstructs independently bounded estimates of both CORAL end centers. It checks identity and length, subtracts the exterior radius, a 0.5 mm size allowance and a 1 mm registration allowance, then requires at least 1 mm beyond an X plane at 537 mm. This plane includes the entire placing robot and its admissible +2 mm endpoint error before withdrawal. Eight finite rays at each earliest/latest qualified exposure check both native camera locations to both modeled end-center and upper-rim landmarks, omitting the camera's own solid proxy. Actual rim visibility, correspondence, camera calibration and actual actuator startup remain unmeasured.

On the short receiver all 17 frame phases see the modeled pipe beyond the whole robot at +55 to +71 ms after the reference shelf event at ±0.10 pixel. At ±0.25 pixel the exposures are +64 to +80 ms. The tested extreme exposure rays clear the complete assembly. The new observation carries the current assembly identity, piece identity and actual exposure. A shelf crossing or a timer alone cannot provide it. [Physical trace and stereo frames](combined-departure-short.json)

The separate [command-clock study](short-vision-clock.json) evaluates a zero deliberate wait after this full observation alongside the original 50 ms delay. It tracks ±1 ms synchronization, 8 ms processing/transport, 0–20 ms scheduler phase and a provisional 0–10 ms command-to-actuator onset allowance. Its separate 3 ms allowance bounds the earlier physical contact pad timestamp. These terms are design budgets. Each trial reports individual exposure, worst possible start, eligibility, full-assembly travel clearance, tested dynamic onsets and late recontact controls.

With a fresh ±0.10-pixel observation and zero deliberate wait, all 68 exposure phases across both specimens and both tested coast-friction coefficients meet the 150 ms prior-contact bound. Their physical withdrawal starts 62–110 ms after the independent 1 mm shelf truth. Bounding the entire range of commanded motion, including tracking and grip errors, leaves at least 36.12 mm separation after support. The combined sweep loses possible contact by +48 ms. The ±0.25-pixel case with coast friction 0.20 misses the guard in one of 17 phases for each specimen. Holding the original 50 ms fails the timing guard in all phases. The 434 discrete actuation-onset tests drawn from both policies and both pixel tiers sustain support. A 500 ms delay recontacts at higher coast friction and a 600 ms delay recontacts in all four trials. Optical qualification still requires measured calibration, latency and physical correspondence.

The [67 kg short-belt route](67kg-short-field-clock.json) uses a 0.64 m/s² spin-wheel acceleration, 1.20 m/s² straight-wheel acceleration, 1.25 m/s peak wheel speed and 1.25 rad/s peak yaw with a 580 mm track. Other robots keep their independent prior schedules. The reference stock and traffic case supports A/B at 62.888/114.413 s and returns at 133.465 s under the nominal 127 ms retreat onset. Competition traffic, stock hand timings, friction, calibrated actuation and official scoring still require separate evidence. [Drive-only mass sensitivity](67kg-field-clock.json) · [Traffic and finite-stock cases](MOVING-TRAFFIC.md)

## Reproduction

Generate the four short-configuration STEP files, motion inventory and rotary bounds with the competition-package source. Set `POSE_DIR` to its export directory. The explicit hashes in the receipt identify the particular input bytes. A new export can have a different STEP header. Each input accepts a separately supplied exact file using `--stowed`, `--receive`, `--transport` or `--place`.

```sh
python mission/review_combined_departure.py --poses "$POSE_DIR" --inventory "$POSE_DIR/part-frames.json" --radial "$POSE_DIR/rotation-bounds.json" --outer-error-mm 15 --grip-error-mm 1.5 --receiver-source design/receiver-drives/receiver_drives.py --contact-root . --output mission/combined-departure-short.json
python mission/short_vision_clock.py --poses "$POSE_DIR" --contact-root . --pose-receipt mission/combined-departure-short.json --output mission/short-vision-clock.json
python mission/heavy_field_clock.py --contact-root . --output mission/67kg-short-field-clock.json
```

The preserved long package uses the original registered contact trace and the 537 mm actuator geometry authored at receiver revision `f830676`. Reopen those four matching STEP files using their inventory and rotary bounds, pass that revision's original receiver source for the inward-loaded comparison, and omit the passive short-contact override. Both native receipts name the source digests, body identities, geometry variants and registration result.
