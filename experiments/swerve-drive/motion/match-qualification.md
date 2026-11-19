# Two station deliveries and a park

The primary red-alliance role starts empty in the middle slot, drives to the station, receives two different CORAL on separate visits, places both on its own L1 and parks. A complete run can earn nine modeled points under the archived competition manual: three for LEAVE, two for each TELEOP L1 and two for PARK. The clock begins with AUTO and ends after the scoring assessment at 156 seconds. [FIRST game manual, G303, G409, §6.5 and Table 6-2](https://firstfrc.blob.core.windows.net/frc2025/Manual/HTML/2025GameManual.htm)

## Opening evidence

The [opening review](opening-line.json) uses fresh executions of all four signed wheel cases from the [reciprocal motor study](energy.json). It intersects the two material bumper covers from the numbered STEP with the starting tape at the actual heading. It then bounds the westmost point of the entire stowed robot, including the steering sweep, at 15 seconds. The geometry allows for rounding of each reported pose coordinate. Each wheel trace must reproduce the previously issued bytes, including all four modules through the disabled interval. Moving the starting pose one metre east is an off-tape control.

| Mass | Wheel and gyro errors | Bumper material over tape at start | Whole-robot clearance east of tape at AUTO end | Modeled LEAVE |
| --- | --- | ---: | ---: | ---: |
| 65 kg | Positive | ≥15,752 mm³ | ≥5.599 m | 3 |
| 65 kg | Negative | ≥15,786 mm³ | ≥5.604 m | 3 |
| 72 kg | Positive | ≥15,752 mm³ | ≥5.600 m | 3 |
| 72 kg | Negative | ≥15,786 mm³ | ≥5.606 m | 3 |

The edge of the native field's starting tape is 0.239 mm east of the layout value. The calculation erodes the tape and reduces the end clearance by that discrepancy plus the pose rounding allowance. All four displaced starts have zero cloth above the tape. The largest reconstructed disabled motor voltage is 3.4 × 10⁻²¹ V. The simulator requests a zero voltage limit and the review allows 10⁻¹⁰ V for numerical cancellation. Each score applies only to LEAVE in that development run.

To reproduce, use the same station/optics inputs and Python packages as the opening study. Supply a fresh directory for the wheel outputs.

```sh
python opening_line.py path/to/project-artifacts path/to/new-wheel-results opening-line.json
```

## What a passing primary match must show

| Wall clock or event | Required observations |
| --- | --- |
| Start and AUTO end, 0–15 s | Actual bumper material overlaps the correct tape at staging. The robot moves until its bumpers clear the tape by AUTO end. Use this run's attained pose and its enabled wheel history. |
| First disable, 15–18 s | All drive, steer and mechanism commands are inhibited. Wheel speeds, gyro history, controller state and field poses carry forward. |
| First station visit and approach | Observe the real station opening from the mounted cameras and debit one available pipe. Show the Java receipt and fresh custody images. Carry that pipe's pose, velocities and contact state through the actual wheel motion and the final sector approach. |
| First placement and return | Observe the pipe clear the robot and qualify on the home trough. Retain it in the field model while the robot withdraws, returns empty and waits for available supply. |
| Second visit and placement | Debit a different pipe upon an observed second feed. Show renewed custody and an independently observed second release. Track both identities, contact with allied robots, trough support, other-level exclusions and the simultaneous-control limit. Use the project's [edition-specific piece assessor](../../six-wheel/scripts/assess_coral.py). |
| Endgame and assessment, 153–156 s | Arrive at the barge from the achieved second release. Show all outputs inhibited after 153 s, assess the actual bumper overlap and carpet support, and recheck both pipes when scoring is assessed. Preserve unscored or ambiguous outcomes. |

Carry one uninterrupted trajectory per case. A checkpoint must identify its input and output state and the clock tick that produced them. Through travel, check field geometry, stock and five peers against the swept robot and load. Check actuator requests against the available shared power. A commanded position, a single sector's isolated release study or the [south-slot fallback](fallback.md) cannot supply any of the missing state transitions.

## Qualification roster

Use eight modeled starts. Cross both registered mass/inertia pairs (65 kg/5 kg·m² and 72 kg/8 kg·m²) with both existing wheel/gyro calibration signs and both independent field-Y placement errors (−3 mm, +3 mm). Use the middle-slot nominal pose from the [registered station study](../../six-wheel/station-landmark/native.json). Apply the prescribed sign to the original 2 mm field-X and 0.12° yaw offsets. Replace the original coupled field-Y offset with the independent value. The controller starts at the common nominal estimate and observes wheel, gyro, camera and mechanism data on their normal schedules. Keep the actual errors and simulated piece state hidden from it.

Require nine modeled points and the whole sequence above on **all eight starts**. This is a project acceptance target for a small set of specified conditions. The previously run 65/72 kg cases are development runs. Repeating them or halving the integration timestep provides regression or numerical evidence.

Before the first counted run, freeze the executable controller, scorer, geometry and observation policy, including the shelf revision, two supply windows and the five peer programs. Pin eight initial states, the roster and its digest using the [Agent FRC cohort procedure](https://github.com/ehartye/agent-frc/blob/v0.29.0/docs/mission-cohort.md). Record every attempted case. A material or logic change starts a new cohort. The proposed full-match configuration is still under development.
