# Trough contact and receiver withdrawal

A pipe can clear the receiver, touch the trough and then roll back into the robot. In the reference continuation, leaving the R6 shelf fixed returns the pipe to its entrance **0.614 s after first floor contact**. A 1.2 s wait gives the same result. The original 62.136 s two-trip scenario used that late wait as a synthetic clearance assumption. Use the revised timing experiment below when assessing this candidate.

| Motion and contact hypotheses | Result from exact gravity-to-plane contact |
| --- | --- |
| Fixed shelf, friction 0.35, zero initial axial roll | At +0.6137 s the finite pipe overlaps reopened receiver STEP geometry by 4.816 mm³. The solver stops at that first sampled overlap. |
| Start retraction 1.2 s after first contact, same pipe | The same receiver recontact occurs before the command starts. |
| Start retraction 50 ms after a 1 mm whole-pipe/shelf gap, at 0.15 m/s | The pipe reaches the trough lip at +0.6905 s. At +2.0 s both the lip and two finite floor regions still support the model. Final continuous low-motion double-contact duration was 1.20 s. Minimum modeled receiver clearance from first floor contact was 48.59 mm. |
| Early retraction, zero friction | Reaches the conservative inner-riser boundary at +0.0895 s. Its material response is outside this continuation. |
| Early retraction, friction 0.35, entering roll −10 rad/s | Reaches the inner-riser boundary at +0.0903 s. Entering roll +10 rad/s instead reaches sustained floor/lip contact. These are unmeasured sensitivities. |

**Implication for the next design test.** A shelf-clear pose or cleared entry beam cannot by itself establish persistent release. Use a fresh exit/separation observation to time actual slide feedback. Compare the rejected late start with the early withdrawal under measured pipe roll, floor contact and actuator lag. The [structural screen](../structural/README.md) also gives a reason to retract before a loaded crossing. Actual scoring still needs a rule-edition assessment with sufficient robot, piece and match observations.

## Bodies, forces and numerical checks

The continuation uses the same upstream piece ID, actual belt exit state and the external target region previously used in the [station-to-reef mission](README.md). It reconstructs the exact first contact from the free-flight segment to avoid restarting with numerical penetration. Two panel support witnesses span the transverse pipe and must lie in the trimmed regions with the mounting-hole exclusions. The model includes the front lip and stops on a conservative inner-riser bounding box or later robot-shelf contact. The R6 receiver follows its actual section and original frame. Native checks use the reopened STEP and a finite outer cylinder at two selected states in each middle-resolution case.

Each modeled support uses a non-tensile 60 kN/m normal spring and a damped response with damping ratio 0.6. Regularized Coulomb tangential force uses a 0.01 m/s slip scale and the stated coefficient. The hollow pipe uses the thin-ring approximation about its transverse axis. Initial axial roll is an experiment input because the preceding model did not resolve it. Gravity, translation, angular motion and contact work continue throughout the 2 s observation. The reference reaches 2.27 mm peak model compression, which needs comparison with actual materials. These are desktop design sensitivities.

A sustained result requires at least 250 ms uninterrupted with positive floor/lip normal forces, speed below 0.01 m/s, absolute axial rotation below 0.2 rad/s and at least 1 mm receiver clearance. The 50, 100 and 200 µs replays retained the displayed outcomes. None of the eighteen runs showed mechanical plus spring energy above the initial energy. A resolved receiver STEP intersection establishes geometry for that sample. The inner-riser event remains a conservative envelope finding.

## Repeat within the match clock

The [offline competition controls](../competition-controls/README.md) use the separate finite-pipe exit witness, 50 ms delay and 150 mm/s requested withdrawal. That controller's desktop tests cover sequencing. Physical sensor location, latency, slide tracking and whole-machine clearance still require integrated verification.

The phase rehearsal uses the identical upstream body state for each continuation. In the reference TELEOP it reaches sustained modeled trough support at **20.935 s** for `coral-A` and **51.935 s** for `coral-B`. It preserves the two assigned source pieces, separate sectors, 35 s second-feed availability and a finite clock. With independent synthetic whole-robot and sector-clear observations the empty robot recovers at **60.363 s**. Without those observations the evidence-only rehearsal holds at the first 20.935 s support result. The shelf and trough were modeled; other robot parts, all other robots, full field collisions, loads, actuator response, measurement latency and final official assessment remain open. Both editions leave earned points unknown.

```sh
python scripts/station_l1_settle.py --withdrawal fixed
python scripts/station_l1_settle.py --withdrawal after-separation --initial-roll -10
python scripts/station_l1_phase.py --phase TELEOP --trough-continuation
python scripts/station_l1_phase.py --phase TELEOP --trough-continuation --assume-clear-witnesses
python mission/replay_post_contact.py
```

The last command uses OCP 8.0.1 and reproduces the [source-pinned sensitivity and native geometry receipt](post-contact-receipt.json). Add `--full` to the single-contact command for time, X/Z, velocity and axial roll sampled every 10 ms.
