# The second station feed after a real return

<!-- cad-still -->
[![Four-module swerve frame and mounting hardware](../../../docs/robot/cad/assembly.png)](../../../docs/robot/cad/index.html)

**Project CAD · Four-module swerve frame and mounting hardware.** Original robot structure for the swerve candidate. The project-local Fusion assembly adds four manufacturer module occurrences. [Source STEP · f8b75f2df4](../../swerve-drive/packaging/swerve-frame.step) · [Rotate the model](../../../docs/robot/cad/index.html)
<!-- /cad-still -->

The [complete relieved-receiver match](../relieved-match/README.md) follows both pipes from the start on this revised 597-body receiver. The station-only cases below retain their original scope.

A fixed 1.3 mm change in the destination of the return route reduces the measured lateral error to ±1.24 mm at the station. Under the stated model, a revised receiver and controller can take the incoming second pipe, observe custody and hold it until scheduled departure. The revised receiver has **597 bodies**. The first-trip score and park used the earlier **593-body** receiver. Seven points remain the strongest complete single-revision result. [Try the interactive station feed](replay.html) · [First score and park](../spatial-contact/README.md) · [Departure states and clocks](handoff.json)

| Configuration on the second return | Result |
|---|---|
| Original wheel demands, issued receiver | The five achieved poses leave at least 9.15 mm between the robot and the actual station. One asks the cartridge for 2.55 mm, beyond its 1.5 mm stop. Four port traces reach IN_BAND under the issued contact law. Some of those trajectories strike real rigid material before that law’s first working-belt contact. |
| Fixed 1.3 mm return bias, issued receiver | The wheel schedule grows by 0.102 s. Arrival still precedes B’s feed by 5.770 s. All five trajectories reach both ports in the virtual belt model. Four intersect a rigid upper cheek before working-belt contact with the unoccupied cartridge against its allowed inboard stop. |
| Biased return, relieved cheeks and shimmed springs | The 597-body low robot incorporates the original eight updated cheek/insert parts, two stop settings, revised springs and four shims at zero stroke. Two of five cases reach the old dual-port confirmation. The others stop too soon to clear the seat port. Alternative 0.45, 0.55 and 0.60 m/s counterfeed speeds retain that failure. |
| Same low receiver, active reverse probe | The original two-port sequence can be observed on all five attained returns at both 0.25 and 0.50 ms. After the seat-only edge, the controller requests −0.14 m/s while looking for both ports to clear. The pulse starts at 160 ms and ends by 480 ms after braking begins. The original 0.16 m/s probing direction then returns the pipe into the dual-beam window. |

The new wheel demands start at A’s actual post-score pose and terminate at the registered station. The last 0.363° turn is too small for the full-circle screening disk to describe its clearance tightly. Bounding the travel of every corner over that actual turn gives a minimum 2.81 mm to fixed obstructions. Every native final pose was also checked against 47 objects from the official corner station. With an 8 mm shift toward the wall and 1° yaw, the robot bumper intersects GE-25007. The return uses the same wheel limits and same five moving traffic programs as the first-trip field study. [Return and traffic bounds](return.json) · [Station structure](dock.json)

## What the observed seat permits

The source plant releases B at the original human feed clock with the same ramp, spin, mass, annular end caps and finite working belts. It transports all three linear and angular velocity components plus the cylinder axis. The two cartridge contact patches share Coulomb traction in their working belt planes. An inelastic, non-tensile two-point shelf impulse acts on the finite cylinder. The collision and load screens assume the project’s stiffness, damping, friction, source orientation and pipe dimensions. Empty station views and pipe angles come from the assembled fixed cameras and official station solids. A matched empty image is available before each feed. [Original actuator and spring data](../design/receiver-drives/README.md) · [Existing relief design](../station-relief/README.md) · [Native rigid-pipe failures](rigid.json)

On the revised robot, all five fine-step trials give five independent stationary-camera exposures in addition to the two port signals. The desktop scheduler first corroborates custody at local 1.480–1.500 s. A missing seat port, undocked input, +3 mm source shift, stale frames and wrong-piece identity each inhibit the corresponding permit. A second run retains port and stereo observations through the scheduled 77.43453 s wall-clock departure. A deliberately late packet stream just before departure expires the previous permit. The handoff contains position, velocity, orientation, spin, robot offset and timestamps from one continuous run. [Port and stereo evidence](custody.json) · [Source and control parameters](qualification.json)

The native swept test covers the complete receiver. Three pressure-pad outer bounds intersect the coarse tool near touchdown. Adaptive distances to the actual finite annulus retain at least **0.0356 mm** through that interval. Two right-hand cheek bounds overlap when the tube and cartridge positions vary independently. A contacted working belt constrains the simultaneous cartridge displacement. If local indentation remains at or below **0.25 mm**, the minimum cheek margin is **0.0645 mm** including a 0.05 mm intersample reserve. With 0.50 mm indentation this bound reaches **−0.0867 mm**. Manufacturing tolerances, free pipe side offset, working belt compression, panel-to-chute registration and available traction remain unmeasured. The upper marker also lies above these downward-facing cameras at the stop. A stationary image of an empty receiver cannot detect an independently shifted incoming source. [Pad distances](pads.json) · [Coupled cheek bounds](coupled-clearance.json) · [Revised swept inventory](probe-native.json)

## Reproduce

Use project Python with NumPy, VTK and OCP, the retained official field STEP, supplier lamp STEP, JDK 17 and the populated project Gradle cache. The local cache contains the source trajectories, raw images, controller logs and PNG previews. All runs are desktop-only. From the repository root, build the return plan first and supply the same cache directory each time.

```sh
python return-station/route.py
python return-station/registration.py "$LAMP_STEP" "$FIELD_STEP" "$CACHE"
python return-station/feed.py "$CACHE" "$JDK17"
python return-station/images.py "$CACHE" "$FIELD_STEP"
python return-station/native.py "$CACHE" "$LAMP_STEP"
python return-station/rigid.py "$CACHE" "$LAMP_STEP"
python return-station/registration.py "$LAMP_STEP" "$FIELD_STEP" "$CACHE" --relief
python return-station/feed.py "$CACHE" "$JDK17" --relief
python return-station/native.py "$CACHE" "$LAMP_STEP" --relief
python return-station/tuning.py "$CACHE" "$JDK17"
python return-station/feed.py "$CACHE" "$JDK17" --probe
python return-station/native.py "$CACHE" "$LAMP_STEP" --probe
python return-station/images.py "$CACHE" "$FIELD_STEP" --probe
python return-station/pads.py "$CACHE" "$LAMP_STEP"
python return-station/coupled.py "$CACHE" "$LAMP_STEP"
python return-station/feed.py "$CACHE" "$JDK17" --hold
python return-station/native.py "$CACHE" "$LAMP_STEP" --hold
python return-station/images.py "$CACHE" "$FIELD_STEP" --hold
python return-station/custody.py "$CACHE" "$JDK17" "$GRADLE_CACHE"
python return-station/handoff.py "$CACHE"
python return-station/viewer.py "$CACHE"
```
