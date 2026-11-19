# Chassis balance and turn traction

<!-- cad-still -->
[![Full swerve robot · stowed, red alliance](../../../../docs/robot/cad/assembly.png)](https://ehartye.github.io/frc-2025-poc/)

**Project CAD · Full swerve robot · stowed, red alliance.** The R3 model brings together the four steerable wheels, fixed-height receiver, electronics and red bumpers. Purchased parts use project-created representations in this public CAD. [Source STEP · 0f96394e6c](../../../../cad/full-swerve-r3/full-robot.step) · [Rotate the model](https://ehartye.github.io/frc-2025-poc/)
<!-- /cad-still -->

The robot can stay upright and still skid before it completes a scheduled turn. The 401-solid design has a positive modeled tipping margin in each of its four nominal poses on level carpet. Its current bill of materials leaves enough uncertainty in mass placement that the planned turn still needs a balance and traction measurement. The tighter design target for loaded carry requires a modeled longitudinal friction coefficient of 0.681 or greater against the earlier carpet assumption of 0.90. [Pose results and input hashes](review.json) · [Drivetrain equations and assumptions](../drivetrain/README.md)

| Loaded condition | Total seated mass | Straight acceleration | Fore/aft support margin | Side support margin in a turn |
|---|---:|---:|---:|---:|
| Receive | 47.54–62.42 kg | 1.20 m/s² | 76.1 mm | 155.7 mm |
| Carry | 47.54–62.42 kg | 1.20 m/s² | 63.1 mm | 156.0 mm |
| Place | 47.54–62.42 kg | 1.20 m/s² | 55.2 mm | 155.7 mm |

Drive commands belong to the transport configuration. The other poses expose how the same demand changes the load line during an unintended movement. The reference rectangle uses the four outer tire centerlines at X ±247.65 mm and Y ±290 mm. This assumes the installed wheels provide that footprint on a level floor. A lifted wheel, a field edge, a collision, a ramp or receiver acceleration needs a separate contact model.

## Mass model

The calculator reopens the installed native pose and checks its SHA-256 and body inventory. It removes the replaced equipment from the drive BOM, uses the updated receiver-drive BOM and allocates the two-module bumper mass over its 44 material bodies. It counts the permanent sleeves separately. It locates the remaining electrical, pneumatic, lift and observation allowances within their CAD envelopes. The unplaced 2–5 kg hardware reserve and the wiring allowance can occupy any part of the complete robot. Material centroids assume uniform density for the specified stock. A seated pipe uses the project mass, radius, length and seat position.

The bound optimizes each mass-weighted coordinate directly over the allowed masses and positions. For straight acceleration it optimizes X plus the height-dependent load shift jointly. For a turn it includes angular acceleration and centripetal acceleration of the offset CG. The yaw-inertia bound encloses each body's full material extent. Required friction and phase current reuse the registered drivetrain calculation at both mass limits. The stated demands apply to the supplied scrub and rolling coefficients. The study includes no measured tire coefficient or impact qualification.

## Build acceptance

With the pipe seated in carry, aim for a horizontal CG within 50 mm of the center and at least 45.2% of the stationary load on each side. The extra stationary allowance preserves 44% on the lighter side during the modeled spin. The original drive configuration also assumes at most 75% of total load on the front and rear axles together during turning. Six stationary readings check initial distribution. Wheel compliance and carpet scrub determine how much transfers during motion.

Use one calibrated scale under each wheel on a level plane. Record the scale uncertainty, bumper configuration, cargo state and exact installed pose hash. The checker reports actual X/Y balance bounds from those readings. It also evaluates a conservative turn with up to 100% of the load on the outer axles. If bounds cross a requirement, record the result before changing the layout or drive trajectory.

```sh
python design/stability/stability.py design/stability/review.json
python design/stability/weigh.py readings.json
```

For a test of the input format, use the following synthetic readings and insert the transport export SHA from the pose receipt. Use `basis: measured` only when the input records a real measurement.

```json
{
  "pose": "transport", "load": "seated", "stepSha256": "<registered STEP SHA-256>",
  "basis": "synthetic", "source": "illustrative equal-load fixture", "units": "kg", "absoluteErrorKg": 0.05,
  "scales": {"wheel-front-L": 9, "wheel-center-L": 9, "wheel-rear-L": 9,
             "wheel-front-R": 9, "wheel-center-R": 9, "wheel-rear-R": 9}
}
```

The equal-load fixture requires 0.625 modeled friction with unrestricted outer-axle loading. Changing the three left readings to 12 kg and the three right readings to 6 kg keeps the total at 54 kg and raises demand to 1.090. Both examples retain the same turn schedule. Neither represents an as-built measurement. [Regression cases](../../tests/test_whole_robot_stability.py)
