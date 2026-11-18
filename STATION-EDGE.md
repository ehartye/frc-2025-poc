# Where does an incoming CORAL leave the station lip?

The existing placement experiment starts with a loaded robot. A repeatable station-fed L1 cycle also needs an incoming piece. Before choosing a receiver position, this experiment compares two ways a transverse CORAL can leave a chute edge. It gives a downstream contact model an initial position and velocity for each assumed feed speed.

## Conditions

The 2025 manual describes a 55° sloped chute in §5.6.2 and a nominal 4.5 in outside pipe diameter in §5.7.1. The model uses the corresponding radius of 57.15 mm. For this experiment the tangent descends 35° from horizontal. The chute description alone does not establish the local lip angle or edge radius. The speed cases are hypotheses. [FIRST 2025 game manual, English HTML edition, retrieved November 18, 2026](https://firstfrc.blob.core.windows.net/frc2025/Manual/HTML/2025GameManual.htm)

Treat the tube as a rigid circle crossing a sharp, frictionless edge at right angles to its axis. The lip is the origin, positive X points along the feed toward the robot, and positive Z points up. The angle θ runs from positive Z toward positive X. The initial value θ₀ is 35°. Gravity is 9.80665 m/s². Side walls, receiver surfaces, tube wear and compliance are outside this model. Frictionless contact exerts no torque about the pipe center.

## Departure condition

On a supported arc, the center follows a circle of radius r around the lip. The normal reaction per unit mass is `g cos(θ) − v²/r`. At the entry speed `sqrt(g r cos(θ₀))`, the reaction reaches zero immediately. Faster entries also separate at the initial tangent. For a slower entry with speed v₀, the center continues to drop as gravity increases its speed. Energy conservation and zero reaction give the first separation angle:

    v² = v₀² + 2 g r (cos(θ₀) − cos(θ))
    cos(θ) = (2 cos(θ₀) + v₀² / (g r)) / 3

Run the calculation from the checkout root:

```sh
python scripts/station_edge_contact.py --speeds 0.5 1 2
```

| Entry speed (m/s) | Contact after the tangent | Separation angle | Center X, Z from lip (mm) | Velocity X, Z (m/s) |
| ---: | --- | ---: | ---: | ---: |
| 0.5 | Supported arc | 46.0° | 41.1, 39.7 | +0.434, −0.449 |
| 1.0 | Immediate separation | 35.0° | 32.8, 46.8 | +0.819, −0.574 |
| 2.0 | Immediate separation | 35.0° | 32.8, 46.8 | +1.638, −1.147 |

The transition is 0.678 m/s in this geometry. At 0.5 m/s, starting free flight at the tangent would place departure about 8.3 mm too early horizontally and 7.1 mm too high. Use the respective separation states in the next receiver experiment. Sweep the tube against the registered station surface and receiver entrance from approach through first receiver contact. A penetration before separation rejects the sharp-edge geometry for that case. A clear transition gives a candidate first impact for a separate bounce and capture study.
