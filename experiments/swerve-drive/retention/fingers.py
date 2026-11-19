"""Twin slide-mounted end yokes: geometry in mm, forces in N, time in seconds."""

import numpy as np

PINS_X = (39.0, 110.0)
PIN_Z = 579.0
PIN_R = 4.0
PROBE_XZ = (75.0, 611.0)
TIP_R = 0.75
INNER, OUTER, HALF = 50.8, 57.15, 150.8125
OPEN, CLOSED, SPEED = 0.0, 35.0, 150.0


def intercept(center, axis, x, z, side):
    """Y and radial distance of a point in the exact finite end plane."""
    radial = np.array([x - center[0], 0.0, z - center[2]])
    y = (side * HALF - np.dot(axis, radial)) / axis[1]
    radial[1] = y
    return center[1] + y, np.linalg.norm(radial - side * HALF * axis)


def geometry(center, axis, stroke):
    per_side = []
    for side in (-1, 1):
        y, radius = intercept(center, axis, *PROBE_XZ, side)
        # Complete circular probe footprint, conservatively projected into the tilted end plane.
        land = min(radius - INNER, OUTER - radius) - TIP_R / abs(axis[1])
        compression = max(0.0, side * y - (183.0 - stroke)) if land >= 0 else 0.0
        per_side.append(
            dict(side=side, rimY=y, land=land, compression=compression, pins=[])
        )
        for x in PINS_X:
            lip, radial = intercept(center, axis, x, PIN_Z, side)
            nose = side * (161.0 - stroke)
            overlap = side * (lip - nose)
            point = np.array([x, nose, PIN_Z])
            v = point - center
            radius_at_nose = np.linalg.norm(v - axis * (v @ axis))
            entry_radius = PIN_R / abs(axis[1])
            per_side[-1]["pins"].append(
                dict(
                    x=x,
                    lip=lip,
                    nose=nose,
                    overlap=overlap,
                    clearance=INNER
                    - max(radial, radius_at_nose if overlap > 0 else radial)
                    - entry_radius,
                )
            )
    return per_side


def mechanics(pipe, stroke, dt):
    """Ball-ended axial feelers and frictionless, unilateral pin/inner-wall forces.

    Geometry uses a conservative cylindrical nose. A candidate striking an end face
    during entry fails the design screen before its contact is used to authorize motion.
    """
    center = pipe.p * 1000
    axis = pipe.a
    samples = geometry(center, axis, stroke)
    force = np.zeros(3)
    torque = np.zeros(3)
    omega = pipe.angular(pipe.l)
    for s in samples:
        side = s["side"]
        arm = (np.array([PROBE_XZ[0], s["rimY"], PROBE_XZ[1]]) - center) / 1000
        speed = side * (pipe.v + np.cross(omega, arm))[1]
        # Two independent spring probes. The hard shoulder lies beyond 5 mm compression.
        s["probeN"] = (
            max(0.0, 0.45 * s["compression"] + 0.7 * speed)
            if s["compression"] > 0
            else 0.0
        )
        load = np.array([0.0, -side * s["probeN"], 0.0])
        force += load
        torque += np.cross(arm, load)
        for pin in s["pins"]:
            pin["forceN"] = 0.0
            if pin["overlap"] <= 0 or pin["clearance"] >= 0:
                continue
            # Extremes of radius in the engaged straight segment occur at an endpoint.
            points = np.array([[pin["x"], y, PIN_Z] for y in (pin["lip"], pin["nose"])])
            offsets = points - center
            radial = offsets - np.outer(offsets @ axis, axis)
            j = np.argmax(np.linalg.norm(radial, axis=1))
            normal = radial[j] / np.linalg.norm(radial[j])
            arm = (offsets[j] + normal * PIN_R) / 1000
            speed = float((pipe.v + np.cross(omega, arm)) @ normal)
            pin["forceN"] = max(0.0, -12.0 * pin["clearance"] - 8.0 * speed)
            load = pin["forceN"] * normal
            force += load
            torque += np.cross(arm, load)
    pipe.v += dt * force / 0.816466266
    pipe.l += dt * torque
    return samples


def observations(rows, stroke, fault):
    # Each end has an independent completion switch and series-spring displacement pickup.
    # Quantization and fixed offsets are applied before the control logic.
    def quantize(value, quantum):
        return round(value / quantum) * quantum

    readings = []
    for s in rows:
        sign = s["side"]
        closed = stroke >= 34.8
        displacement = s["compression"]
        if fault == "missing-probe" and sign == 1:
            displacement = 0.0
        if fault == "stuck-open" and sign == -1:
            closed = False
        if fault == "empty":
            displacement = 0.0
        readings.append((closed, quantize(displacement + 0.15 * sign, 0.1)))
    return readings


def permits(samples):
    return len(samples) == 5 and all(
        on and 0.65 <= mm <= 4.5 for pair in samples for on, mm in pair
    )
