"""Fit the projected circular ends to certified visible contours from both roof pupils."""

import math
import numpy as np


def extract(images, reference, optics):
    ends = []
    for eye, (im, (gray, z)) in enumerate(zip(images, reference)):
        known = z < 3490
        bright = (im >= 185) & (im.astype(int) - gray.astype(int) > 36) & known
        where = []
        for row in range(4, im.shape[0] - 4):
            pixels = np.flatnonzero(bright[row])
            pixels = (
                pixels[pixels < optics.W / 2]
                if eye == 0
                else pixels[pixels > optics.W / 2]
            )
            if len(pixels) < 20:
                continue
            outer = int(min(pixels) if eye == 0 else max(pixels) + 1)
            if not 4 < outer < optics.W - 4:
                continue
            if not np.all(known[row, outer - 3 : outer + 3]):
                continue
            # The immediately outward neighbor must be part of the surveyed backing
            # behind the pipe. Reject apparent contours along nearer foreground hardware.
            back = float(np.min(z[row, outer - 3 : outer + 3]))
            if back < optics.ORIGIN[2] - 575:
                continue
            where.append((row + 0.5, outer, back))
        if len(where) < 45:
            return None
        pts = np.array(where)
        if np.ptp(pts[:, 0]) < 65:
            return None
        ends.append(pts)
    return ends


def project(pose, row, eye, optics):
    """Intersect a single image row with a finite pipe's planar circular rim.

    A normalized ray D=(q,d,-1) reaches the end plane at distance
    t=n/(q*ax+d*ay-az). Substitution in |origin+tD-end|²=R² yields
    A*d²+B*d+C=0. Each pupil uses the root on its outward side.
    """
    x, y, z, yaw, tilt = pose
    a = np.array(
        [math.sin(yaw) * math.cos(tilt), math.cos(yaw) * math.cos(tilt), math.sin(tilt)]
    )
    sign = 1 if eye == 0 else -1
    end = np.array([x, y, z]) + sign * 0.5 * optics.L * a
    origin = optics.ORIGIN + np.array([0, optics.EYES[eye], 0])
    bx, by, bz = origin - end
    n = (end - origin) @ a
    q = (optics.H / 2 - row) / optics.F
    c = q * a[0] - a[2]
    h = bx * q - bz
    k = (bx * bx + by * by + bz * bz) - optics.R**2
    A = k * a[1] ** 2 + 2 * n * by * a[1] + n * n
    B = 2 * (k * c * a[1] + n * (h * a[1] + by * c))
    C = k * c * c + 2 * n * h * c + n * n * (q * q + 1)
    disc = B * B - 4 * A * C
    u0 = (-B + np.sqrt(np.maximum(0, disc))) / (2 * A)
    u1 = (-B - np.sqrt(np.maximum(0, disc))) / (2 * A)
    selected = np.maximum(u0, u1) if eye == 0 else np.minimum(u0, u1)
    return optics.W / 2 - optics.F * selected, disc >= 0, n / (c + a[1] * selected)


def fit(images, reference, optics, position, yaw):
    if (
        position["status"] != "position-witness"
        or yaw is None
        or max(map(abs, yaw)) > math.radians(2)
    ):
        return {"status": "unbounded-position-or-axis"}
    data = extract(images, reference, optics)
    if data is None:
        return {"status": "too-few-backed-end-pixels"}
    u = position["u_interval_mm"]
    y = position["y_interval_mm"]
    h = position["bottom_relative_to_shelf_mm"]
    low = np.array([u[0], y[0], 557.15 + h[0], yaw[0], math.radians(-3)])
    high = np.array([u[1], y[1], 557.15 + h[1], yaw[1], math.radians(3)])
    point = (low + high) / 2

    def residual(pose):
        rows = []
        for eye, arr in enumerate(data):
            curve, ok, d = project(pose, arr[:, 0], eye, optics)
            visible = arr[:, 2] > d + 2.0
            rows.append(np.where(visible, np.where(ok, curve - arr[:, 1], 25.0), 0.0))
        return np.concatenate(rows)

    steps = np.array([0.03, 0.03, 0.03, 0.0001, 0.0001])
    for _ in range(16):
        err = residual(point)
        J = np.stack(
            [
                (residual(point + h) - residual(point - h)) / (2 * s)
                for s, h in zip(steps, np.diag(steps))
            ],
            axis=1,
        )
        weight = 1 / np.sqrt(np.maximum(1.0, abs(err) / 1.25))
        update = np.linalg.lstsq(J * weight[:, None], -err * weight, rcond=None)[0]
        next_point = np.clip(point + update, low, high)
        if np.linalg.norm(next_point - point) < 1e-4:
            break
        point = next_point
    quality = []
    for eye, arr in enumerate(data):
        curve, ok, d = project(point, arr[:, 0], eye, optics)
        visible = arr[:, 2] > d + 2.0
        good = ok & visible & (abs(curve - arr[:, 1]) <= 1.5)
        accepted = arr[good]
        if len(accepted) < 3:
            quality.append({"visible": int(visible.sum()), "agreed": len(accepted)})
            continue
        # A line tangent/diagonal or synthetic bounding box cannot describe a curved rim.
        line = np.polyfit(accepted[:, 0], accepted[:, 1], 1)
        deviation = float(
            np.max(abs(accepted[:, 1] - np.polyval(line, accepted[:, 0])))
        )
        quality.append(
            {
                "visible": int(visible.sum()),
                "agreed": len(accepted),
                "rowSpan": float(np.ptp(accepted[:, 0])),
                "lineDeparturePx": deviation,
            }
        )
    ok = all(
        s["agreed"] >= 45
        and s["agreed"] >= 0.88 * s["visible"]
        and s.get("rowSpan", 0) >= 65
        and s.get("lineDeparturePx", 0) > 3.5
        for s in quality
    )
    return {
        "status": "two-projected-rims" if ok else "ambiguous-rims",
        "eyes": quality,
        "trialPose": point.tolist(),
    }
