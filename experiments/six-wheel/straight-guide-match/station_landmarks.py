"""Use the stereo disparity and known hole heights to require two distinct panel landmarks."""

import math
import numpy as np
import panel


class Landing:
    def __init__(self):
        self.audit = []
        self.fit_events = []

    def observe(self, frames, prior, bias):
        if prior is not None:
            predicted = panel.pixels(prior) + bias
            out = np.full((2, 4, 2), np.nan)
            for eye, frame in enumerate(frames):
                found = np.asarray(panel.components(frame)).reshape(-1, 2)
                for i, point in enumerate(predicted[eye]):
                    hits = found[np.linalg.norm(found - point, axis=1) < 9]
                    if len(hits) > 1:
                        continue
                    if len(hits) == 1:
                        out[eye, i] = hits[0]
            out[:, 2:] = np.nan
            seen = np.isfinite(out[:, :, 0])
            if (
                seen.any(axis=0).sum() < 2
                or not seen.all(axis=0).any()
                or seen.sum(axis=1).min() < 1
            ):
                out = None
            self.audit.append(
                None if out is None else np.isfinite(out[:, :, 0]).tolist()
            )
            return out
        marks = np.full((2, 4, 2), np.nan)
        left, right = map(panel.components, frames)
        for L in left:
            matches = []
            for R in right:
                disparity = L[0] - R[0]
                if abs(L[1] - R[1]) > 1.5 or not 11 <= disparity <= 66:
                    continue
                depth = panel.FOCAL_PX * 75 / disparity
                z = 970 + depth * (
                    -math.sin(panel.PITCH)
                    + (240 - L[1]) / panel.FOCAL_PX * math.cos(panel.PITCH)
                )
                idx = (
                    0
                    if abs(z - panel.CORNERS[0, 2]) < 38
                    else 2
                    if abs(z - panel.CORNERS[-1, 2]) < 38
                    else None
                )
                col = (L[0] + R[0]) / 2 - 320
                if idx is None or abs(col) < 12:
                    continue
                matches.append((idx + int(col > 0), R))
            if len(matches) > 1:
                return None
            if matches:
                j, R = matches[0]
                if np.isfinite(marks[0, j, 0]):
                    return None
                marks[:, j] = [L, R]
        marks[:, 2:] = np.nan
        use = np.isfinite(marks[0, :, 0])
        self.audit.append(use.tolist())
        if not (use[0::2].any() and use[1::2].any()):
            return None
        return marks

    def can_confirm(self, marks):
        valid = np.isfinite(marks[:, :, 0])
        joint = valid.any(axis=0)
        return (
            valid.sum() >= 4
            and joint.sum() >= 2
            and valid.sum(axis=1).min() >= 1
            and valid.all(axis=0).any()
        )
