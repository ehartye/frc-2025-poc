"""Centerlines for the fixed feeds and horizontal shelf return, in millimeters."""
import math
import numpy as np

RADIUS = 100.0
FIXED_X = -5.0
MOVING_X = -72.5
STROKE = 370.0
LOW_Z = 600.0
HIGH_Z = LOW_Z + 2 * RADIUS
PITCH = 2.0
CHAIN_PITCH = 30.5
CHAIN_K = 380.0
CHAIN_LINKS = math.ceil((STROKE/2 + abs(MOVING_X + STROKE/2 - FIXED_X) + CHAIN_K) / CHAIN_PITCH)
MANAGED_LENGTH = CHAIN_LINKS * CHAIN_PITCH


def unit(vector):
    return vector / np.linalg.norm(vector)


def line(a, b):
    length = float(np.linalg.norm(b - a))
    return length, np.linspace(a, b, max(1, math.ceil(length / PITCH)) + 1)


def arc(center, start_radius, tangent, angle):
    radius = float(np.linalg.norm(start_radius))
    length = radius * angle
    angles = np.linspace(0, angle, math.ceil(length / PITCH) + 1)[:, None]
    return length, center + np.cos(angles) * start_radius + np.sin(angles) * radius * tangent


def fillet_path(vertices, radius):
    vertices = np.asarray(vertices, dtype=float)
    start = vertices[0]
    sections = []
    for a, corner, b in zip(vertices, vertices[1:], vertices[2:]):
        incoming, outgoing = unit(corner - a), unit(b - corner)
        cosine = float(np.dot(incoming, outgoing))
        angle = math.acos(np.clip(cosine, -1, 1))
        retreat = radius * math.tan(angle / 2)
        before, after = corner - incoming * retreat, corner + outgoing * retreat
        if np.dot(before - start, incoming) < -1e-7:
            raise ValueError("Insufficient straight length for the selected bend radius")
        center = before + radius * unit(outgoing - incoming * cosine)
        sections.extend((line(start, before), arc(center, before - center, incoming, angle)))
        start = after
    if np.dot(vertices[-1] - start, outgoing) < -1e-7:
        raise ValueError("Last segment too short for the selected bend radius")
    sections.append(line(start, vertices[-1]))
    return sections


def rolling_loop(stroke, y):
    if not 0 <= stroke <= STROKE:
        raise ValueError("Shelf stroke outside travel")
    # Half-speed bend center gives one fixed-length cable at every shelf position.
    turn = (FIXED_X + MOVING_X + stroke + math.pi*RADIUS - MANAGED_LENGTH) / 2
    bottom = np.array([turn, y, LOW_Z])
    center = bottom + [0, 0, RADIUS]
    top = bottom + [0, 0, 2 * RADIUS]
    return [line(np.array([FIXED_X, y, LOW_Z]), bottom),
            arc(center, bottom - center, np.array([-1, 0, 0]), math.pi),
            line(top, np.array([MOVING_X + stroke, y, HIGH_Z]))]


def power_feed(y):
    # The design reserves a vertical exit inside the converter's top face.
    return fillet_path([(-78,-163,277), (-78,-163,333), (55,-163,333),
                        (55,y,333), (55,y,LOW_Z), (FIXED_X,y,LOW_Z)], 36)


def data_feed(y):
    # Upward datum on the host envelope allows a separately qualified short plug lead.
    return fillet_path([(-245,-199,193), (-245,-199,355), (75,-199,355),
                        (75,y,355), (75,y,LOW_Z), (FIXED_X,y,LOW_Z)], 50)


def gap_to_boxes(sections, boxes, cable_radius, exclude=()):
    # The distance function is 1-Lipschitz. Within an arclength sample interval
    # the continuous path can be at most half its pitch closer than either end.
    points = np.concatenate([points for _, points in sections])
    results = []
    for key, box in boxes.items():
        if key in exclude:
            continue
        delta = np.maximum(0, np.maximum(box[:3] - points, points - box[3:]))
        gap = float(np.linalg.norm(delta, axis=1).min() - cable_radius - PITCH/2)
        results.append((gap, key))
    return sorted(results)
