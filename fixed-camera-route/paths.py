"""Dimensioned static cable paths in robot millimetres; circular bends are tangent."""

import math
import numpy as np
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "moving-harness"))
from routing import unit

PITCH = 1.0
F = np.array([math.cos(math.radians(26)), 0, -math.sin(math.radians(26))])
U = np.array([-F[2], 0, F[0]])
L = np.array([0.0, 1.0, 0.0])
Z = np.array([0.0, 0.0, 1.0])
O = np.array([-315.0, 0.0, 970.0])
DX = 48.539 - 19.756857241338032
DATA = O - 1.5 * F + 23.5505 * L + DX * U
POWER = DATA - 16 * L
SIDE = -F
START_DATA = DATA + 7.0 * U + 28.5 * SIDE
START_POWER = POWER + 24.95 * U


def line(a, b):
    a, b = np.array(a, dtype=float), np.array(b, dtype=float)
    d = np.linalg.norm(b - a)
    if d < 1e-7:
        return []
    return [(float(d), np.linspace(a, b, math.ceil(d / PITCH) + 1), None)]


def arc(a, d1, d2, r):
    a, d1, d2 = np.array(a), unit(np.array(d1)), unit(np.array(d2))
    co = np.clip(d1 @ d2, -1, 1)
    n = unit(d2 - co * d1)
    angle = math.acos(co)
    th = np.linspace(0, angle, math.ceil(angle * r / PITCH) + 1)[:, None]
    return [(angle * r, a + r * (np.sin(th) * d1 + (1 - np.cos(th)) * n), r)]


def fillet(points, radii):
    pts = np.asarray(points, dtype=float)
    out = []
    a = pts[0]
    for p, q, r, rad in zip(pts[:-2], pts[1:-1], pts[2:], radii, strict=True):
        d1 = unit(q - p)
        d2 = unit(r - q)
        if np.linalg.norm(d1 - d2) < 1e-6:
            continue
        retreat = rad * math.tan(math.acos(np.clip(d1 @ d2, -1, 1)) / 2)
        before = q - d1 * retreat
        if (before - a) @ d1 < -1e-6:
            raise ValueError(f"Insufficient tangent at {q}")
        out += line(a, before) + arc(before, d1, d2, rad)
        a = out[-1][1][-1]
    if (pts[-1] - a) @ unit(pts[-1] - pts[-2]) < -1e-6:
        raise ValueError("Final tangent crosses the endpoint")
    return out + line(a, pts[-1])


def add_arc(parts, d1, d2, r):
    return parts + arc(parts[-1][1][-1], d1, d2, r)


YAW = math.radians(-15)
T = np.array([math.cos(YAW), math.sin(YAW), 0])
B = np.array([-T[1], T[0], 0])
PI = np.array([-249.0, -200.0, 0.0])
MOUTH = PI + 45.5 * T - B


def host(x, y, z):
    return MOUTH + (x + 202) * T + (y + 200) * B + z * Z


def routes():
    # Upper pigtail makes two tangent turns after the intact factory overmold.
    thin = line(START_POWER, START_POWER + 3 * U)
    lateral = -L
    thin = add_arc(thin, U, lateral, 21)
    p = thin[-1][1][-1]
    thin += line(p, p + 9 * lateral)
    thin = add_arc(thin, lateral, -Z, 21)
    p = thin[-1][1][-1]
    thin += line(p, p - 5 * Z)
    board = thin[-1][1][-1]
    # Terminals on opposite ends of a vertical, insulated transition board.
    bulk = board - [0, 8, 14]
    power = fillet(
        [
            bulk,
            bulk - [0, 77, 0],
            [-338, -107, 902],
            [-338, -29, 840],
            [-338, -29, 310],
            [-346, -74, 260],
            [-276, -130, 321],
            [-122, -163, 339],
            [-122, -163, 277],
        ],
        [36] * 7,
    )
    first = START_DATA + 5 * SIDE
    lead = line(START_DATA, first) + arc(first, SIDE, -Z, 21)
    high = lead[-1][1][-1]
    fixed = lead + fillet(
        [
            high,
            [high[0], high[1], 445],
            [-325, -205, 242],
            host(-145, -247, 215),
            host(-139, -247, 169),
            host(-139, -200, 169),
            host(-167, -200, 169),
        ],
        [21] * 5,
    )
    roof_start = host(-164, -200, 177)
    guide = np.array([-5.0, -317.0, 600.0])
    roof = fillet(
        [
            roof_start,
            host(4, -200, 177),
            host(4, -200, 401),
            [147, -317, 435],
            [147, -317, 600],
            guide,
        ],
        [78] * 4,
    )
    return {"data": fixed, "pigtail": thin, "power": power, "roof-host": roof}, board


def points(path):
    return np.vstack([row[1] for row in path])


def lengths(path):
    return float(sum(row[0] for row in path))
