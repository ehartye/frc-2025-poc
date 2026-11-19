"""Measure candidate axial contact from the public Rotor Clip C-50 installed preview.

Run with Python 3 and the vendor zip path. Input coordinates and output lengths
are millimetres. The local shaft axis passes through (0, 0) along Z. The ring
mouth points toward -Y. The washer is square-edged, parallel and concentric
unless a center offset is explicitly applied. Nothing is extracted to disk.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import struct
import zipfile

ARCHIVE_SHA = "7e3525f38246505036ce6fab04e6e960b037955e741a8fed259fdae2e86445de"
GEOMETRY_SHA = "e229adb89f90eadca53f73d7270767da8b77f6398db64d4a3ee5577a9f2e6ca4"
INNER = 6.95
OUTER = 8.35
ECCENTRICITY = 0.20


def read_faces(archive):
    if hashlib.sha256(archive.read_bytes()).hexdigest() != ARCHIVE_SHA:
        raise ValueError("The C-50 archive differs from the registered source.")
    with zipfile.ZipFile(archive) as source:
        metadata = json.loads(source.read("c-50.js"))
        binary = source.read("c-50_000.bin")
    if hashlib.sha256(binary).hexdigest() != GEOMETRY_SHA:
        raise ValueError("Unregistered Three.js mesh")
    if binary[:12] != b"Three.js 003" or metadata["scale"] != 1:
        raise ValueError("Unexpected viewer geometry format or scale")
    counts = struct.unpack_from("<12I", binary, 24)
    vertex_count, normal_count, line_count, triangle_count = (
        counts[0], counts[1], counts[2], counts[5]
    )
    if (vertex_count, normal_count, line_count, triangle_count) != (198, 226, 213, 392):
        raise ValueError("Unexpected topology")
    start = binary[12]
    vertices = list(struct.iter_unpack("<3f", binary[start:start + 12 * vertex_count]))
    # Vertex triples (float32), normal triples (signed bytes, 4-byte padding),
    # then line index-pairs (uint32). Smooth triangles follow, as vertex triples.
    start += 12 * vertex_count + 4 * math.ceil(3 * normal_count / 4) + 8 * line_count
    faces = list(struct.iter_unpack("<3I", binary[start:start + 12 * triangle_count]))
    return vertices, faces


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def disk_crossings(a, b, radius):
    dx, dy = b[0] - a[0], b[1] - a[1]
    quadratic = dx * dx + dy * dy
    if not quadratic:
        return []
    linear = 2 * (a[0] * dx + a[1] * dy)
    disc = linear * linear - 4 * quadratic * (dot(a, a) - radius * radius)
    if disc <= 0:
        return []
    roots = [(-linear + sign * math.sqrt(disc)) / (2 * quadratic) for sign in (-1, 1)]
    return [t for t in roots if 0 < t < 1]


def interpolate(a, b, t):
    return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))


def disk_edge_area(a, b, radius):
    # Green's theorem for a polygon clipped to a circle. A subedge inside the
    # circle contributes a signed triangle. Outside contributes a signed sector.
    cuts = [0.0, *disk_crossings(a, b, radius), 1.0]
    result = 0.0
    for lo, hi in zip(cuts, cuts[1:]):
        p, q = interpolate(a, b, lo), interpolate(a, b, hi)
        mid = interpolate(p, q, 0.5)
        if dot(mid, mid) <= radius * radius:
            result += cross(p, q) / 2
        else:
            result += radius * radius * math.atan2(cross(p, q), dot(p, q)) / 2
    return result


def disk_area(triangles, center, radius):
    terms = []
    for tri in triangles:
        points = [(p[0] - center[0], p[1] - center[1]) for p in tri]
        terms.extend(disk_edge_area(points[i - 1], points[i], radius) for i in range(3))
    return abs(math.fsum(terms))


def overlap(triangles, center, inner=INNER, outer=OUTER):
    return disk_area(triangles, center, outer) - disk_area(triangles, center, inner)


def section(vertices, faces, height):
    axial = [f for f in faces if all(vertices[i][2] == height for i in f)]
    triangles = [[vertices[i][:2] for i in f] for f in axial]
    edges = Counter(tuple(sorted((f[i - 1], f[i]))) for f in axial for i in range(3))
    boundary = [(vertices[i][:2], vertices[j][:2]) for (i, j), n in edges.items() if n == 1]
    return triangles, boundary


def mouth(boundary):
    angles = []
    for a, b in boundary:
        for t in disk_crossings(a, b, INNER):
            x, y = interpolate(a, b, t)
            angles.append(math.degrees(math.atan2(y, x)) % 360)
    angles.sort()
    if len(angles) != 2 or not angles[0] < 270 < angles[1]:
        raise ValueError("Unrecognized mouth section on the washer ID circle")
    return {"anglesFromPositiveXDeg": angles, "openArcDeg": angles[1] - angles[0],
            "description": "Open arc on washer ID circle, through -Y, in installed preview"}


def report(archive, force):
    vertices, faces = read_faces(archive)
    heights = sorted({p[2] for p in vertices})
    if len(heights) != 2:
        raise ValueError("Unexpected axial planes")
    maximum_radius = max(math.hypot(x, y) for x, y, _ in vertices)
    if maximum_radius + ECCENTRICITY >= OUTER:
        raise ValueError("Washer OD no longer encloses the displaced ring")
    result = {"sourceArchiveSha256": ARCHIVE_SHA, "meshSha256": GEOMETRY_SHA,
              "vertexCount": len(vertices), "triangleCount": len(faces),
              "radialOriginEvidence": "Model (0,0) to central inward vertex at (0,5.715) matches catalog groove radius .225 inch.",
              "axialFacesZMm": heights, "maxVertexRadiusMm": maximum_radius,
              "washerIdOdMm": [2 * INNER, 2 * OUTER],
              "maximumEccentricityMm": ECCENTRICITY,
              "registration": "Shaft +Z; washer axis parallel to Z. Mouth at -Y. Sector angles are counterclockwise from +X.",
              "pressureAssumption": "Projected square-face area at exact axial coincidence; average over all projected contact only. No local pressure or load capacity follows.",
              "faces": []}
    for height in heights:
        triangles, boundary = section(vertices, faces, height)
        nominal = overlap(triangles, (0, 0))
        # Every offset inner disk is inside the enlarged centered disk. The
        # contracted centered disk is inside every offset inner disk. The
        # washer OD encloses the entire projection in all these positions.
        low = overlap(triangles, (0, 0), INNER + ECCENTRICITY)
        high = overlap(triangles, (0, 0), INNER - ECCENTRICITY)
        controls = {}
        for angle in range(0, 360, 45):
            theta = math.radians(angle)
            center = (ECCENTRICITY * math.cos(theta), ECCENTRICITY * math.sin(theta))
            controls[str(angle)] = overlap(triangles, center)
        separated = overlap(triangles, (25, 0))
        if abs(separated) > 1e-7 or not all(low <= area <= high for area in controls.values()):
            raise ValueError("Independent disjoint/eccentric controls failed")
        result["faces"].append({
            "zMm": height, "planarTriangles": len(triangles),
            "nominalAreaMm2": nominal, "areaBoundsAllOffsetsMm2": [low, high],
            "areaAtMaxOffsetByDirectionDegMm2": controls,
            "disjointRadialControlCenterMm": [25, 0], "disjointAreaMm2": max(0, separated),
            "mouthAtWasherID": mouth(boundary), "assumedAxialForceN": force,
            "fullContactMeanPressureMPa": force / nominal,
            "fullContactMeanPressureCeilingAtAnyOffsetMPa": force / low,
        })
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--force-n", type=float, default=1.0,
                        help="assumed axial load for illustrative mean pressure (default 1 N)")
    args = parser.parse_args()
    if not math.isfinite(args.force_n) or args.force_n < 0:
        parser.error("--force-n must be a finite nonnegative axial force")
    print(json.dumps(report(args.archive, args.force_n), indent=2, allow_nan=False))
