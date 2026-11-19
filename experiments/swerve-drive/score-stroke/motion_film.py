"""Render the project-owned R4 CAD and conditional mechanism traces as a silent student film."""

import argparse
import subprocess
import sys
from pathlib import Path
from collections import defaultdict

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy

from motion_track import HERE, PROJECT, read, verify

W, H, FPS = 960, 540, 20
SIX = PROJECT / "experiments/six-wheel"
sys.path.insert(0, str(HERE.parent / "motion"))
import receiver_witness as camera


def mesh(vertices, faces, colors):
    pts = vtk.vtkPoints()
    pts.SetData(
        numpy_to_vtk(np.ascontiguousarray(vertices, dtype=np.float32), deep=True)
    )
    cells = vtk.vtkCellArray()
    triangles = np.ascontiguousarray(faces, dtype=np.int64).reshape(-1)
    cells.SetData(
        numpy_to_vtkIdTypeArray(
            np.arange(0, len(triangles) + 1, 3, dtype=np.int64), deep=True
        ),
        numpy_to_vtkIdTypeArray(triangles, deep=True),
    )
    poly = vtk.vtkPolyData()
    poly.SetPoints(pts)
    poly.SetPolys(cells)
    poly.GetCellData().SetScalars(
        numpy_to_vtk(np.ascontiguousarray(colors, dtype=np.uint8), deep=True)
    )
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetAmbient(0.45)
    actor.GetProperty().SetDiffuse(0.55)
    return actor


def tube():
    v = []
    faces = []
    n = 72
    for y, r in (
        (-150.8125, 57.15),
        (150.8125, 57.15),
        (-150.8125, 50.8),
        (150.8125, 50.8),
    ):
        for a in np.arange(n) * 2 * np.pi / n:
            v.append([r * np.cos(a), y, r * np.sin(a)])
    for a, b in ((0, 1), (1, 3), (3, 2), (2, 0)):
        for i in range(n):
            j = (i + 1) % n
            faces.extend(
                [[a * n + i, b * n + i, a * n + j], [a * n + j, b * n + i, b * n + j]]
            )
    return mesh(v, faces, np.tile([255, 247, 212], (len(faces), 1)))


def receiver():
    model = PROJECT / "cad/full-swerve-r4"
    manifest = read(model / "appearances.json")
    verify(model / "full-robot.step", manifest["sourceSha256"])
    rows = manifest["solids"]
    shapes = camera.sensor.mount.cad.read_named(model / "full-robot.step", rows)
    changed = HERE / "cam-lands.step"
    verify(changed, read(HERE / "cam.json")["candidateStepSha256"])
    layers = ("r6-polymer-facing", "r6-receiver-insert")
    shapes.update(
        camera.sensor.mount.cad.read_named(changed, [{"id": n} for n in layers])
    )
    parts = read(SIX / "design/competition-package/shop-duplex/red/part-frames.json")
    colors = {
        r["role"]: r["color"]
        for r in read(PROJECT / "docs/robot/cad/receipt.json")["appearances"]["groups"]
    }
    stacks = defaultdict(list)
    for row in rows:
        name = row["id"]
        explicit = (
            name in layers
            or name in parts
            and (
                parts[name] in ("middle", "slide")
                or name in ("slide-L-outer", "slide-R-outer", "slide-base-bridge")
            )
        )
        group = parts.get(name, "base") if explicit else "context"
        if name in layers:
            group = "slide"
        v, f = camera.sensor.mount.original.triangulate(shapes[name])
        vs = np.asarray(v, dtype=np.float32).reshape(-1, 3) / 10
        fs = np.asarray(f, dtype=np.int64).reshape(-1, 3)
        rgb = list(bytes.fromhex(colors[row["role"]][1:]))
        stacks[group].append((vs, fs, np.tile(rgb, (len(fs), 1))))
    output = {}
    for group, items in stacks.items():
        offset = 0
        idx = []
        for vertices, indices, _ in items:
            idx.append(indices + offset)
            offset += len(vertices)
        output[group] = mesh(
            np.vstack([v for v, _, _ in items]),
            np.vstack(idx),
            np.vstack([col for _, _, col in items]),
        )
    return output


def goal(track):
    target = read(SIX / "mission/target.json")
    v = []
    tri = []
    base = np.array(track["dockOriginM"])
    yaw = track["dockYawRad"]
    c, s = np.cos(yaw), np.sin(yaw)
    transform = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    for panel in target["panels"]:
        quad = np.array(
            [
                [x, y, target["floor_intercept_m"] + target["floor_slope"] * x]
                for x, y in panel["bounds_xy"]
            ]
        )
        i = len(v)
        v.extend((quad - base) @ transform * 1000)
        tri.extend([[i, i + j, i + j + 1] for j in range(1, len(quad) - 1)])
    # Draw the collision model's finite front lip and risers. The target
    # surfaces are educational proxies derived from the project register.
    bounds = np.vstack([panel["bounds_xy"] for panel in target["panels"]])
    boxes = [
        [
            0,
            bounds[:, 1].min(),
            target["floor_intercept_m"],
            target["lip_depth_m"],
            bounds[:, 1].max(),
            target["lip_top_m"],
        ],
        *target["riser_bounds_m"],
    ]
    for x0, y0, z0, x1, y1, z1 in boxes:
        box = np.array(
            [[x, y, z] for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
        )
        i = len(v)
        v.extend((box - base) @ transform * 1000)
        for a, b, c, d in (
            (0, 2, 3, 1),
            (4, 5, 7, 6),
            (0, 1, 5, 4),
            (2, 6, 7, 3),
            (0, 4, 6, 2),
            (1, 3, 7, 5),
        ):
            tri.extend([[i + a, i + b, i + c], [i + a, i + c, i + d]])
    return mesh(v, tri, np.tile([199, 103, 99], (len(tri), 1)))


def font(size):
    for name in ("segoeui.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default(size=size)


def run(ffmpeg, out, poster):
    track = read(HERE / "motion-track.json")
    stages = {k: np.array(track[k]) for k in ("station", "extend", "release")}
    parts = receiver()
    pipe = tube()
    target = goal(track)
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.025, 0.065, 0.15)
    renderer.SetBackground2(0.23, 0.35, 0.48)
    renderer.GradientBackgroundOn()
    for actor in [*parts.values(), pipe, target]:
        renderer.AddActor(actor)
    cam = renderer.GetActiveCamera()
    cam.ParallelProjectionOn()
    win = vtk.vtkRenderWindow()
    win.SetShowWindow(False)
    win.SetOffScreenRendering(1)
    win.SetMultiSamples(0)
    win.AddRenderer(renderer)
    win.SetSize(W, H)
    grab = vtk.vtkWindowToImageFilter()
    grab.SetInput(win)
    grab.ReadFrontBufferOff()
    cmd = [
        str(ffmpeg),
        "-y",
        "-loglevel",
        "error",
        "-f",
        "image2pipe",
        "-c:v",
        "mjpeg",
        "-framerate",
        str(FPS),
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libvpx",
        "-threads",
        "2",
        "-b:v",
        "1450k",
        str(out),
    ]
    labels = [
        (
            "station",
            "01   Station pickup",
            "First acquired pipe  /  0.1 mm absorber gap",
            2.0,
        ),
        (
            "extend",
            "02   Extend at the reef",
            "Separate dock fixture  /  54 mm extra setback",
            1.0,
        ),
        (
            "release",
            "03   Release into the trough",
            "Separate launch state  /  assumed belt exit speed",
            1.0,
        ),
    ]
    small, title = font(17), font(28)
    try:
        with subprocess.Popen(cmd, stdin=subprocess.PIPE) as sink:
            try:
                for stage, heading, sub, rate in labels:
                    data = stages[stage]
                    station = stage == "station"
                    parts["context"].SetVisibility(station)
                    target.SetVisibility(not station)
                    if station:
                        cam.SetFocalPoint(38, 0, 510)
                        cam.SetPosition(1380, -1750, 1240)
                        cam.SetParallelScale(680)
                    else:
                        cam.SetFocalPoint(275, -10, 470)
                        cam.SetPosition(990, -1820, 1110)
                        cam.SetParallelScale(405)
                    cam.SetViewUp(0, 0, 1)
                    renderer.ResetCameraClippingRange()
                    duration = (data[-1, 0] - data[0, 0]) / rate
                    for frame in range(round((duration + 0.65) * FPS)):
                        t = min(data[-1, 0], data[0, 0] + rate * frame / FPS)
                        stroke, *state = [
                            np.interp(t, data[:, 0], d) for d in data[:, 1:].T
                        ]
                        for group, actor in parts.items():
                            actor.SetPosition(
                                stroke * {"slide": 1, "middle": 0.5}.get(group, 0), 0, 0
                            )
                        axis = np.asarray(state[3:])
                        axis /= np.linalg.norm(axis)
                        rotation = np.cross([0, 1, 0], axis)
                        transform = vtk.vtkTransform()
                        transform.Translate(*state[:3])
                        if np.linalg.norm(rotation) > 1e-9:
                            transform.RotateWXYZ(
                                np.degrees(np.arccos(np.clip(axis[1], -1, 1))),
                                *rotation,
                            )
                        pipe.SetUserTransform(transform)
                        win.Render()
                        grab.Modified()
                        grab.Update()
                        rgb = vtk_to_numpy(
                            grab.GetOutput().GetPointData().GetScalars()
                        ).reshape(H, W, 3)[::-1]
                        image = Image.fromarray(rgb.copy())
                        draw = ImageDraw.Draw(image)
                        draw.rounded_rectangle(
                            (16, 14, 420, 84), radius=9, fill=(13, 32, 55)
                        )
                        draw.text((31, 18), heading, font=title, fill="white")
                        draw.text((31, 55), sub, font=small, fill="#bdd3e8")
                        if station:
                            status = (
                                "Captured in seat"
                                if t >= track["firstInBandS"]
                                else "Receiving"
                            )
                        elif stage == "extend":
                            status = f"Rack travel  {stroke:.0f} mm"
                        else:
                            status = (
                                "Paired trough support"
                                if t >= 3 - track["landing"]["final_dwell_s"]
                                else "Tracking released pipe"
                            )
                        draw.rectangle((0, H - 40, W, H), fill=(12, 29, 52))
                        draw.text((22, H - 32), status, font=small, fill="#ffdc85")
                        draw.text(
                            (W - 305, H - 32),
                            f"Fixture time  {t:4.2f} s     /     {rate:g}x playback",
                            font=small,
                            fill="white",
                        )
                        if stage == "extend" and frame == round(2.6 * FPS):
                            image.save(poster)
                        image.save(sink.stdin, format="JPEG", quality=95, subsampling=0)
            finally:
                sink.stdin.close()
            if sink.wait():
                raise RuntimeError("Video encoder failed")
    finally:
        win.Finalize()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for k in ("ffmpeg", "output", "poster"):
        p.add_argument(k, type=Path)
    a = p.parse_args()
    run(a.ffmpeg, a.output, a.poster)
