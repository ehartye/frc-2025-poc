"""Render the original robot, the five reopened Fusion groups and an indicative cage."""

import argparse
import gzip
import io
import json
import math
import subprocess
import tempfile
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from installed import HERE, ROOT, load_parts
from kit_mesh import phase


def actor(poly, color):
    m = vtk.vtkPolyDataMapper()
    m.SetInputData(poly)
    a = vtk.vtkActor()
    a.SetMapper(m)
    a.GetProperty().SetColor(*[n / 255 for n in color])
    a.GetProperty().SetAmbient(0.36)
    return a


def base_actor():
    raw = gzip.decompress((ROOT / "docs/robot/cad/assembly.stl.gz").read_bytes())
    rows = np.frombuffer(
        raw, offset=84, dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]
    )
    groups = json.loads((ROOT / "cad/swerve-captive/motion-ranges.json").read_bytes())[
        "batches"
    ]
    palette = json.loads((ROOT / "cad/component-palette.json").read_bytes())["colors"]
    colors = []
    pts = []
    off = 0
    for role, group, count in groups:
        if group != "Demonstration CORAL":
            rgb = bytes.fromhex(palette[role][1:])
            colors.extend([list(rgb)] * count)
            pts.append(rows["v"][off : off + count])
        off += count
    xyz = np.ascontiguousarray(np.concatenate(pts).reshape(-1, 3))
    p = vtk.vtkPoints()
    p.SetData(numpy_to_vtk(xyz, deep=True))
    n = len(xyz)
    c = vtk.vtkCellArray()
    c.SetData(
        numpy_to_vtkIdTypeArray(np.arange(0, n + 1, 3, dtype=np.int64), deep=True),
        numpy_to_vtkIdTypeArray(np.arange(n, dtype=np.int64), deep=True),
    )
    poly = vtk.vtkPolyData()
    poly.SetPoints(p)
    poly.SetPolys(c)
    poly.GetCellData().SetScalars(
        numpy_to_vtk(np.asarray(colors, dtype=np.uint8), deep=True)
    )
    return actor(poly, (255, 255, 255))


def stl(shape, tmp):
    mesh = BRepMesh_IncrementalMesh(shape, 0.18, False, 0.18, True)
    mesh.Perform()
    w = StlAPI_Writer()
    w.ASCIIMode = False
    w.Write(shape, str(tmp))
    r = vtk.vtkSTLReader()
    r.SetFileName(str(tmp))
    r.Update()
    out = vtk.vtkPolyData()
    out.DeepCopy(r.GetOutput())
    return out


def run(ffmpeg, out, still):
    fps, w, h = 8, 920, 560
    rend = vtk.vtkRenderer()
    rend.SetBackground(0.078, 0.184, 0.388)
    rend.SetBackground2(0.24, 0.36, 0.49)
    rend.GradientBackgroundOn()
    win = vtk.vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.SetSize(w, h)
    win.AddRenderer(rend)
    main = base_actor()
    rend.AddActor(main)
    colors = {
        "Shallow cage jack frame": "77899A",
        "Cage crosshead": "D47D3D",
        "Left cage shoe": "647181",
        "Right cage shoe": "647181",
        "Lower plate clamp": "007F78",
    }
    groups = {}
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp) / "one.stl"
        for _, (group, part) in load_parts(HERE)[1].items():
            ac = actor(stl(part, tmp), bytes.fromhex(colors[group]))
            rend.AddActor(ac)
            groups.setdefault(group, []).append(ac)
    cage = []
    for x in (-76.2, 76.2):
        for y in (-76.2, 76.2):
            c = vtk.vtkCylinderSource()
            c.SetRadius(16.7)
            c.SetHeight(609.6)
            c.SetResolution(32)
            c.Update()
            ac = actor(c.GetOutput(), (222, 198, 145))
            ac.RotateX(90)
            ac.SetPosition(300 + x, y, 765.175 + 304.8)
            cage.append(ac)
            rend.AddActor(ac)
    # Project-drawn annulus exposes the measured bore through the plate.
    for z in (785.7, 1374.8):
        c = vtk.vtkDiskSource()
        c.SetInnerRadius(38.1)
        c.SetOuterRadius(94)
        c.SetCircumferentialResolution(96)
        c.Update()
        ac = actor(c.GetOutput(), (226, 200, 147))
        ac.SetPosition(300, 0, z)
        cage.append(ac)
        rend.AddActor(ac)
    c = vtk.vtkCubeSource()
    c.SetXLength(2400)
    c.SetYLength(1600)
    c.SetZLength(3)
    c.SetCenter(0, 0, -4)
    c.Update()
    rend.AddActor(actor(c.GetOutput(), (80, 102, 121)))
    cam = rend.GetActiveCamera()
    cam.ParallelProjectionOn()
    cam.SetFocalPoint(-20, 0, 710)
    cam.SetPosition(1530, -2470, 1550)
    cam.SetViewUp(0, 0, 1)
    cam.SetParallelScale(870)
    grab = vtk.vtkWindowToImageFilter()
    grab.SetInput(win)
    grab.ReadFrontBufferOff()

    def font(size):
        for name in (
            "C:/Windows/Fonts/segoeui.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        ):
            if Path(name).exists():
                return ImageFont.truetype(name, size)
        return ImageFont.load_default(size=size)

    title, small = font(26), font(18)
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
        str(fps),
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libvpx",
        "-threads",
        "2",
        "-b:v",
        "1300k",
        str(out),
    ]
    with subprocess.Popen(cmd, stdin=subprocess.PIPE) as proc:
        for i in range(26 * fps + 1):
            t = 130 + i / fps

            v = phase(t)
            approach, top, spread, seat, locked, q, lift, tilt = (
                v[k]
                for k in (
                    "approach",
                    "top",
                    "spread",
                    "seat",
                    "locked",
                    "crosshead",
                    "lift",
                    "tilt",
                )
            )

            def tf(x=0, y=0, z=0):
                v = vtk.vtkTransform()
                v.Translate(300, 0, 1508.064)
                v.RotateY(-tilt)
                v.Translate(-300, 0, -1508.064)
                v.Translate(x, y, z)
                return v

            main.SetUserTransform(tf(approach, 0, lift))
            for ac in cage:
                ac.SetUserTransform(tf())
            for group, actors in groups.items():
                dy = (
                    spread
                    if group.startswith("Left")
                    else -spread
                    if group.startswith("Right")
                    else 0
                )
                dz = (0 if group.startswith("Shallow") else q) + (
                    v["clamp"] if group.startswith("Lower") else 0
                )
                for ac in actors:
                    ac.SetUserTransform(tf(approach, dy, lift + dz))
            rend.ResetCameraClippingRange()
            win.Render()
            grab.Modified()
            grab.Update()
            pixels = vtk_to_numpy(grab.GetOutput().GetPointData().GetScalars()).reshape(
                h, w, 3
            )[::-1]
            im = Image.fromarray(pixels.copy())
            d = ImageDraw.Draw(im)
            status = (
                "Approach own shallow cage"
                if t < 132.6
                else "Register empty rack and bore"
                if t < 133.1
                else "Raise narrow head"
                if t < top
                else "Spread bearing shoes"
                if t < seat
                else "Close lower clamp"
                if t < locked
                else "Raise chassis"
                if t < 148.182
                else "Hold with passive locks"
            )
            d.rounded_rectangle((14, 12, 445, 80), 8, fill="#112747")
            d.text((26, 14), status, font=title, fill="white")
            d.text(
                (26, 50),
                f"Wall time {t:05.1f} s     •     Lift {lift:.0f} mm",
                font=small,
                fill="#bdd8ee",
            )
            d.rectangle((0, h - 39, w, h), fill="#112747")
            d.text(
                (18, h - 33),
                "Conditional jack motion   |   Field strength and inspection mass unresolved   |   Match credit 0",
                font=small,
                fill="white",
            )
            if i == 19 * fps:
                im.save(still)
            buf = io.BytesIO()
            im.save(buf, format="JPEG", quality=86)
            proc.stdin.write(buf.getvalue())
        proc.stdin.close()
        assert proc.wait() == 0
    win.Finalize()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("ffmpeg", type=Path)
    args = p.parse_args()
    run(args.ffmpeg, HERE / "shallow-climb.webm", HERE / "shallow-climb.png")
