"""Draw equal-scale sections from a registered pipe frame and two installed receiver solids."""

import argparse
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Section
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Pln, gp_Pnt, gp_Dir
from installed_contact import local, shapes, cylinder, read, sha


def font(size):
    for name in ("segoeui.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default(size)


def section(draw, solid, x, color, width, project):
    cut = BRepAlgoAPI_Section(solid, gp_Pln(gp_Pnt(x, 0, 0), gp_Dir(1, 0, 0)), False)
    cut.Build()
    if not cut.IsDone():
        raise ValueError("Failed section")
    edges = TopExp_Explorer(cut.Shape(), TopAbs_EDGE)
    while edges.More():
        cast = getattr(TopoDS, "Edge_s", None) or TopoDS.Edge
        curve = BRepAdaptor_Curve(cast(edges.Current()))
        vals = np.linspace(curve.FirstParameter(), curve.LastParameter(), 45)
        points = [
            project(curve.Value(float(t)).Y(), curve.Value(float(t)).Z()) for t in vals
        ]
        draw.line(points, fill=color, width=width)
        edges.Next()


def run(trace, output):
    check = read(trace.with_suffix(".installed.json"))
    if sha(trace) != check["trace"]:
        raise ValueError("Different signed exit state")
    raw = np.load(trace)
    q0 = np.load(trace.parent / "rack-337mm-31.25us.npz")["q"][0, :6]
    items = {k: s for k, s, _ in shapes()["Outer slide"]}
    foot = items["grip-R-front-support-foot"]
    riser = items["grip-R-front-riser"]
    im = Image.new("RGB", (1160, 642), "#f5f6f4")
    d = ImageDraw.Draw(im)
    ink = "#172c3d"
    hardware = "#496a81"
    pipe_color = "#975800"
    collision = "#c33235"
    d.text((36, 17), "Where the pipe catches", font=font(38), fill=ink)
    d.text(
        (38, 73),
        "Right front corner of the receiver  ·  transverse CAD sections  ·  positive wheel calibration",
        font=font(20),
        fill=ink,
    )
    for x0, t, title in [
        (40, 3.42, "First measured foot interference"),
        (620, 3.82, "Largest measured riser interference"),
    ]:
        row = raw[np.argmin(abs(raw[:, 0] - t))]
        p, a = local(row, q0, "Outer slide")
        shell = cylinder(p, a)
        focus = foot if t < 3.5 else riser
        bounds = Bnd_Box()
        BRepBndLib.AddOptimal_s(
            BRepAlgoAPI_Common(shell, focus).Shape(), bounds, False, False
        )
        station = (bounds.CornerMin().X() + bounds.CornerMax().X()) / 2
        d.text((x0, 125), title, font=font(23), fill=ink)
        d.text(
            (x0, 163),
            f"{row[0]:.3f} s after stopping  ·  X = {station:.1f} mm",
            font=font(19),
            fill=ink,
        )
        x1, y1 = x0 + 6, 515

        def project(y, z):
            return (round(x1 + (y + 200) * 3.3), round(y1 - (z - 485) * 3.3))

        # Clip crop to the local physical section, preserving equal axis scale.
        panel = Image.new("RGB", im.size, "#ffffff")
        pd = ImageDraw.Draw(panel)
        for y in range(-200, -59, 20):
            for z in (490, 510, 530, 550, 570):
                x, y0 = project(y, z)
                pd.line((x - 2, y0, x + 2, y0), fill="#cbd6d8", width=1)
        section(pd, foot, station, hardware, 3, project)
        section(pd, riser, station, hardware, 3, project)
        section(pd, shell, station, pipe_color, 4, project)
        for solid in (foot, riser):
            overlap = BRepAlgoAPI_Common(shell, solid).Shape()
            section(pd, overlap, station, collision, 5, project)
        crop = (x0, 210, x0 + 500, 525)
        im.paste(panel.crop(crop), crop[:2])
        d.line((x0 + 330, 503, x0 + 396, 503), fill=ink, width=3)
        d.text((x0 + 328, 473), "20 mm", font=font(16), fill=ink)
    for x, label, color in [
        (40, "Receiver foot and riser", hardware),
        (354, "Hollow CORAL", pipe_color),
        (626, "Shared material at this slice", collision),
    ]:
        d.line((x, 557, x + 28, 557), fill=color, width=5)
        d.text((x + 39, 540), label, font=font(18), fill=ink)
    d.text(
        (40, 594),
        "A pipe cannot follow the computed path through these parts. Both scoring trials remain uncredited.",
        font=font(20),
        fill=ink,
    )
    im.save(output)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trace", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    run(a.trace, a.output)
