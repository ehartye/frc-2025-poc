"""Render the attained R4 fallback trajectory into a student field replay."""
import argparse
from io import BytesIO
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from matplotlib import font_manager

ROOT = Path(__file__).resolve().parents[1]
RECEIPT = ROOT / "experiments/swerve-drive/motion/fallback-match.json"
W, H, FPS = 1200, 660, 12
BG, PANEL, TEXT, GOLD = "#11283e", "#1b3953", "#faf5e7", "#ffcf6b"
MUTED = "#c2d6df"


def font(size, bold=False):
    props = font_manager.FontProperties(family="DejaVu Sans",weight="bold" if bold else "normal")
    return ImageFont.truetype(font_manager.findfont(props),size)


def overhead_robot():
    """Project the same colored R4 mesh as the orbit viewer at one pixel per mm."""
    import gzip
    import hashlib
    from tempfile import TemporaryDirectory
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk, vtk_to_numpy

    cad = ROOT/"docs/robot/cad"
    mesh = (cad/"assembly.stl.gz").read_bytes()
    receipt = json.loads((cad/"receipt.json").read_bytes())
    if hashlib.sha256(mesh).hexdigest() != receipt["meshSha256"]:
        raise ValueError("R4 viewer mesh differs from its release receipt")
    groups = receipt["appearances"]["groups"]
    with TemporaryDirectory() as folder:
        stl=Path(folder)/"r4.stl"
        stl.write_bytes(gzip.decompress(mesh))
        reader=vtk.vtkSTLReader()
        reader.SetFileName(str(stl))
        reader.MergingOff()
        reader.Update()
        poly=reader.GetOutput()
        if poly.GetNumberOfCells()!=receipt["triangles"]:
            raise ValueError("Incomplete R4 triangle set")
        colors=np.array([list(bytes.fromhex(g["color"][1:])) for g in groups],dtype=np.uint8)
        rgb=np.repeat(colors,[g["triangles"] for g in groups],axis=0)
        poly.GetCellData().SetScalars(numpy_to_vtk(rgb,deep=True))
        mapper=vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)
        mapper.SetScalarModeToUseCellData()
        mapper.SetColorModeToDirectScalars()
        actor=vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetAmbient(.35)
        scene=vtk.vtkRenderer()
        scene.SetBackgroundAlpha(0)
        scene.AddActor(actor)
        camera=scene.GetActiveCamera()
        camera.ParallelProjectionOn()
        camera.SetFocalPoint(0,0,0)
        camera.SetPosition(0,0,2500)
        camera.SetViewUp(0,1,0)
        camera.SetParallelScale(450)
        scene.ResetCameraClippingRange()
        window=vtk.vtkRenderWindow()
        window.SetOffScreenRendering(1)
        window.SetAlphaBitPlanes(1)
        window.SetMultiSamples(0)
        window.AddRenderer(scene)
        window.SetSize(900,900)
        window.Render()
        shot=vtk.vtkWindowToImageFilter()
        shot.SetInput(window)
        shot.SetInputBufferTypeToRGBA()
        shot.ReadFrontBufferOff()
        shot.Update()
        pixels=vtk_to_numpy(shot.GetOutput().GetPointData().GetScalars()).reshape(900,900,4)
        sprite=Image.fromarray(np.flipud(pixels).copy())
        window.Finalize()
        return sprite.resize((342,342),Image.Resampling.LANCZOS)


def render(encoder, output, frame_preview):
    data=json.loads(RECEIPT.read_bytes())
    run=next(c for c in data["cases"] if c["case"]==1)
    verdict=next(c for c in data["trials"] if c["case"]==1)
    rows=np.array(run["displaySecondsXYHeading"])
    if not verdict["provisionalPoints"] or rows[0,0]!=0 or rows[-1,0]!=156:
        raise ValueError("Complete reviewed fallback required")
    fonts={k:font(s,b) for k,s,b in [("title",37,True),("label",18,True),("copy",20,False),("value",31,True),("minor",15,False)]}
    def point(x,y):
        return (62+(x-8.65)*380, 358-(y-2.4272875)*380)
    robot=overhead_robot()
    base=Image.new("RGB",(W,H),BG)
    canvas=ImageDraw.Draw(base)
    canvas.text((42,27),"Leave the line, then park",font=fonts["title"],fill=TEXT)
    canvas.text((44,84),"Stowed swerve fallback  /  Red 2 in the reserved south lane  /  65 kg",font=fonts["copy"],fill=MUTED)
    left,right,top,bottom=42,1158,145,548
    canvas.rectangle((left,top,right,bottom),fill=PANEL)
    zone_x=point(368.4375*.0254,0)[0]
    canvas.rectangle((left,top,zone_x,bottom),fill="#5e383f")
    tape1,tape2=[point(x*.0254,0)[0] for x in (690.875-144-65.5-88-2,690.875-144-65.5-88)]
    canvas.rectangle((tape1,top,tape2,bottom),fill="#d3dee5")
    canvas.text((left+12,top+11),"RED BARGE ZONE",font=fonts["label"],fill=TEXT)
    canvas.text((tape2+9,top+11),"Starting line",font=fonts["label"],fill=TEXT)
    for x in (9,9.5,10,10.5,11,11.5):
        px,_=point(x,0)
        canvas.line((px,bottom-8,px,bottom),fill=MUTED,width=2)
        canvas.text((px,bottom-12),f"{x:g} m",anchor="ms",font=fonts["minor"],fill=MUTED)
    for x in (9.6,10.60):
        px,_=point(x,0)
        canvas.line((px-6,358,px+6,358),fill=GOLD,width=2)
        canvas.line((px,352,px,364),fill=GOLD,width=2)
    def image_at(t,scale):
        image=base.copy()
        d=ImageDraw.Draw(image)
        x,y,h=[np.interp(t,rows[:,0],col) for col in rows[:,1:].T]
        sprite=robot.rotate(math.degrees(h),resample=Image.Resampling.BICUBIC)
        px,py=point(x,y)
        image.paste(sprite,(round(px-sprite.width/2),round(py-sprite.height/2)),sprite)
        state = "AUTO" if t<15 else "DISABLED" if t<18 else "TELEOP" if t<153 else "ASSESSMENT"
        action = "Clear the tape" if t<run["events"][1][0] else "Hold clear of the line" if t<15 else "Drive outputs disabled" if t<18 else "Return to the barge zone" if t<run["events"][2][0] else "Hold the parked pose"
        d.text((43,573),f"{t:06.2f} s",font=fonts["value"],fill=GOLD)
        d.text((248,574),state,font=fonts["label"],fill=MUTED)
        d.text((248,606),action,font=fonts["copy"],fill=TEXT)
        d.text((1157,578),f"Field X   {x:.3f} m",anchor="ra",font=fonts["label"],fill=TEXT)
        d.text((1157,611),f"Playback   {scale:g}×",anchor="ra",font=fonts["minor"],fill=MUTED)
        return image
    if frame_preview:
        image_at(155.,1).save(frame_preview)
    with subprocess.Popen([str(encoder),"-hide_banner","-loglevel","error","-n","-f","image2pipe","-r",str(FPS),
            "-c:v","mjpeg","-i","pipe:0","-an","-c:v","libvpx","-b:v","1300k","-pix_fmt","yuv420p",str(output)],stdin=subprocess.PIPE) as proc:
        try:
            for start,end,scale in ((0,4,1),(4,15,5),(15,18,1),(18,23,1),(23,153,25),(153,156,1)):
                for t in np.linspace(start,end,math.ceil((end-start)*FPS/scale),endpoint=False):
                    encoded=BytesIO()
                    image_at(t,scale).save(encoded,format="JPEG",quality=91)
                    proc.stdin.write(encoded.getvalue())
            encoded=BytesIO()
            image_at(156.,1).save(encoded,format="JPEG",quality=91)
            proc.stdin.write(encoded.getvalue())
        finally:
            proc.stdin.close()
        if proc.wait():
            raise RuntimeError("Film encoder failed")


if __name__=="__main__":
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("ffmpeg",type=Path)
    ap.add_argument("output",type=Path)
    ap.add_argument("--preview",type=Path)
    args=ap.parse_args()
    render(args.ffmpeg,args.output,args.preview)
