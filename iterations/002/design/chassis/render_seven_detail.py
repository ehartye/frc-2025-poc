"""Render the two real repaired brace screw axes from the checked shop STEP."""
import hashlib,io,json,re,sys
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE,TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
import vtk
from vtk.util.numpy_support import vtk_to_numpy,numpy_to_vtk,numpy_to_vtkIdTypeArray

HERE=Path(__file__).resolve().parent
def mesh(shape):
    BRepMesh_IncrementalMesh(shape,.045,False,.14,True).Perform()
    pts=[];cells=[];it=TopExp_Explorer(shape,TopAbs_FACE)
    while it.More():
        face=TopoDS.Face(it.Current());it.Next();loc=TopLoc_Location()
        m=BRep_Tool.Triangulation_s(face,loc)
        if m is None:continue
        base=len(pts);trans=loc.Transformation()
        pts.extend(m.Node(i).Transformed(trans).Coord() for i in range(1,m.NbNodes()+1))
        for i in range(1,m.NbTriangles()+1):
            a,b,c=m.Triangle(i).Get()
            if face.Orientation()==TopAbs_REVERSED:b,c=c,b
            cells.extend((3,base+a-1,base+b-1,base+c-1))
    points=vtk.vtkPoints();points.SetData(numpy_to_vtk(np.asarray(pts,dtype=float),deep=True))
    faces=vtk.vtkCellArray();faces.SetCells(len(cells)//4,numpy_to_vtkIdTypeArray(np.asarray(cells,dtype=np.int64),deep=True))
    geometry=vtk.vtkPolyData();geometry.SetPoints(points);geometry.SetPolys(faces)
    crop=vtk.vtkBox();crop.SetBounds(208.,293.,105.,183.,156.,243.)
    clip=vtk.vtkClipPolyData();clip.SetInputData(geometry);clip.SetClipFunction(crop);clip.InsideOutOn()
    # Intersect six planes separately so a large plane with all outer vertices
    # outside the box can still cross the inspection region.
    last=geometry
    for axis,lo,hi in [(0,208.,293.),(1,105.,183.),(2,156.,243.)]:
        for coord,sgn in [(lo,1.),(hi,-1.)]:
            t=vtk.vtkPlane();o=[0.,0.,0.];v=[0.,0.,0.];o[axis]=coord;v[axis]=sgn;t.SetOrigin(o);t.SetNormal(v)
            c=vtk.vtkClipPolyData();c.SetInputData(last);c.SetClipFunction(t);c.Update();last=vtk.vtkPolyData();last.DeepCopy(c.GetOutput())
    return last

def run(step,archive,out,scripts):
    sys.path.insert(0,str(scripts))
    from showcase_contacts import load_solids
    from mechanical_motion import bounds
    cold=json.loads((HERE/"connected-seven-stock-receipt.json").read_bytes())
    def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
    if sha(step)!=cold["diagnosticStepSHA256"] or sha(archive)!="ef436f6ff3b2a3c63ccb4a100fff39d4c0750ba1adcd7405c14da339016a00ed":raise ValueError("Wrong cold source")
    parts,_=load_solids(step);selection={};chosen=[]
    for name,v in parts.items():
        n=re.sub(r" \[[0-9.]+\]$","",name)
        if n.startswith("Equipment sheet 1"): category="sheet"
        elif n.startswith("Truss landing 250 145"):category="foot"
        elif n.startswith("Captive truss landing nutbar 250 145"):category="bar"
        elif n.startswith("Pan crossmember 250"):category="beam"
        elif n.startswith("Pan truss 1 member "):category="tube"
        else:continue
        b=bounds(v)
        if b[0]>293 or b[3]<208 or b[1]>183 or b[4]<105 or b[2]>243 or b[5]<156:continue
        g=mesh(v)
        if g.GetNumberOfCells():selection[name]=(category,g);chosen.append(name)
    if {k for k,_ in selection.values()}!={"sheet","foot","bar","beam","tube"}:raise ValueError(list(selection))
    colors={"sheet":(.69,.76,.80),"foot":(.49,.72,.67),"beam":(.70,.74,.75),"bar":(.52,.57,.59),"tube":(.60,.65,.68)}
    size=760;panels=[];projected=[]
    for hide in [False,True]:
        ren=vtk.vtkRenderer();ren.SetBackground(.965,.970,.975)
        win=vtk.vtkRenderWindow();win.SetOffScreenRendering(1);win.SetSize(size,570);win.SetMultiSamples(4);win.AddRenderer(ren)
        for name,(category,g) in selection.items():
            if category=="sheet" and hide:continue
            mp=vtk.vtkPolyDataMapper();mp.SetInputData(g);actor=vtk.vtkActor();actor.SetMapper(mp);pr=actor.GetProperty()
            pr.SetColor(colors[category]);pr.SetAmbient(.40);pr.SetDiffuse(.67);pr.SetSpecular(.13);pr.SetSpecularPower(20);ren.AddActor(actor)
            edges=vtk.vtkFeatureEdges();edges.SetInputData(g);edges.BoundaryEdgesOn();edges.FeatureEdgesOn();edges.ManifoldEdgesOff();edges.NonManifoldEdgesOff();edges.SetFeatureAngle(40)
            mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(edges.GetOutputPort());mapper.ScalarVisibilityOff()
            edge=vtk.vtkActor();edge.SetMapper(mapper);edge.GetProperty().SetColor(.26,.33,.35);edge.GetProperty().SetLineWidth(1.1);ren.AddActor(edge)
        cam=ren.GetActiveCamera();cam.SetFocalPoint(250,145,189);cam.SetPosition(293,56,314);cam.SetViewUp(0,0,1);cam.ParallelProjectionOn();cam.SetParallelScale(51)
        ren.ResetCameraClippingRange();win.Render()
        img=vtk.vtkWindowToImageFilter();img.SetInput(win);img.ReadFrontBufferOff();img.Update()
        pixels=vtk_to_numpy(img.GetOutput().GetPointData().GetScalars()).reshape((570,size,-1))[::-1,:,:3]
        panels.append(Image.fromarray(pixels.copy()))
        poses=[]
        for x,y in [(234,132),(266,158)]:
            ren.SetWorldPoint(x,y,193.676 if hide else 205.233,1);ren.WorldToDisplay();a,b,_=ren.GetDisplayPoint();poses.append((a,570-b))
        projected.append(poses);win.Finalize()
    final=Image.new("RGB",(1600,745),"#f6f8fa");draw=ImageDraw.Draw(final)
    def f(bold,sz):
        return ImageFont.truetype("C:/Windows/Fonts/"+("segoeuib.ttf" if bold else "segoeui.ttf"),sz)
    draw.text((33,12),"Pan brace screw access",font=f(True,40),fill="#182733")
    draw.text((35,66),"Equipment sheet installed",font=f(True,23),fill="#253641")
    draw.text((817,66),"Same view with the equipment sheet hidden",font=f(True,23),fill="#253641")
    for i,panel in enumerate(panels):
        ox=23+i*783;oy=110;final.paste(panel,(ox,oy))
        for j,(x,y) in enumerate(projected[i]):
            x+=ox;y+=oy;dx,dy=((-47,27) if not j else (44,-20))
            draw.line((x+dx,y+dy,x,y),fill="#aa4018",width=3);draw.ellipse((x-3,y-3,x+3,y+3),fill="#aa4018")
            draw.text((x+dx+(2 if j else -19),y+dy-10),str(j+1),font=f(True,20),fill="#863311")
    draw.text((34,697),"Port-side brace foot, two relocated screw axes. Cropped from the cold machining STEP. Screws and threads are pending.",font=f(False,22),fill="#33424d")
    out.mkdir(exist_ok=True,parents=True);path=out/"seven-brace-detail.png";final.save(path)
    r=dict(coldArchive=sha(archive),coldStep=sha(step),picture=sha(path),selected=chosen,view="Same registered port-side stock in both panels, orthographic camera, right hides only the finished-cut equipment sheet. Witnesses at x234/y132 and x266/y158 mm.")
    (out/"seven-brace-detail.json").write_text(json.dumps(r,indent=1)+"\n");print(r["picture"],len(chosen))
if __name__=="__main__":run(*map(Path,sys.argv[1:]))
