"""Draw two axonometric views from the reopened, identified solid geometry."""
from pathlib import Path
import argparse
import math
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location
from build_assembly import COLORS,PARTS,read_named


def dot(a,b):
    return sum(i*j for i,j in zip(a,b))


def cross(a,b):
    return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])


def unit(a):
    n=math.sqrt(dot(a,a))
    return [v/n for v in a]


def project(path,cx,cutaway):
    camera=unit([0.65,-1,0.7])
    right=unit(cross([0,0,1],camera))
    up=cross(camera,right)
    colors={p["id"]:COLORS[p["group"]] for p in PARTS}
    omit={"bumper","plate","shelf","belt","lift","slide","guide","sensor","compute","power","battery"}
    excluded={p["id"] for p in PARTS if p["group"] in omit or p["frame"]!="base" or p["id"].startswith("mast")}
    shapes=read_named(path)
    rows=[]
    for key,shape in shapes.items():
        if cutaway and key in excluded:
            continue
        BRepMesh_IncrementalMesh(shape,0.7,False,0.22,False)
        faces=TopExp_Explorer(shape,TopAbs_FACE)
        while faces.More():
            face=TopoDS.Face(faces.Current())
            loc=TopLoc_Location()
            mesh=BRep_Tool.Triangulation_s(face,loc)
            if mesh:
                points=[None]+[mesh.Node(i).Transformed(loc.Transformation()).Coord() for i in range(1,mesh.NbNodes()+1)]
                for i in range(1,mesh.NbTriangles()+1):
                    a,b,c=(points[n] for n in mesh.Triangle(i).Get())
                    n=cross([j-k for j,k in zip(b,a)],[j-k for j,k in zip(c,a)])
                    if face.Orientation()==TopAbs_REVERSED:
                        n=[-v for v in n]
                    if dot(n,camera)<=1e-6:
                        continue
                    scale=0.35 if not cutaway else 0.43
                    baseline=467 if not cutaway else 370
                    xyz=[a,b,c]
                    coord=[(cx+scale*dot(p,right),baseline-scale*dot(p,up)) for p in xyz]
                    shade=0.64+0.36*max(0,dot(unit(n),unit([-0.4,-0.65,1])))
                    rgb=bytes.fromhex(colors[key][1:])
                    color="#"+"".join(f"{round(v*shade):02x}" for v in rgb)
                    triangle="M"+"L".join(f"{x:.1f},{y:.1f}" for x,y in coord)+"Z"
                    rows.append((sum(dot(p,camera) for p in xyz),color,triangle))
            faces.Next()
    # Collect consecutive triangles of the same color into one SVG path.
    rendered=[]
    prev=None
    for _,color,tri in sorted(rows):
        if color!=prev:
            if prev is not None:
                rendered.append('"/>')
            rendered.append(f'<path fill="{color}" stroke="{color}" stroke-width=".4" d="')
            prev=color
        rendered.append(tri)
    rendered.append('"/>')
    return "".join(rendered)


def render(review_folder,out):
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="620" viewBox="0 0 1100 620" role="img"><title>Station-fed L1 assembly in its placing pose and the drivetrain underneath</title><rect width="1100" height="620" fill="#f6f7f4"/>']
    svg.append('<g fill="#132c34" font-family="Arial,sans-serif"><text x="35" y="42" font-size="27" font-weight="bold">Station-fed L1 package</text><text x="36" y="70" font-size="15">B4 derivative · named CAD solids · millimetres</text><text x="37" y="112" font-size="19">Placing pose</text><text x="580" y="112" font-size="19">Drivetrain with deck and bumpers removed</text></g>')
    svg.append(project(review_folder/"place.step",270,False))
    svg.append(project(review_folder/"stowed.step",815,True))
    svg.append('<g fill="#203441" font-family="Arial,sans-serif" font-size="15"><text x="36" y="563">Gold receiver floor · teal end-rim cartridges · blue bumpers</text><text x="580" y="563">Two driven center axles · four transfer loops · six wheels</text><text x="36" y="594">Outlined equipment volumes and pitch surfaces represent allocated space. Purchased hardware remains open.</text></g></svg>')
    out.write_text("".join(svg)+"\n",encoding="utf-8")


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("review_folder",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    render(args.review_folder,args.output)
