"""Draw two orthogonal sections of the installed left drum and shaft stop."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.gp import gp_Pln, gp_Pnt, gp_Dir
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from fit import HERE, X, Z, j, routing

COLORS = {"Fixed arm and bearing":"#545c65","Primary drum":"#22748a","Short shaft":"#23374c",
          "Recessed spacer":"#239071","Pinned end stop":"#b85b28","Cross screw and locknut":"#865493"}


def edges(shape, axis, level):
    origin = [0.,0.,0.]
    normal = [0.,0.,0.]
    origin[axis],normal[axis] = level,1.
    op = BRepAlgoAPI_Section(shape, gp_Pln(gp_Pnt(*origin),gp_Dir(*normal)), False)
    op.Build()
    if not op.IsDone():
        raise ValueError("Section failed")
    walk = TopExp_Explorer(op.Shape(),TopAbs_EDGE)
    while walk.More():
        curve = BRepAdaptor_Curve(TopoDS.Edge(walk.Current()))
        a,b = curve.FirstParameter(),curve.LastParameter()
        yield [curve.Value(a+(b-a)*i/40).Coord() for i in range(41)]
        walk.Next()


def draw():
    volumes = j.read(HERE/"fit.json")["volumeMm3"]
    kit = j.cad.read_named(HERE/"stationary-shaft.step",[{"id":name} for name in volumes])
    primary = routing(560)
    groups = {"Fixed arm and bearing":[primary[k+"-L"] for k in ("bearing-arm","bearing-envelope")],
              "Primary drum":[primary[k+"-L"] for k in ("drum-core","drum-flange-0","drum-flange-1")],
              "Short shaft":[kit["fixed-haul-shaft"]],"Recessed spacer":[kit["haul-sleeve-L"]],
              "Pinned end stop":[kit["haul-stop-L"]],
              "Cross screw and locknut":[kit["haul-screw-L"],kit["haul-locknut-L"]]}
    fig, axes = plt.subplots(1,2,figsize=(11,4.6))
    fig.subplots_adjust(left=.07,right=.98,bottom=.23,top=.79,wspace=.20)
    fig.suptitle("Axial retention at the left primary drum",x=.07,ha="left",fontsize=19,weight="bold")
    for ax,section,level,ordinate,title in zip(axes,(0,2),(X+.2,Z+.2),(2,0),
            ("Elevation through the shaft","Plan through the cross screw")):
        for label,shapes in groups.items():
            for shape in shapes:
                for pts in edges(shape,section,level):
                    ax.plot([p[1] for p in pts],[p[ordinate] for p in pts],color=COLORS[label],lw=1.6)
        ax.set(xlim=(212,283),ylim=((306,354) if ordinate==2 else (-117,-67)),aspect="equal",
               xlabel="Distance from robot center (mm)",ylabel="Height (mm)" if ordinate==2 else "Fore/aft (mm)",
               title=title)
        ax.spines[["right","top"]].set_visible(False)
    fig.legend([Line2D([],[],color=c,lw=3) for c in COLORS.values()],list(COLORS),
               loc="lower center",ncol=3,frameon=False)
    fig.savefig(HERE/"drum-retention.png",dpi=150)
    plt.close(fig)


if __name__=="__main__":
    draw()
