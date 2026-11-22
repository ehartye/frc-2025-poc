"""Register exact vendor electrical packaging, battery service and the accessible disconnect in iteration 002."""
from pathlib import Path
import hashlib,itertools,json,sys
import numpy as np

HERE=Path(__file__).resolve().parent
CAD=HERE.parent/"private-inputs/electrical"
CACHE=HERE.parent/"supplier/electrical-limits"
FILES={"SPARK MAX":"REV-11-2158.STEP","PDH":"REV-11-1850.STEP","roboRIO 2":"am-3000a NI roboRIO 2.STEP","VH-109":"VH-109.STEP","CB185":"am-0282-CB185-120.step","SB50":"SB 50 (Std)-02-08-STEP.stp"}
Y_UP=np.array([[1,0,0],[0,0,-1],[0,1,0]])
LONG_X=np.array([[0,0,1],[1,0,0],[0,1,0]])
FLAT=np.eye(3)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def box_corners(b):return np.array(list(itertools.product(*zip(b[:3],b[3:]))))
def bb(points):
    p=np.array(points);return np.r_[p.min(0),p.max(0)].tolist()
def gap(a,b):
    return float(np.linalg.norm(np.maximum(np.maximum(np.array(a[:3])-b[3:],np.array(b[:3])-a[3:]),0)))
def hits(a,b):
    return np.maximum(0,np.minimum(a[3:],b[3:])-np.maximum(a[:3],b[:3])).tolist()
def source(key):
    from mechanical_motion import bounds
    from showcase_contacts import load_solids
    file=CAD/FILES[key];digest=sha(file);CACHE.mkdir(parents=True,exist_ok=True)
    cached=CACHE/(key.replace(' ','-')+".json")
    if cached.exists():
        d=json.loads(cached.read_bytes())
        if d["sourceSha256"]==digest:return d
    if key!="CB185":
        ps,sheets=load_solids(file);parts={k:bounds(v) for k,v in ps.items()};kind="solid";count=len(parts)
        if sheets:raise ValueError("Unexpected surface bodies "+key)
    else:
        from OCP.STEPControl import STEPControl_Reader
        from OCP.TopAbs import TopAbs_SHELL
        from OCP.TopExp import TopExp_Explorer
        reader=STEPControl_Reader();reader.ReadFile(str(file));reader.TransferRoots();parts={}
        it=TopExp_Explorer(reader.OneShape(),TopAbs_SHELL)
        while it.More():
            parts["supplier shell "+str(len(parts)+1)]=bounds(it.Current());it.Next()
        count=len(parts);kind="open supplier shell"
        if count!=16:raise ValueError("Unexpected 120 A disconnect shell representation")
    d=dict(sourceFile=file.name,sourceSha256=digest,representation=kind,count=count,parts=parts,
        bounds=bb([p for a in parts.values() for p in (a[:3],a[3:])]))
    cached.write_text(json.dumps(d,indent=2)+"\n");print(key,count,d["bounds"],flush=True)
    return d


def plug_fit(file,roster,hardware,devices):
    origin=HERE.parents[3]/"cad/assembly-engineering/receiver-service/battery-disconnect.json"
    prior=json.loads(origin.read_bytes())
    if prior["supplierSha256"]!=sha(file):raise ValueError("Changed Anderson family shell")
    start=np.array(devices["Battery plug"]);fixed=np.array(devices["Chassis battery plug"])
    end=start+[0,-24.7,0,0,-24.7,0]
    # Historical fixed shell rotates pi around Z; its mate rotates pi about X at Y=0.
    # Rotate that virtual pair pi around Z. Require the exact new relative centers and
    # orientation. The supplier bodies and their resulting common volumes are invariant.
    reference=np.array(prior["sourceBoundsMm"])
    expected=reference[1]+reference[4]-.25
    delta=((start[:3]+start[3:])-(fixed[:3]+fixed[3:]))/2
    a=np.array(roster["Chassis battery plug"]["worldMatrixMm"])[:,:3]
    b=np.array(roster["Battery plug"]["worldMatrixMm"])[:,:3]
    if not (np.allclose(delta,[0,expected,0],rtol=0,atol=2e-7) and np.array_equal(a,np.eye(3)) and np.array_equal(b,np.diag([1,-1,-1]))):
        raise ValueError("Physical source-pair registration changed")
    rows={"-0.25":dict(commonMm3=prior["fitAtCatalogPairLengthMm3"]),"0.0":dict(commonMm3=prior["fitAtQuarterMmLongerMm3"]),
          "24.7":dict(separationLowerBoundMm=gap(end,fixed))}
    envelope=bb(box_corners(start).tolist()+box_corners(end).tolist())
    others={k:v for k,v in dict(hardware,**devices).items() if "plug" not in k}
    others["largest battery"]=[-241.5,-39,33,-58.5,39,202]
    nearest=min((gap(envelope,b),name) for name,b in others.items())
    return dict(offsetSamplesMm=rows,sourcePairReadbackSha256=sha(origin),relativeCenterMm=delta.tolist(),envelopeMm=envelope,externalSweepBoundMm=nearest[0],nearest=nearest[1],externalObjectCount=len(others),
                scope="The corrected and short-mated positions are congruent to the two historical vendor BRep measurements. The endpoint is physically separated. The finite envelope bounds all external bodies across the stroke. Intermediate shell-to-shell release, contact springs, boots and the detached cable still need independent checks.")

def run(plugin):
    sys.path.insert(0,str(plugin/"scripts"))
    from fusion_authority import register_path,verify
    from showcase_contacts import load_solids
    from mechanical_motion import bounds
    from structural_perimeter import relocate_lower
    import re
    reg=register_path(HERE.parents[3]);line=reg.parent/"issue-005/lineage.json";auth=verify(HERE.parents[3],reg,line)
    sources={k:source(k) for k in FILES}
    parts,sheets=load_solids(line.with_name("assembly.step"))
    if sheets or len(parts)!=552:raise ValueError("Wrong issued lower source")
    lower=relocate_lower({k:v for k,v in parts.items() if int(re.search(r"\[1\.(\d+)\.",k)[1])<=8},json.loads((HERE/"socket-holes.json").read_bytes())["rails"],12.7)
    hardware={k:bounds(s) for k,s in lower.items()}
    for cy in (-115.3,115.3):hardware["proposed sill "+str(cy)]=[-311.55,cy-12.7,111.125,311.55,cy+12.7,161.925]
    for x in (110,250):hardware["proposed upper crossmember "+str(x)]=[x-25.4,-242,161.925,x+25.4,242,187.325]
    # Bound each actual segment individually. The rear chamfers share the same Z.
    from four_corner_packaging import outline
    ao=outline(370,365,42,0);ai=outline(370,365,42,38.1)
    for j in range(6):
        points=[[*q,z] for q in (ao[j],ao[(j+1)%6],ai[j],ai[(j+1)%6]) for z in (101.6,203.2)]
        hardware["proposed ring "+str(j)]=bb(points)
    roster={};allparts={};devices={};reserves={}
    def place(name,key,R,cx,cy,zfloor):
        src=sources[key];q=bb(box_corners(src["bounds"])@R.T)
        t=np.array([cx-(q[0]+q[3])/2,cy-(q[1]+q[4])/2,zfloor-q[2]])
        b=bb(box_corners(src["bounds"])@R.T+t)
        roster[name]=dict(source=key,sourceSha256=src["sourceSha256"],worldMatrixMm=np.column_stack((R,t)).tolist(),boundsMm=b)
        devices[name]=b
        for k,v in src["parts"].items():allparts[name+" | "+k]=bb(box_corners(v)@R.T+t)
    place("Power distribution","PDH",LONG_X,-145,152,212)
    place("Control computer","roboRIO 2",FLAT,-142,-159,212)
    place("Radio","VH-109",FLAT,-142,-283,212)
    place("Main 120 A breaker","CB185",LONG_X,-96,274,212)
    for side in (-1,1):
        for ix,x in enumerate((38.,194.)):
            for iy,y in enumerate((116.,196.)):place(("port" if side>0 else "starboard")+f" controller {2*ix+iy+1}","SPARK MAX",Y_UP,x,side*y,212)
    place("Pan controller","SPARK MAX",Y_UP,38,273,212)
    # Commission a chassis-side shell in a removable cable bracket. Its mating
    # half, connector contacts and strain relief are separate articles.
    place("Chassis battery plug","SB50",FLAT,-299,95,180)
    # The complementary housing turns 180 degrees about the true local X axis.
    # The 0.25 mm extra opening is an independent family-model fit control.
    place("Battery plug","SB50",np.diag([1,-1,-1]),-299,95-33.5281248,180)
    installed=dict(hardware,**devices)
    def clearance(target,objects):
        ranked=sorted((gap(target,v),k,hits(target,v)) for k,v in objects.items())
        return dict(minBoundMm=ranked[0][0],nearest=ranked[0][1],overlappingBoxes=[dict(id=k,overlapXYZmm=s) for g,k,s in ranked if g==0])
    neighbors={k:clearance(b,{n:s for n,s in installed.items() if n!=k}) for k,b in devices.items()}
    # Source battery dimensions are supplier tolerances, unlike the deliberately larger
    # keep-out for connector boots, handle, keeper removal and finger positions.
    gauges={"largest battery parked":[-241.5,-39.,33.,-58.5,39.,202.],
            "dressed battery and grip vertical 250mm":[-255.,-60.,33.,-45.,60.,510.],
            "rearward carry after 250mm":[-650.,-60.,283.,-45.,60.,510.],
            "full breaker above-hand corridor":[-143.,233.,256.,-49.,315.,720.],
            "plug grip from above":[-329.,27.,198.,-269.,115.,440.]}
    service={k:clearance(b,{n:s for n,s in installed.items() if k!="full breaker above-hand corridor" or n!="Main 120 A breaker"}) for k,b in gauges.items()}
    service["SB50 matched source shell fit"]=plug_fit(CAD/FILES["SB50"],roster,hardware,devices)
    negative_plug=[-215.25625,58.9890624,180.,-178.74375,107.0109376,196.047565]
    for key,axis,extension,margin in (("Power distribution",1,25.,6.),("Control computer",0,25.,5.)):
        b=devices[key].copy();b[axis]-=extension;b[axis+3]+=extension
        reserves[key+" terminal allowance"]=b
    for k,b in devices.items():
        if "controller" not in k:continue
        q=b.copy();q[0]-=27;q[3]+=27;reserves[k+" lead allowance"]=q
    service["otherDevicesVsLeads"]={k:clearance(b,{n:v for n,v in installed.items() if not k.startswith(n)}) for k,b in reserves.items()}
    return dict(authority=auth,pooledSources={k:{n:v for n,v in row.items() if n!="parts"} for k,row in sources.items()},
        sourceCounts={k:v["count"] for k,v in sources.items()},sourceArticleInstances=len(roster),mountedSourceBodiesAndShells=len(allparts),
        projectedLowerBodies=len(lower),candidatePlacements=roster,deviceNeighbors=neighbors,gaugesMm=gauges,service=service,
        negative=dict(plannedBattery=[-245,-40,33,-55,40,203],plannedCentralPanel=[-175,-150,155,175,150,235],overlapXYZmm=[120,80,48],earlierFixedPlugBounds=negative_plug,earlierPlugVsLiftXYZmm=hits(negative_plug,gauges["dressed battery and grip vertical 250mm"])),
        scope="Positions are prospective in iteration 002. Independent full-article world bounds give conservative positive separations. Overlapping boxes are unresolved unless a narrower geometric measurement is performed. The main breaker supplier file contains 16 open shells; there is no solid-contact inference. The separate breaker-access receipt checks a finite finger against the red trip button. The original once-issued battery method informs the 250 mm rise. New mounting brackets, battery retention, contacts and booted cable travel, rated cable bend radius, pan sweep and dressed wiring remain open. The small frame test did not install electronics in Fusion.")


if __name__=="__main__":
    r=run(Path(sys.argv[1]));(HERE/"inset-electrical-package.json").write_text(json.dumps(r,indent=2)+"\n")
    print(r["sourceCounts"],r["mountedSourceBodiesAndShells"])
    for k,d in r["deviceNeighbors"].items():print(k,round(d["minBoundMm"],2),d["nearest"])
    for k,d in r["service"].items():
        if "minBoundMm" in d:print(k,round(d["minBoundMm"],2),d["nearest"],[(g["id"],g["overlapXYZmm"]) for g in d["overlappingBoxes"]][:3])

