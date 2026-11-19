"""Check two gussets, both deck options and staged fastener access against registered hardware."""
import argparse
import numpy as np
from mast import HERE,ROOT,MAST,RAIL,PROFILE,deck,j,overlay,slot,prism,compound,upward_removal,outward_removal,lateral_jog
from pathlib import Path


def reopen(path,names,pin):
    if j.digest(path)!=pin: raise ValueError("Changed source "+path.name)
    return j.cad.read_named(path,[{"id":k} for k in names])


def run(art,lamp,rib):
    src,counts,pins=deck.source(art,lamp,ROOT/"cover-retention")
    import harness_fit
    from routing import power_feed
    # Replace the old full feed rectangles with the issued continuous route axes.
    # Retain the supplier-supported guides and joints, including removable troughs.
    for k in list(src):
        if k in harness_fit.RETIRED or k=="wire/power-feed" or k.startswith("jacket/"):
            del src[k]
    lines=harness_fit.jacket_and_form()[0]
    power=[]
    for length,pts in power_feed(-329):
        straight=abs(length-np.linalg.norm(pts[-1]-pts[0]))<1e-6
        power.append((length,pts,None if straight else 1.))  # edges() uses the true arc through three samples
    lines["previous-roof-power"]=(power,np.zeros(3),4.35)
    edges=[]
    for k,(path,offset,radius) in lines.items():
        for e in harness_fit.edges(path,offset):
            edges.append((k,e,j.cad.bounds(e),radius+.5))
    tied=j.read(ROOT/"deck-joints/fit.json")
    src.update(reopen(ROOT/"deck-joints/deck-through-ties.step",tied["volumeMm3"],tied["stepSha256"]))
    for k in ("deck-main","cross-front","cross-rear"): del src[k]
    main="tie-machined-deck-main"
    manufactured=overlay(src,main)
    rib_meta=j.read(rib/"geometry.json")
    alternative=reopen(rib/"ribbed-deck.step",["integral-rib-deck"],rib_meta["stepSha256"])["integral-rib-deck"]
    alt=slot(alternative)
    alt_path=HERE/"slotted-ribbed-deck.step"
    j.write_step(alt_path,{"mast-slotted-ribbed-deck":alt})
    j.finalize_step(alt_path)
    alt=reopen(alt_path,["mast-slotted-ribbed-deck"],j.digest(alt_path))["mast-slotted-ribbed-deck"]
    path=HERE/"mast-joints.step"
    j.write_step(path,manufactured)
    j.finalize_step(path)
    parts=reopen(path,manufactured,j.digest(path))
    volumes={k:round(j.cad.volume(s)[0],4) for k,s in parts.items()}
    if max(abs(v-j.cad.volume(manufactured[k])[0]) for k,v in volumes.items())>.002:
        raise ValueError("STEP reopen failed")
    before={main,*["backup-"+s+"-drilled-mast" for s in ("L","R")],*["drive-rail-inner-"+s for s in ("L","R")]}
    obs={**{k:v for k,v in src.items() if k not in before},**parts}
    boxes={k:j.cad.bounds(s) for k,s in obs.items()}
    def screen(shape,skip=(),line_skip=()):
        result=deck.screen(shape,obs,{k:v for k,v in boxes.items() if k not in skip})
        a=j.cad.bounds(shape)
        gaps=[(j.distance(shape,e)-r,k) for k,e,b,r in edges if k not in line_skip and j.lower_bound(a,b)<r+12]
        result["closeRoutedLeadMarginMm"]=min(gaps,default=None)
        return result
    checks={}
    for k,s in parts.items():
        if k.endswith(("-mast","-rail","-deck")): continue
        row=screen(s,{k})
        row["alternativeDeckCommonMm3"]=round(j.cad.intersection(s,alt),4) if j.lower_bound(j.cad.bounds(s),j.cad.bounds(alt))<.001 else 0
        checks[k]=row
    svc={}
    service_controls={}
    service_paths={}
    tall_control={}
    for side,sign in (("L",1),("R",-1)):
        prefix="mast-base-"+side+"-"
        moving=upward_removal(sign)
        ignore={k for k in parts if k.startswith(prefix) and not k.endswith(("-mast","-rail"))}
        # Each bumper half is a removable unit. Keep the robot-side threaded sleeves.
        bumper={k for k in obs if k.startswith(("bumper-backing-","bumper-foam-","bumper-cover-"))}
        removable=j.read(ROOT/"fixed-camera-route/guides.json")["removableTroughs"]
        opened={k for k in removable if k.startswith("roof-host-trough-")} if sign<0 else set()
        line_skip={"roof-host","roof-host-form"} if sign<0 else set()
        service_controls[side+"/upward"]=screen(moving,ignore)
        service_controls[side+"/directOutward"]=screen(outward_removal(sign),ignore|opened|bumper,line_skip)
        tall_control[side]=round(j.cad.intersection(outward_removal(sign,full=True),obs["backup-"+side+"-bearing-support"]),4)
        svc[side+"/upward"]=screen(moving,ignore|opened|bumper,line_skip)
        for dx in (0.,115.,-135.):
            jog=screen(lateral_jog(sign,dx),ignore|opened|bumper,line_skip) if dx else None
            out=screen(outward_removal(sign,dx),ignore|opened|bumper,line_skip)
            if all(not r["overlapMm3"] and (not r["closeRoutedLeadMarginMm"] or r["closeRoutedLeadMarginMm"][0]>=0) for r in (jog,out) if r):
                if jog:svc[side+"/jog"]=jog
                svc[side+"/outward"]=out
                service_paths[side]=dx
                break
            if jog:service_controls[side+"/jog"+str(dx)]=jog
            service_controls[side+"/out"+str(dx)]=out
        for index,(x,z) in enumerate(RAIL+MAST):
            top=index>=len(RAIL)
            inner=210 if top else 229
            own={k for k in parts if k.startswith(prefix+str(index)+"-")}
            gauges={
                "inboardHeadSocket":j.rod(x,sign,inner-21,inner,z,7),
                "outboardNutSocket":j.rod(x,sign,239.6,264,z,7),
                "screwWithdrawal":compound([j.rod(x,sign,inner-43,inner,z,6),
                                            j.rod(x,sign,inner-39,247.5,z,3)])}
            if top: gauges["sleeveWithdrawal"]=j.rod(x,sign,180,235,z,5)
            for label,gauge in gauges.items():
                ignore=own.copy()
                if top and label=="screwWithdrawal": ignore.discard(prefix+str(index)+"-sleeve")
                svc[f"{side}/{index}/{label}"]=screen(gauge,ignore|opened,line_skip)
    negative={}
    witness={}
    for side,sign in (("L",1),("R",-1)):
        prefix="mast-base-"+side+"-"
        splice=parts[prefix+"splice"]
        negative[side]=j.cad.intersection(splice,src[main])
        for direction in (-1,1):
            delta=[0,0,.5*direction]
            chain=(("mast-sleeve",parts[prefix+"mast"],parts[prefix+"4-sleeve"]),
                   ("sleeve-screw",parts[prefix+"4-sleeve"],parts[prefix+"4-screw"]),
                   ("screw-splice",parts[prefix+"4-screw"],splice),
                   ("splice-base-screw",splice,parts[prefix+"0-screw"]),
                   ("base-screw-rail",parts[prefix+"0-screw"],parts[prefix+"rail"]))
            witness[f"{side}/{direction}"]={k:round(j.cad.intersection(j.cad.translated(a,delta),b),5) for k,a,b in chain}
    row={"sources":{**pins,"deck-through-ties.step":tied["stepSha256"],"ribbed-deck.step":rib_meta["stepSha256"]},
         "counts":counts,"stepSha256":j.digest(path),"ribbedSlottedSha256":j.digest(alt_path),"volumeMm3":volumes,
         "material":checks,"tools":svc,"serviceControls":service_controls,"extractionJogXmm":service_paths,
         "tallBracketBearingMm3":tall_control,
         "halfMmBearingChainCommonMm3":witness,"unrelievedDeckCommonMm3":negative,
         "stockRemovedMm3":{k:j.cad.volume(src[k])[0]-v for k,v in {
             main:volumes["mast-slotted-deck"],
             **{"drive-rail-inner-"+s:volumes["mast-base-"+s+"-rail"] for s in ("L","R")},
             **{"backup-"+s+"-drilled-mast":volumes["mast-base-"+s+"-mast"] for s in ("L","R")}}.items()},
         "ribbedDeckRemovedMm3":j.cad.volume(alternative)[0]-j.cad.volume(alt)[0]}
    j.write(HERE/"fit.json",row)
    bad={k:r for k,r in {**checks,**svc}.items() if r["overlapMm3"] or r.get("alternativeDeckCommonMm3",0)>.001 or
         (r["closeRoutedLeadMarginMm"] and r["closeRoutedLeadMarginMm"][0]<-.001)}
    if bad or len(service_paths)!=2: raise ValueError([service_paths,[(k,r["overlapMm3"]) for k,r in bad.items()]])


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for key in ("art","lamp","rib"):p.add_argument(key,type=Path)
    a=p.parse_args()
    run(a.art,a.lamp,a.rib)
