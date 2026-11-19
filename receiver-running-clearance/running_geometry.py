"""Machine continuous button relief and opposed guide lands as two independent revisions."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/"extension-support"))
import rail_capture as rail
from rail_geometry import exchange, button_contact, translated
from tray import FACE, TRAY, box, cad, common, digest, subtract, unite, volume, write_step, finalize_step
sys.path.insert(0,str(ROOT/"selected-match"))
from rail_profile import prism_y

STATIONS=(-66.,42.)
START_X=114.6
FINISH_X=98.
CUT_MM=3.4
LANE_HALF=5.
FLOOR_RISE=.96
UPPER_GAP=.06


def read(path):
    return json.loads(path.read_bytes())


def bottom_mm(x):
    return 485.+CUT_MM*np.clip((START_X-x)/(START_X-FINISH_X),0.,1.)


def selected():
    old=ROOT/"extension-support"
    spec=read(old/"keepers.json")
    rails=exchange(old/"rail-fixture.step",spec["stepSha256"],spec["volumesMm3"])
    old=ROOT/"shelf-structure"
    shelf=read(old/"installed.json")
    tray=exchange(old/"stow-receiver.step",shelf["manufacturingSha256"],shelf["volumeMm3"])
    return rails,{k:tray[k] for k in (FACE,TRAY)},spec,shelf


def machine_shelf(bodies):
    changed=bodies.copy()
    for y in STATIONS:
        section=[(-101.,483.),(START_X,483.),(START_X,485.),(FINISH_X,485.+CUT_MM),(-101.,485.+CUT_MM)]
        cutter=prism_y(section,y-LANE_HALF,2*LANE_HALF)
        for k,v in changed.items():
            changed[k]=subtract(v,cutter)
    return changed


def floor_strip(shape,y,top,width):
    mask=box(-160,y-width/2,top-.02,160,y+width/2,top+FLOOR_RISE)
    # Copy the local source floor. Original holes stay open through the raised
    # face; only the interior of the existing extrusion receives extra stock.
    return unite(shape,common(translated(shape,z=FLOOR_RISE),mask))


def machine_rails(old):
    pristine=rail.stock()
    changed={}
    for side,y in (("L",180.),("R",-180.)):
        nm,no=f"slide-{side}-middle",f"slide-{side}-outer"
        offset=rail.RACE_RADIUS-rail.RADIUS-UPPER_GAP
        middle=subtract(pristine[nm],rail.slot(rail.INNER_X-1,rail.INNER_X+rail.STROKE/2+1,rail.INNER_Z-offset,y-15))
        middle=subtract(middle,rail.slot(rail.OUTER_X-rail.STROKE/2-1,rail.OUTER_X+1,rail.OUTER_Z+offset,y-15))
        changed[nm]=floor_strip(middle,y,445.,17.)
        changed[no]=floor_strip(old[no],y,441.,24.)
    return changed


def witnesses(rails,new_shelf,old_shelf):
    result={}
    for side in "LR":
        inner=rails[f"drilled-slide-{side}-inner"]
        mid0=rails[f"slide-{side}-middle"]; outer=rails[f"slide-{side}-outer"]
        pin0=rails[f"inner-crosspin-{side}"]
        dogs=[rails[n] for n in rails if n.startswith(f"outer-dog-{side}-")]
        for s in (0.,185.,370.):
            mid=translated(mid0,x=s/2); pin=translated(pin0,x=s)
            sheet=translated(inner,x=s)
            paper={}
            for dz in (-.05,-.03,0.,.05,.07):
                values=[cad.intersection(translated(sheet,z=dz),mid),cad.intersection(translated(mid,z=dz),outer),
                        cad.intersection(translated(pin,z=dz),mid),sum(cad.intersection(translated(mid,z=dz),dog) for dog in dogs)]
                paper[f"{dz:g}"]=values
                if [v>1e-7 for v in values] != [dz<-.04,dz<-.04,dz>.06,dz>.06]:
                    raise ValueError(f"Running fit differs from stated four limits, {side}/{s}/{dz}: {values}")
            result[f"{side}/{s:g}"]=paper
    orig=unite(old_shelf[FACE],old_shelf[TRAY])
    new=unite(new_shelf[FACE],new_shelf[TRAY])
    pads={}
    for y in STATIONS:
        # Protect the complete source pad across a 0.1 mm lateral/longitudinal
        # positioning allowance, including stock above its working surface.
        mask=cad.brep(["cyl",[118.,y,484.,0,0,1,3.3,16.]])
        removal=volume(common(subtract(orig,new),mask))
        old=button_contact(orig,0,118.,y,484.9); revised=button_contact(new,0,118.,y,484.9)
        if removal>1e-7 or abs(revised["firstGapMm"]-.1)>1e-5 or revised["faceFractionWithin0p15Mm"]<.999:
            raise ValueError("Original stow pad or clearance lost")
        pads[str(y)]={"removedInWorkingDiskPlus0p1mm3":removal,"original":old,"new":revised,
                      "travelGapsMm":{str(s):button_contact(new,s,118.,y,484.9)["firstGapMm"] for s in (3,10,21.25,21.375,22,24,35,100,185,215,218,222)}}
    return result,pads


def run(output):
    output.mkdir(exist_ok=True,parents=True)
    rails,shelf,keeper,installed=selected()
    replacement={**machine_rails(rails),**machine_shelf(shelf)}
    step=output/"running-kit.step"
    write_step(step,replacement);finalize_step(step)
    new=cad.read_named(step,[{"id":k} for k in replacement])
    if any(cad.solid_count(v)!=1 for v in new.values()):raise ValueError("Disconnected replacement stock")
    baseline={**rails,**shelf}
    parts={}
    for k,v in new.items():
        added=volume(subtract(v,baseline[k])); removed=volume(subtract(baseline[k],v))
        v0,c0=cad.volume(baseline[k]);v1,c1=cad.volume(v)
        if abs((v1-v0)-(added-removed))>.05: raise ValueError("Section accounting does not close")
        parts[k]=dict(originalMm3=v0,revisedMm3=v1,addedMm3=added,removedMm3=removed,
                      originalCenterMm=list(c0),revisedCenterMm=list(c1))
    lands,pads=witnesses({**rails,**new},new,shelf)
    return dict(keeperSourceSha256=keeper["stepSha256"],shelfSourceSha256=installed["manufacturingSha256"],
                kitSha256=digest(step),parts=parts,landWidthsMm=[17.,24.],raisedLandsMm=FLOOR_RISE,
                nominalLowerTravelMm=1-FLOOR_RISE,nominalUpperTravelMm=UPPER_GAP,
                contactOrder=["inner floor","middle floor","inner upper crosspin","outer upper dogs"],
                contactWitnessMm3=lands,retainedStowPads=pads,
                section=dict(yMm=list(STATIONS),halfWidthMm=LANE_HALF,startRampXMm=START_X,fullReliefXMm=FINISH_X,reliefDepthMm=CUT_MM))


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("output",type=Path);a=p.parse_args()
    receipt=run(a.output)
    (a.output/"geometry.json").write_bytes((json.dumps(receipt,indent=2,allow_nan=False)+"\n").encode())
