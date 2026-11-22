"""Compare the inset mounting footprint and weighed chassis stock on the original Red 2 stop-to-stop clock."""
import hashlib,json,math,sys
from pathlib import Path
from crossframe_clock import run as drive
from four_corner_packaging import outline
from pan_route_schedule import run as timed_route

HERE=Path(__file__).resolve().parent
STRATEGY=HERE.parent/"strategy"


def load(f):return json.loads((HERE/f).read_bytes())


def run():
    master=load("inspection-mass-552.json")
    base=master["currentCohortsKg"]
    ring=load("structural-perimeter-101.6-38.1-101.6-inset-12.7.json")
    fit=load("inset-saddle-exterior.json");duty=load("terminal-mass-clock.json")["clock"]["cases"]
    if ring["authority"]!=fit["authority"] or ring["authority"]!=master["authority"]:raise ValueError("Mixed native sources")
    loop=outline(370,365,42,38.1/2)
    cut_length=sum(math.dist(a,b) for a,b in zip(loop,loop[1:]+loop[:1]))*.001
    catalog_ring=cut_length*1.544*.45359237/.3048
    new_rails=0.
    for r in load("socket-holes.json")["rails"].values():
        ax=r["lengthAxis"];length=r["maximumMm"][ax]-r["minimumMm"][ax]-25.4
        net=(50.8*25.4-44.45*19.05)*length-6*math.pi*(5.3/2)**2*(2*3.175)
        new_rails+=net*2700e-9
    four_sills=sum(v["kg"] for v in fit["sillsAndUpperSupport"].values())
    review=load("low-line-frame-clock.json")["remodeledStructuralItems"]
    old_central=sum(v["kg"] for v in review[:4])
    ring_mass=max(catalog_ring,fit["grossRingKg"])
    eight=8*fit["eachSaddleGroupGrossKg"]
    # Include the actual candidate inner screw sleeves even though the distribution and fit
    # remain under review. Do not use machining savings for the new saddle cross holes.
    inner_charge=load("inner-socket-sleeves.json")["directSleeveKg"]
    change=(ring_mass-base["plannedSixMiteredGrossTubes"] +eight-base["measuredFourCartridgeVolumes"]+
            four_sills-old_central+new_rails-base["measuredFourDrilledRails"]+inner_charge)
    options={}
    for name,row in master["cases"].items():
        old=duty[name]
        options[name]=(row["fieldKg"]+change,old["estimatedYawInertiaKgM2"],old["cgMm"])
    clock=drive(options,module_half=(.2693,.2643))
    out=dict(authority=fit["authority"],halfTrackXYm=[.2693,.2643],ringGrossCatalogKg=catalog_ring,
        ringCADKg=fit["grossRingKg"],newFourLowerRailsKg=new_rails,newSillsAndCrossbarsKg=four_sills,
        eightCompleteSaddleGroupsKg=eight,innerScrewContingencySleevesKg=inner_charge,
        originalGrossCentralStockKg=old_central,fullRetainedJoiningAndBracketsAllowanceKg=review[-1]["kg"],
        netTo552Kg=change,cases={name:{**row,"fieldKg":row["fieldKg"]+change,"fixedRobotKg":row["fixedRobotKg"]+change,
            "withBumpersKg":row["withBumpersKg"]+change,"R103MarginKg":row["R103MarginKg"]-change,
            "R408MarginKg":row["R408MarginKg"]-change,"I103AdditionalMechanismMarginKg":row["I103AdditionalMechanismMarginKg"]-change}
            for name,row in master["cases"].items()},clock=clock,
        scope="Source-derived stock and eight groups include nominal proposed metal and all 64 actual catalog screw shapes. The original full 8.08665 kg allowance for splices, backing and unresolved frame joints is preserved in addition, as are all high-mass reserves. Four saddle-motor collisions in the unrelieved screening configuration remain a design failure. The yaw inertia, cg and final 12 mm docking stand-off remain adverse design references while the contracted wheelbase is explicitly replayed. No credit from slot drilling and no claimed operational score without independently simulated receiver support.")
    file=HERE/"inset-structure-clock.json";file.write_text(json.dumps(out,indent=2)+"\n")
    schedule=timed_route(.012,(.2693,.2643),file)
    (STRATEGY/"inset-route-12mm.json").write_text(json.dumps(schedule,indent=2)+"\n")
    return out


if __name__=="__main__":
    r=run();print(r["cases"],r["netTo552Kg"])
