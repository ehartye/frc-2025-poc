"""Recompute load placement after removing the air kit from the fixed-height robot."""
import importlib.util
import itertools
import math

from service import HERE, ROOT, cad, read
from account import GROUP, write
from accounting import locks

SHOP = HERE.parent/"competition-package/shop-duplex"
spec = importlib.util.spec_from_file_location("shop_mass",SHOP/"mass.py")
shop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(shop)
st = shop.stability


def extreme(loads, axis, maximize):
    """Bound CG for signed machining deltas, with positive total robot mass."""
    choices = []
    for item in loads:
        subtract = item.mass[1] <= 0
        far = maximize != subtract
        q = sum(c*item.bounds[i + (3 if (c > 0)==far else 0)] for i,c in enumerate(axis))
        choices.append((q,*item.mass))
    span = max(abs(q) for q,_,_ in choices) * 2 + 1
    lo,hi = -span,span
    def residual(x):
        return sum((q-x)*(b if (q>x)==maximize else a) for q,a,b in choices)
    if residual(lo)<0 or residual(hi)>0:
        raise ValueError("Signed center bound was not bracketed")
    for _ in range(55):
        mid = (lo+hi)/2
        if residual(mid)>0:
            lo=mid
        else:
            hi=mid
    return (lo+hi)/2


def prepare(mode):
    pose,case,file = ("stowed","red","locks.step") if mode=="high" else ("place","low-red-place","low-locks.step")
    values = cad.MODEL["poses"][pose]
    original,_ = st.loads_for(pose)
    shapes = shop.original.flatten(shop.pose_layers(values["height"],values["stroke"])[0])
    original += shop.additions(pose,values,shapes)
    baseline = read(SHOP/"mass.json")["poses"][pose]["empty"]["mass_kg"]
    if any(abs(sum(r.mass[i] for r in original)-baseline[i])>1e-7 for i in (0,1)):
        raise ValueError("Full 545-body mass ledger differs from the recorded pose")
    issued = read(HERE.parent/"fixed-height/native.json")[case]
    added,removed = issued["addedVolumesMm3"],issued["editedVolumesMm3"]
    hardware = cad.read_named(HERE.parent/"fixed-height"/file,[{"id":n} for n in (*added,*removed)])
    for key,v in added.items():
        material,w = (2.7e-6,.20) if "fillet" in key else (7.85e-6,.10)
        measured,center = cad.volume(hardware[key])
        if abs(measured-v)>.001:
            raise ValueError("Lock article has changed")
        original.append(st.Load(key,(v*material*(1-w),v*material*(1+w)),(*center,*center),cad.bounds(hardware[key])))
    for key,v in removed.items():
        material = .95e-6 if "lift-shoe" in key else 2.7e-6
        bounds = cad.bounds(shapes[key])
        original.append(st.Load("cut "+key,(v*material*1.05,v*material*.95),bounds,bounds))
    # Permit all remaining mass in each changed electrical group to relocate within
    # either old or new installation. Keep the full source allowance for the old light.
    kit_dir = HERE.parent/"signal-light"
    replacement = cad.read_named(kit_dir/"deck-mount.step",[{"id":k} for k in ("front-electronics-platform","light-spacer")])
    lamp_box = (112,-283,250,178,-217,346)
    deck_box = st.union([cad.bounds(s) for s in replacement.values()]+[lamp_box])
    for name in ("radio, controller, RSL, main breaker","battery hold-down, panel, posts and brackets"):
        i = next(i for i,r in enumerate(original) if r.name==name)
        row = original[i]
        bounds = st.union([row.extent,deck_box])
        original[i] = st.Load(name,row.mass,bounds,bounds)
    allowance = read(kit_dir/"review.json")["incremental_budget_kg"]
    robot = st.union([cad.bounds(s) for s in shapes.values()]+[deck_box])
    original.append(st.Load("light and mounting change",tuple(allowance),robot,robot))
    return original


def summarize(rows, mode):
    mass = [sum(r.mass[i] for r in rows) for i in (0,1)]
    cg = [[extreme(rows,k,False),extreme(rows,k,True)] for k in ((1,0,0),(0,1,0),(0,0,1))]
    radius = max(math.hypot(x,y) for x,y in itertools.product(cg[0],cg[1]))/1000
    inertia = sum(max(0,r.mass[1])*sum(max(abs(r.extent[i]),abs(r.extent[i+3]))**2 for i in (0,1))
                  for r in rows)/1e6
    span = shop.original.drive_model.CENTERS
    half_track = st.DEFAULT["trackM"]*500
    margin = span-max(extreme(rows,(s,0,st.CLOCK["maximumWheelAccelMps2"]/st.G),True) for s in (-1,1))
    side_fraction = (half_track-max(abs(y) for y in cg[1]))/(2*half_track)
    result = {"running_mass_kg":mass,"cg_mm":cg,"yaw_inertia_upper_kg_m2":inertia,
              "straight_load_line_margin_mm":margin,"static_side_fraction_lower":side_fraction}
    if mode=="high":
        path = locks.direct_path(locks.corrected())
        args = {"inertiaKgM2":inertia,"cgOffsetM":radius,"minSideWeightFraction":side_fraction,
                "outerAxleWeightFraction":1}
        checks = [locks.assess(dict(args,massKg=m),{"segments":path.segments}) for m in mass]
        result["straight_park"] = {"passes_endpoints":all(c["parameterCaseClears"] for c in checks) and margin>0,
            "peak_phase_A":max(c["extrema"]["phaseA"] for c in checks),
            "required_mu":max(c["extrema"]["frictionRequired"] for c in checks),
            "min_bus_V":min(c["extrema"]["busV"] for c in checks)}
    return result


if __name__=="__main__":
    out = {}
    for mode in ("high","low-place"):
        all_rows=prepare(mode)
        dry=[r for r in all_rows if r.name!=GROUP]
        if len(dry)!=len(all_rows)-1:
            raise ValueError("Air allocation must occur exactly once")
        out[mode]={"with_air":summarize(all_rows,mode),"no_air":summarize(dry,mode)}
        if mode=="low-place":
            camera = read(ROOT/"moving-observer/mass.json")
            # Bracket leads and moving hardware anywhere in the assembled envelope.
            # The fixed converter/protection stays on the chassis; no credit for the shelf bores.
            envelope=st.union([item.extent for item in all_rows])
            mounted=st.Load("camera and moving loom",camera["stage_carried_kg"],envelope,envelope)
            electrical=st.Load("camera power",camera["estimated_installation"]["fixed_power_conversion_protection_wires_kg"],
                               envelope,envelope)
            out["low-place-camera"] = {"with_air":summarize(all_rows+[mounted,electrical],mode),
                "no_air":summarize(dry+[mounted,electrical],mode)}
            original = read(HERE/"native.json")["cases"]["high"]["pit_support"]["L"]["bending"]["selected"]
            force=original["one_pin_force_N"]+2*9.80665*camera["stage_carried_kg"][1]
            out["low-place-camera"]["pit_at_2g"]={"one_pin_force_N":force,
                "keeper_bore_nominal_MPa":original["sections"]["keeper_bore"]["nominal_stress_MPa"]*force/original["one_pin_force_N"]}
    write(HERE/"balance.json",out)
