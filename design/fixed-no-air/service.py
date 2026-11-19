"""Fit an unpressurized fixed-height configuration and its removable service supports."""
import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent / "signal-light"))
from inspect_light import PANEL, X, Y, cad, common, configuration, digest, hits, mounted, product, read
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
sys.path.insert(0, str(HERE.parent / "receiver-lift"))
from check_routing import write_step
from duplex import finalize_step

AIR = {"gauge-regulator-dump-reserve", "regulated-pawl-valve", "tank-avt-pp-35",
       "tank-saddle-front", "tank-saddle-rear", "viair-90c"}
LIGHT = HERE.parent / "signal-light"
KEEPER_X = -107.5


def cyl(x, y, z, axis, radius, length):
    return ["cyl", [x, y, z, *axis, radius, length]]


def pin_parts(y, z, tip=-88):
    # Head and shank are one turned blank. A headed cross-bolt and locknut sit in
    # front of the rail, behind the first point of carriage contact at X=-98.
    along, across = (1,0,0), (0,0,1)
    pin = ["union", [cyl(-130,y,z,along,7,5.7), cyl(-124.3,y,z,along,4,tip+124.3)]]
    cross = cyl(KEEPER_X,y,z-8,across,1.15,16)
    bolt = ["union", [cyl(KEEPER_X,y,z-6,across,2,2), cyl(KEEPER_X,y,z-4,across,1,11)]]
    nut = ["cut", [cyl(KEEPER_X,y,z+4.2,across,2.2,2.2), cyl(KEEPER_X,y,z+4,across,1.02,3)]]
    return {k: cad.brep(v) for k,v in {"pin":["cut",[pin,cross]],"keeper":bolt,"nut":nut}.items()}


def distance(a,b):
    query = BRepExtrema_DistShapeShape(a,b)
    if not query.IsDone():
        raise ValueError("Native distance failed")
    return query.Value()


def nearby(parts, boxes, name, tolerance=2):
    a = boxes[name]
    out = {}
    for key, other in parts.items():
        b = boxes[key]
        if key == name or any(a[i] > b[i+3]+tolerance or b[i] > a[i+3]+tolerance for i in range(3)):
            continue
        gap = distance(parts[name], other)
        if gap <= tolerance+1e-6:
            out[key] = gap
    return out


def bending(pin, y, z, tip):
    """One pin carries the full 2g comparison at its most forward exposed point."""
    mass = read(HERE.parent/"fixed-height/review.json")["mountDemand"]["movingMassUpperKgWithAllLockMass"]
    force = 2 * mass * 9.80665
    rows = {}
    for label, x in (("bushing_exit",-112),("keeper_bore",KEEPER_X)):
        thickness = .1
        slab = cad.brep(["box",[x-thickness/2,y-12,z-12,thickness,24,24]])
        section = BRepAlgoAPI_Common(pin,slab)
        if not section.IsDone():
            raise ValueError("Pin section failed")
        properties = GProp_GProps()
        BRepGProp.VolumeProperties_s(section.Shape(),properties)
        area = properties.Mass()/thickness
        inertia = properties.MatrixOfInertia().Value(2,2)/thickness - area*thickness**2/12
        modulus = inertia/4
        # At the head stop the pin is 0.3 mm farther forward relative to the stationary bushing.
        arm = tip-x+(.3 if label=="bushing_exit" else 0)
        rows[label] = {"loaded_arm_mm":arm,"area_mm2":area,"section_modulus_mm3":modulus,
                       "nominal_stress_MPa":force*arm/modulus}
    return {"one_pin_force_N":force,"sections":rows}


def review(mode, lamp, camera=False):
    parts = configuration(mode)
    parts.pop("rsl-reserve")
    deck = cad.read_named(LIGHT / "deck-mount.step", [{"id":n} for n in ("front-electronics-platform","light-spacer")])
    parts.update(deck)
    parts["am-3583"] = lamp
    boxes = {k:cad.bounds(v) for k,v in parts.items()}
    source = read(LIGHT/"review.json")["cases"][mode]
    if len(parts) != source["installed_model_bodies"] or not AIR <= parts.keys():
        raise ValueError("Issued light or air inventory changed")
    interfaces = {k:nearby(parts,boxes,k) for k in sorted(AIR)}
    retained = {k:v for k,v in parts.items() if k not in AIR}
    camera_info = None
    if camera:
        mount = ROOT/"moving-observer"
        readings = read(mount/"mount.json")
        path = mount/"moving-bridge.step"
        relative = cad.read_named(path,[{"id":k} for k in [*readings["parts"],"receiver-R6-drilled-for-camera"]])
        delta = (370 if mode=="low-place" else 0,0,0)
        original = retained.pop("receiver-R6-ported")
        drilled = cad.translated(relative.pop("receiver-R6-drilled-for-camera"),delta)
        loss = cad.volume(original)[0]-cad.volume(drilled)[0]
        if abs(loss-readings["removed_shelf_mm3"]) > .01 or abs(cad.volume(drilled)[0]-common(original,drilled)) > .01:
            raise ValueError("The camera shelf does not replace the issued part in its actual pose")
        retained["receiver-R6-drilled-for-camera"] = drilled
        retained.update({"roof-"+k:cad.translated(v,delta) for k,v in relative.items()})
        camera_info = {"native_sha256":digest(path),"new_bodies":len(relative),"removed_shelf_mm3":loss}
    boxes = {k:cad.bounds(v) for k,v in retained.items()}
    height = 940 if mode=="high" else 560
    service = {}
    study_parts = {}
    for tag, sign in (("L",1),("R",-1)):
        y,z = sign*282,height-165
        local = pin_parts(y,z)
        pin, keeper = local["pin"], local["keeper"]
        plate = retained[f"backup-{tag}-carriage-plate"]
        rail = retained[f"backup-{tag}-service-pin-rail"]
        bush = retained[f"backup-{tag}-service-pin-bushing-{height}"]
        # The union of a translating coaxial head and shank is two longer cylinders.
        sweep = cad.brep(["union",[cyl(-200,y,z,(1,0,0),7,75.7),cyl(-194.3,y,z,(1,0,0),4,106.3)]])
        keeper_sweep = cad.brep(["union",[cyl(KEEPER_X,y,z-26,(0,0,1),2,22),
                                           cyl(KEEPER_X,y,z-24,(0,0,1),1,31)]])
        inserted = {k:hits(v,retained,boxes) for k,v in local.items()}
        # A 2 mm downward shift includes the 1 mm nominal support gap.
        down = cad.translated(plate,[0,0,-2])
        pin_toward_bush = cad.translated(pin,[0,0,-.5])
        short_bounds, plate_bounds = cad.bounds(pin), cad.bounds(plate)
        # Simulate combined assembly and axial clearance rather than assuming a centered part.
        tolerance_pin = cad.translated(pin,[-3,0,0])
        shifted_plate = cad.translated(plate,[1,0,-2])
        tool = cad.brep(cyl(KEEPER_X,y,z+4.2,(0,0,1),4,21))
        service[tag] = {"nominal_hardware_hits_mm3":inserted, "pin_sweep_hits_mm3":hits(sweep,retained,boxes),
            "keeper_sweep_hits_mm3":hits(keeper_sweep,retained,boxes), "pin_bolt_overlap_mm3":common(pin,keeper),
            "keeper_sweep_pin_overlap_mm3":common(keeper_sweep,pin),"nut_tool_hits_mm3":hits(tool,retained,boxes),
            "head_tool_hits_mm3":hits(cad.brep(cyl(KEEPER_X,y,z-27,(0,0,1),4,21)),retained,boxes),
            "prior_tool_hits_mm3":hits(cad.brep(cyl(-109,y+4.2,z,(0,1,0),4,21)),retained,boxes),
            "lateral_tool_hits_mm3":hits(cad.brep(cyl(KEEPER_X,y+4.2,z,(0,1,0),4,21)),retained,boxes),
            "pin_nut_overlap_mm3":common(pin,local["nut"]), "bolt_nut_overlap_mm3":common(keeper,local["nut"]),
            "gap_to_carriage_mm":distance(pin,plate), "minimum_pin_to_bushing_mm":distance(pin,bush),
            "nominal_plate_engagement_mm":short_bounds[3]-plate_bounds[0],
            "adverse_plate_engagement_mm":short_bounds[3]-3-(plate_bounds[0]+1),
            "adverse_bearing_at_2mm_down_mm3":common(tolerance_pin,shifted_plate),
            "bearing_at_2mm_down_mm3":common(pin,down), "seated_shank_at_half_mm_mm3":common(pin_toward_bush,bush),
            "head_at_1mm_forward_mm3":common(cad.translated(pin,[1,0,0]),bush),
            "keeper_at_3mm_withdrawal_mm3":common(cad.translated(keeper,[-3,0,0]),rail)
                                           +common(cad.translated(keeper,[-3,0,0]),bush),
            "wrong_height_carriage_gap_mm":distance(cad.translated(pin,[0,0,380 if height==560 else -380]),plate)}
        if tag=="L" and mode=="high":
            service[tag]["bending"] = {"selected":bending(pin,y,z,-88),
                "long_control":bending(pin_parts(y,z,-66)["pin"],y,z,-66)}
        study_parts.update({f"pit-{tag}-{n}":s for n,s in local.items()})
    if mode == "low-receive" and not camera:
        path = HERE/"pit-pins.step"
        write_step(path,study_parts)
        finalize_step(path)
        checked = cad.read_named(path,[{"id":k} for k in study_parts])
        for k,v in checked.items():
            if abs(cad.volume(v)[0]-cad.volume(study_parts[k])[0])>.001:
                raise ValueError("Service hardware changed on STEP reopen")
    return {"match_bodies":len(retained),"camera_kit":camera_info,
            "air_installed_neighbours_within_2mm":interfaces,"pit_support":service}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("signal_light_step", type=Path)
    args = parser.parse_args()
    lamp = mounted(product(args.signal_light_step),X,Y,PANEL)
    cases = {mode:review(mode,lamp) for mode in ("high","low-receive","low-place")}
    cases.update({mode+"-camera":review(mode,lamp,True) for mode in ("low-receive","low-place")})
    sources = [LIGHT/"review.json", LIGHT/"deck-mount.step", HERE.parent/"fixed-height/native.json",
               HERE.parent/"receiver-lift/duplex.py",ROOT/"moving-observer/mount.json",ROOT/"moving-observer/moving-bridge.step"]
    output = {"sources":{p.relative_to(ROOT).as_posix():digest(p) for p in sources},
              "pit_step_sha256":digest(HERE/"pit-pins.step"),"cases":cases}
    (HERE/"native.json").write_bytes((json.dumps(output,indent=2)+"\n").encode())


if __name__ == "__main__":
    main()
