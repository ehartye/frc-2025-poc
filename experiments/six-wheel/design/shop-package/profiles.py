"""Finished millimetres. U,V start at the lower left on the indicated face."""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Operation:
    label: str
    u: float
    v: float
    diameter: float
    travel: float = 0
    depth: float = 0


@dataclass
class Part:
    key: str
    title: str
    size: tuple
    ids: list
    view: str
    details: list
    holes: list = field(default_factory=list)
    wall: float = 0
    radius: float = 0


def profiles():
    out=[]
    for kind,length,thickness in [("outer",700,25),("inner",660,6)]:
        rail=Part(kind+"-rail",kind.capitalize()+" drive rail",(length,100,thickness),
                  ["drive-rail-"+kind+"-"+s for s in "LR"],"XZ",[
                      "A = finished broad face. B = rear end. C = lower edge. U increases forward, V upward.",
                      "Mate left and right axle patterns in one datum setup. Nominal axle height above lower edge 26.20.",
                      "End slots are horizontal. Travel 8.00 is center-to-center; overall equals travel + diameter."])
        if kind=="outer":
            rail.wall=2
            rail.details.append("Rectangular tube 100 x 25 x 2; machine BOTH walls. Bumper sleeve holes pass both walls.")
        else:
            rail.details.append("Solid 6 plate. The low gearbox access at V0.80 opens into the bottom edge by 4.40.")
        for shift in [-247.65,0,247.65]:
            for dx,dv,diam in [(0,0,32.4),(-19,-17,5.4),(-19,17,5.4),(19,-17,5.4),(19,17,5.4)]:
                rail.holes.append(Operation("Carrier opening" if dx==0 else "Carrier clamp",length/2+shift+dx,26.2+dv,diam,8 if shift else 0))
        if kind=="outer":
            rail.holes.extend(Operation("Bumper sleeve",length/2+dx,77,12) for dx in [-125,125])
        else:
            rail.holes.extend(Operation("Gear head access",length/2,26.2+dv,10.4) for dv in [-25.4,25.4])
            rail.holes.extend(Operation("Adapter clamp",length/2+dx,26.2+dv,5.4) for dx in [-40,40] for dv in [-12,12])
        out.append(rail)
    out.append(Part("end-tube","Front and rear bumper cross tubes",(650,100,20),["cross-front","cross-rear"],"YZ",
                    ["A = broad face. B = robot-right end. C = bottom. U runs right to left and V upward.",
                     "20 x 100 rectangular tube, 2 wall. Drill BOTH walls. Match the welded sleeves to actual holes.",
                     "Check all four frame joints and squareness before welding. Fixed outer sleeve face flush with rail."],
                    [Operation("Bumper sleeve",u,77,12) for u in [150,500]],wall=2))
    fixed=[Operation("Bearing seat",28,26,28.62)]
    fixed.extend(Operation("Carrier clamp",28+du,26+dv,5.3) for du in [-19,19] for dv in [-17,17])
    for special in [False,True]:
        ids=[f"carrier-{where}-{side}-{lr}" for where in ["front","center","rear"] for side in ["inner","outer"] for lr in "LR"
             if (where=="center" and side=="inner")==special]
        out.append(Part("center-inner-carrier" if special else "standard-carrier",
                        "Center inner bearing carrier" if special else "Bearing carrier",(56,52,6),ids,"XZ",
                        ["A = broad face. B = rear edge. C = bottom. Pair opposing seats after stock and bearings arrive.",
                         "28.62 is CAD nominal. Actual fit, retention and OD acceptance require the bearing lot drawing.",
                         "Center-inner reliefs break open at both edges." if special else "Ten interchangeable geometries; fixed and adjustable installations have different mating hardware."],
                        fixed+([Operation("Head relief",28,26+dv,10.4) for dv in [-25.4,25.4]] if special else [])))
    out.append(Part("gearbox-adapter","Gearbox face adapter",(96,66,5),["gearbox-faceplate-L","gearbox-faceplate-R"],"XZ",
                    ["A = gearbox mating face. B = rear edge. C = lower edge. No thread is implied by a clearance hole.",
                     "Two gearbox fasteners are #10-32. Four outer holes are the M5 adapter-to-rail pattern.",
                     "Inner-center carrier clamps have no matching relief here. See the joint disposition before releasing hardware."],
                    [Operation("Drive opening",48,33,36)]+[Operation("Gear screws",48,33+dv,5.3) for dv in [-25.4,25.4]]
                      +[Operation("Adapter clamp",48+du,33+dv,5.4) for du in [-40,40] for dv in [-12,12]]))
    out.append(Part("r6-shelf","Ported receiver shelf",(265,420,15),["receiver-R6-ported"],"XY",
                    ["A = underside. B = square rear end. C = robot-right edge. U is forward; V runs right to left.",
                     "Machine continuous R6 only on the upper forward edge (U265); tangencies U259 at top and W9 at front.",
                     "Counterbores open from the TOP. Two optical ports pass through on the centerline. Keep lip burr-free.",
                     "Candidate PE grade and contact finish require coupon checks. See receiver fastener stack."],
                    [Operation("Optical through",u,210,12) for u in [140,200]]
                    +[Operation("Stack through",u,v,5.3) for u in [20,240] for v in [30,390]]
                    +[Operation("Top counterbore",u,v,9.2,depth=3) for u in [20,240] for v in [30,390]],radius=6))
    out.append(Part("shelf-spacer","Shelf mounting spacer",(250,20,6),["shelf-spacer-L","shelf-spacer-R"],"XY",
                    ["A = lower face. B = rear end. C = right edge. Holes pass fully through.",
                     "Make and inspect against the actual shelf datum and anchor. Both locations use the same cut."],
                    [Operation("M5 clearance",u,10,5.3) for u in [15,235]]))
    out.append(Part("anchor-block","Shelf anchor as modeled",(16,14,7),
                    [f"anchor-block-{s}-{x}" for s in "LR" for x in [-80,140]],"XY",
                    ["HOLD THREAD AND RETENTION. The current 5.30 bore is clearance; an M5 screw cannot tighten into it.",
                     "This sheet identifies the failed load-path detail. See the receiver stack for a proposed replacement."],
                    [Operation("Existing clearance",8,7,5.3)]))
    return out
