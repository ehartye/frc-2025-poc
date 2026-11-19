"""Continuous positioning proofs for the 545-body field robot between its parked poses."""
from functools import cache
from pathlib import Path
from assembly import HERE,cad,original,pose_layers
from build_successor import write_json
from mechanical import box,cylinder
import travel


def difference(a,b):
    overlap=cad.intersection(a,b)
    return max(abs(cad.volume(shape)[0]-overlap) for shape in (a,b))


@cache
def at_height(h):
    return original.flatten(pose_layers(h,0)[0])


@cache
def verify_line(name,low,high):
    # Source paths are straight cylinders. Positively identify radius, fixed XY,
    # fixed upper endpoint and affine lower endpoint at all three registered heights.
    initial=cad.bounds(at_height(low)[name])
    x=(initial[0]+initial[3])/2
    y=(initial[1]+initial[4])/2
    radius=(initial[3]-initial[0])/2
    for h in (low,670,high):
        lower=initial[2]+h-low
        ideal=cad.brep(cylinder(x,y,lower,[0,0,1],radius,initial[5]-lower))
        if difference(at_height(h)[name],ideal)>.001:
            raise ValueError("Unqualified changing rope geometry "+name)
    return True


@cache
def verify_rigid(name,low,high,dz):
    start=at_height(low)[name]
    for h in (670,high):
        translated=cad.translated(start,(0,0,dz*(h-low)/(high-low)))
        if difference(translated,at_height(h)[name])>.001:
            raise ValueError("Reference frame mismatch "+name)
    return True


def supplemental(a,b,shapes,frames,start,end):
    low,high=start[0],end[0]
    if low>=high or start[1]!=0 or end[1]!=0:return None
    ropes=[k for k in (a,b) if frames[k]=="variable-rope"]
    if ropes:
        for name in ropes:verify_line(name,low,high)
        for name in set((a,b))-set(ropes):
            offset=0 if frames[name]=="base" else high-low
            verify_rigid(name,low,high,offset)
        # In a base frame the cylinder shortens from below; in a carriage frame
        # it shortens from above. Its low-pose cylinder contains every intermediate
        # occupation in either frame. Two shrinking cylinders also stay subsets.
        return cad.intersection(shapes[a],shapes[b])
    masts=[k for k in (a,b) if k.endswith("drilled-mast")]
    if len(masts)==1:
        mast=masts[0]
        other=b if a==mast else a
        if frames[other]=="base":return None
        verify_rigid(mast,low,high,0)
        verify_rigid(other,low,high,high-low)
        # In the carriage frame the full fixed mast travels down. Fill its holes
        # and hollow interior, extend the resulting box over that entire leg.
        x,y,z,xx,yy,zz=cad.bounds(shapes[mast])
        swept=cad.brep(box(x,y,z-(high-low),xx-x,yy-y,zz-z+high-low))
        bound=cad.intersection(swept,shapes[other])
        if bound<=.001:return bound
    return None


def check():
    result=travel.screen(pose_layers,supplemental)
    for leg,row in result.items():
        positives={k:v for k,v in row["exact_swept_common_mm3"].items() if v>travel.VOLUME_TOLERANCE}
        if positives or row["unresolved"] or row["reference_conflicts_mm3"]:
            raise ValueError((leg,positives,row["unresolved"],row["reference_conflicts_mm3"]))
    return result


if __name__=="__main__":
    write_json(HERE/"travel.json",check())
