"""Estimate mill-access stock left by a 6 mm bull-nose cutter on the underside."""
import json
import math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent


def stock_depth(record, pitch, left=-352, right=352):
    # A uniform grid includes overtravel outside the finished perimeter.
    x = np.arange(left + pitch/2, right, pitch)[:,None]
    y = np.arange(-264 + pitch/2, 264, pitch)[None,:]
    inside = (np.abs(x)<346) & (np.abs(y)<258)
    lands = (np.abs(x)>=321) | (np.abs(y)>=212)
    for left,right,low,high in record["upperBearingLands"].values():
        lands |= (x>=left) & (x<=right) & (y>=low) & (y<=high)
    material = np.where(inside,np.where(lands,4.,record["skinMm"]),0.).astype(np.float32)
    profile = np.array(record["sectionDepthFromTopMm"])
    varying = np.interp(np.abs(y),profile[:,0],profile[:,1]).astype(np.float32)
    for station in record["stationsMm"]:
        row=(np.abs(x[:,0]-station) < record["webMm"]/2)
        material[row] = np.maximum(material[row],np.where(np.abs(y)<=226,varying,0))
    return material,inside


def disk_extreme(values, radius, pitch, get_max):
    # Scan horizontal chords of the disk. A sliding-window 1D reduction
    # is cached per chord width and reused by both halves of the circle.
    from numpy.lib.stride_tricks import sliding_window_view
    combine = np.maximum if get_max else np.minimum
    init = -np.inf if get_max else np.inf
    n = int(radius/pitch)
    buffer = np.pad(values,((n,n),(0,0)),mode="edge")
    result = np.full_like(values,init)
    previous = -1
    for dy in range(n,-1,-1):
        dx=int(math.sqrt(max(0,radius**2-(dy*pitch)**2))/pitch)
        if dx!=previous:
            chord=buffer[n-dx:n+len(values)+dx]
            windows=sliding_window_view(chord,2*dx+1,axis=0)
            extreme = windows.max(axis=-1) if get_max else windows.min(axis=-1)
            previous=dx
        if dy:
            combine(result[:,dy:],extreme[:,:-dy],out=result[:,dy:])
            combine(result[:,:-dy],extreme[:,dy:],out=result[:,:-dy])
        else:
            combine(result,extreme,out=result)
    return result


def tile(record,pitch,left,right,reserve):
    depth,inside=stock_depth(record,pitch,left-8,right+8)
    # A grid-center command must clear the whole 3 mm radius. Only the
    # inner 2 mm radius is flat. The outer annulus can remove material up
    # to 1 mm less deeply. Placement reserve encloses a half-cell diagonal.
    stop=disk_extreme(depth,3.+reserve,pitch,True)
    core=disk_extreme(stop,2.-reserve,pitch,False)
    rim=disk_extreme(stop,3.-reserve,pitch,False)+1.
    extra=np.maximum(0,np.minimum(core,rim)-depth)
    extra[~inside]=0
    halo=round(8/pitch)
    return (float(depth[halo:-halo].sum(dtype=np.float64)*pitch**2),
            float(extra[halo:-halo].sum(dtype=np.float64)*pitch**2),float(extra[halo:-halo].max()))


def screen(record,pitch,reserved=True):
    reserve=pitch/math.sqrt(2) if reserved else 0.
    tiles=[tile(record,pitch,x,x+44,reserve) for x in range(-352,352,44)]
    return {"pitchMm":pitch,"placementReserveMm":reserve,"gridStockBeforeTieBoresMm3":sum(v[0] for v in tiles),
            "uncutStockMm3":sum(v[1] for v in tiles),"peakUncutMm":max(v[2] for v in tiles)}


def run():
    data=json.loads((HERE/"geometry.json").read_bytes())
    rows=[screen(data,pitch) for pitch in (.5,.25,.125,.0625)]
    rows.append(screen(data,.25,False))
    out={"geometrySha256":data["stepSha256"],"cutterDiameterMm":6,"cutterCornerRadiusMm":1,
         "finishStockVolumeReserveMm3":40000,"trials":rows,"actualMachinedVolumeMm3":None}
    (HERE/"machining.json").write_text(json.dumps(out,indent=2)+"\n",encoding="utf8",newline="\n")
    print(rows)


if __name__=="__main__":
    run()
