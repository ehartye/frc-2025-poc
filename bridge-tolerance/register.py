"""Bind source, assembly, observation registers and the final desktop receipts."""
import hashlib
import json
import math
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
def read(path):return json.loads(path.read_bytes())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def run():
    old=read(ROOT/"moving-observer/mount.json")
    new=read(HERE/"mount.json")
    if old["parts"].keys()!=new["parts"].keys():raise ValueError("Unaccounted bridge article")
    max_delta=0.
    for k,p in old["parts"].items():
        if abs(new["parts"][k]["kg"]-p["kg"])>1e-8:raise ValueError("Mass changed: "+k)
        for i,(a,b) in enumerate(zip(p["bounds_mm"],new["parts"][k]["bounds_mm"])):
            max_delta=max(max_delta,abs(b-a-(-50 if i in (0,3) else 0)))
    if max_delta>.001 or abs(old["removed_shelf_mm3"]-new["removed_shelf_mm3"])>.001:
        raise ValueError("Native interchange differs")
    initial=read(ROOT/"moving-frame-timing/results.json")
    aft=read(HERE/"timing.json")
    comparisons={}
    for mode,delays in initial["profiles"].items():
        changes={d:{"issued":p,"aft":aft["profiles"][mode][d]} for d,p in delays.items() if p!=aft["profiles"][mode][d]}
        comparisons[mode]=changes
    report={"source_sha256":{p:sha(ROOT/p) for p in (
        "moving-observer/mount.json","moving-observer/moving-bridge.step","moving-observer/checks.json","moving-observer/optics.py",
        "moving-observer/replay.py","moving-observer/MovingWitnessTest.java","fixed-height-match/native.json",
        "design/fixed-no-air/whole-robot.json","design/fixed-no-air/inspection.json","moving-frame-timing/results.json",
        "moving-frame-timing/PacketClockTest.java","moving-frame-timing/OpticalWindow.java","moving-frame-timing/verify.py",
        "low-observation/results.json","full-match/installed-stock.json","leave-park/study.json")},
        "candidate_sha256":{p:sha(HERE/p) for p in ("design.py","field.py","replay_candidate.py","bridge_fit.py","calibration.py",
            "moving-bridge.step","field.json","replay.json","travel-hal.json","timing.json","calibration.json","mount.json","contexts.csv","frames.csv","exposures.csv")},
        "unchanged_article_count":len(old["parts"]),"largest_native_translation_error_mm":max_delta,
        "unchanged_removed_shelf_mm3":new["removed_shelf_mm3"],"credible_fit_on_existing_bored_shelf":False,
        "neighbor_bore_common_mm3_per_pair":15*(2*2.25**2*math.acos(3/4.5)-1.5*math.sqrt(4*2.25**2-9)),
        "low_camera_mass":read(ROOT/"design/fixed-no-air/inspection.json")["cases"]["low-camera-red"],
        "hal_all_phase_ticks_and_results_identical":initial["hal"]==aft["hal"],
        "changed_offset_sweep_counts":comparisons}
    (HERE/"register.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":run()
