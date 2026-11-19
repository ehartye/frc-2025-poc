"""Check selected branch articles against the model and bound accepted camera/host terminal tolerances."""
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent


def check():
    data=json.loads((HERE/"results.json").read_bytes())["cases"]
    inventory=json.loads((ROOT/"design/electrical-inspection/circuits.json").read_bytes())["branches"]
    seat=json.loads((ROOT/"fixed-height-match/placements.json").read_bytes())["placements"][0]["seat"]
    candidate=data["park372-fixed"]
    circuits={}
    for row in inventory:
        k=row["load"]
        a=candidate["branches"][k]["peakBranchA"]
        rating=row["ratingA"]
        atm=row["protection"]=="ATM blade fuse"
        sku="Eaton ATM-10" if atm else "REV-11-"+str({10:1860,20:1861,30:1862,40:1863}[rating])
        baseline=1.1 if atm else 1.
        circuits[k]={"channel":row["channel"],"articleCandidate":sku,"A":rating,"maximumModeledBranchA":a,
                     "belowPublishedHoldCurrent":a<=rating*baseline,"publishedMinimumHoldS":360000 if atm else 1800,
                     "twiceRatedTripWindowS":[.15,5.] if atm else [5.,60.]}
    # No current, heatsink temperature, socket condition or USB drop has been measured.
    # R_loop is the entire 5 V supply/return path including connectors and Y adapter.
    rails={}
    for name,amps in (("host",3.),("camera",1.2)):
        resistance=.025
        line_uncertainty=.015
        selected_low,selected_high=5.12,5.18
        rails[name]={"worstOperatingA":amps,"catalogOutputRangeV":[4.8,5.2],"proposedSelectedOutputRangeV":[selected_low,selected_high],
           "maxCompleteOutputLoopOhm":resistance,"extraVoltageUncertaintyV":line_uncertainty,
           "catalogWorstTerminalV":4.8-amps*resistance-line_uncertainty,
           "selectedTerminalRangeV":[selected_low-amps*resistance-line_uncertainty,selected_high+line_uncertainty],
           "directMeasurementRequired":True}
    return {"branches":circuits,"main":{"article":"Eaton CB185-120","ratedA":120,"modeledPeakA":candidate["peakBusA"],
           "measuredTripTime":None},"powerChain":{"battery":"EnerSys NP18-12 bolt-terminal proposal","upstreamCopperAWG":6,"connector":"Anderson SB50",
           "measuredOpenCircuitV":None,"measuredPackAndTrunkOhm":None},"observations":rails,
           "cameraCriterionV":[5.,5.25],"thermalLimits":{"caseTemperatureC":None,"windingTemperatureC":None,"continuousDriveLimitQualified":False},
           "holdJam":{"eachRequestedGripPhaseA":8,"modeledLeftWindingJ":data["stalledGrip-fixed"]["branches"]["LeftGrip"]["windingJ"],
           "modeledPeakLeftSupplyA":data["stalledGrip-fixed"]["branches"]["LeftGrip"]["peakBranchA"],
           "stalledIntervalS":135-seat,"protectionRatingA":20}}


if __name__=="__main__":
    (HERE/"protection.json").write_text(json.dumps(check(),indent=2)+"\n",encoding="utf-8",newline="\n")
