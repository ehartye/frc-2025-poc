"""Couple the successor drive to the signed route and four entire-robot exports."""
import hashlib
import json
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
DESIGN=ROOT/"design"/"drivetrain"
sys.path.insert(0,str(DESIGN))
sys.path.insert(0,str(ROOT))
from route_check import assess,ROUTE,DEFAULT
from scripts.registered_transit import routes


class DriveEngineering(unittest.TestCase):
    def test_real_route_and_source(self):
        for path,key in [(ROOT/"scripts"/"registered_transit.py","sourceSha256"),
                         (ROOT/"mission"/"field-registration.json","registrationSha256")]:
            self.assertEqual(hashlib.sha256(path.read_bytes().replace(b"\r\n",b"\n")).hexdigest(),ROUTE[key])
        _,loaded,_=routes(ROUTE["reefSectorM"])
        self.assertEqual(json.loads(json.dumps(loaded.segments)),ROUTE["segments"])
        result=assess()
        self.assertTrue(result["parameterCaseClears"])
        self.assertAlmostEqual(result["routeSeconds"],loaded.duration)
        self.assertAlmostEqual(result["straightTravelM"],8.948798139066927)
        self.assertEqual(result,json.loads((DESIGN/"route-receipt.json").read_text())["candidate"])
        for key,p in [("friction",{"longitudinalFriction":.45}),("batteryV",{"openCircuitV":10.9,"wiringAndBatteryOhm":.04}),
                      ("phaseA",{"ratio":12}),("phaseA",{"scrubCoefficient":.8})]:
            self.assertLess(assess(p)["margins"][key],0)

    def test_solid_inventory_and_powertrain_stack(self):
        from drive_parts import parts,LANES,CENTERS
        from build_assembly import read_named,bounds
        inventory=parts()
        review=DESIGN/"review"
        data=json.loads((review/"review.json").read_text())
        for name,pose in data["poses"].items():
            path=review/(name+".step")
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),pose["stepSha256"])
            self.assertEqual(pose["shapeCount"],len(inventory))
            self.assertEqual(pose["interferencesOver0.001mm3"],[])
            self.assertLess(sum([pose["allAccountedMassKg"][1],pose["maxPayloadAtSeatKg"]]),DEFAULT["massKg"])
        solids=read_named(review/"transport.step",inventory)
        def extent(k):
            return bounds(solids[k])
        a,b=(extent(f"chain-{i}-L") for i in range(2))
        self.assertAlmostEqual(a[4]-a[1],9.1948,places=4)
        self.assertGreater(a[4]-a[1],5)
        self.assertAlmostEqual(b[1]-a[4],LANES[1]-LANES[0]-9.1948,places=4)
        self.assertGreater(extent("wheel-center-L")[1]-b[4],4)
        self.assertGreater(a[1]-extent("bearing-center-inner-L")[4],4)
        self.assertAlmostEqual(extent("gearbox-L")[4]-extent("axle-center-L")[1],10,places=4)
        self.assertAlmostEqual(extent("wheel-front-L")[0]-extent("wheel-center-L")[0],CENTERS,places=4)
