"""Electrical, geometry, inspection and interlock regressions, all offline."""
from dataclasses import replace
from pathlib import Path
import json
import sys
import unittest
ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/"design"/"electrical-inspection"
sys.path.insert(0,str(HERE))
import replay
import inspection
import pawl
from lift_interlock import Inputs,update

class ElectricalReview(unittest.TestCase):
    def test_circuit_and_failure_controls(self):
        replay.validate(replay.CIRCUITS)
        for override in [{"channel":1},{"awg":18},{"can":12}]:
            broken=[dict(x) for x in replay.CIRCUITS]
            broken[0].update(override)
            with self.assertRaises(ValueError): replay.validate(broken)
        review=json.loads((HERE/"power.json").read_text())
        new=replay.round_values(replay.replay(mass=67,turn_accel=.64))
        self.assertEqual(new,review["upperWithSlowerTurns"])
        self.assertTrue(new["minimumDeviceV"]>=review["floorV"])
        self.assertGreater(review["upperAtRegisteredTiming"]["drivePeakPhaseA"],38)
        self.assertLess(new["loadEstimateByBranch"]["DriveRight"]["maxPhaseA"],38)
        self.assertLess(review["weakenedSource"]["minDeviceV"],9.5)
        self.assertIn("collapse",review["collapsedCase"]["observation"])
        self.assertAlmostEqual(review["nominal"]["durationS"],replay.MISSION["elapsed_s"],places=4)

    def test_inspection_distinguishes_unverified_as_built(self):
        m=inspection.study()
        self.assertIsNone(m["actualWeighedKg"])
        self.assertGreater(m["seatedPipeRangeMm"]["seat"],30)
        self.assertLess(m["seatedPipeRangeMm"]["seat"],80)
        self.assertLess(m["maximum"]["R103kg"],115*.45359237)
        self.assertGreater(m["maximum"]["emptyInstalledKg"]+.816466266,62)
        p=pawl.screen()
        self.assertFalse(p["physicalPowerLossCatchVerified"])
        self.assertGreater(p["cases"][2]["meanDrumArrestNmAt1mm"],7*p["staticProofDrumNm"])
        geom=json.loads((HERE/"packaging.json").read_text())
        self.assertEqual(set(geom["poses"]),{"stowed","receive","transport","place"})
        for pose in geom["poses"].values():
            self.assertFalse(pose["collisionMm3"])
            self.assertFalse(any(pose["serviceMm3"].values()))

    def test_never_lower_on_single_command(self):
        good=Inputs(True,True,True,True,False,True,False,True,True)
        self.assertEqual(update("locked",good),("supporting",True,False,False))
        self.assertEqual(update("supporting",good),("supporting",True,False,False))
        ok=replace(good,support_observed=True)
        self.assertEqual(update("supporting",ok),("releasing",True,True,False))
        free=replace(ok,engaged=False,released=True)
        self.assertEqual(update("releasing",free),("moving",True,True,True))
        self.assertEqual(update("moving",replace(free,request_move=False)),("stopping",True,True,False))
        self.assertEqual(update("stopping",replace(free,request_move=False)),("engaging",True,False,False))
        self.assertEqual(update("engaging",ok),("locked",False,False,False))
        for change in [dict(enabled=False),dict(fresh=False),dict(limits_ok=False),dict(air_ok=False),
                       dict(engaged=True),dict(timed_out=True)]:
            state,support,valve,motion=update("moving",replace(free,**change))
            self.assertEqual((state,valve,motion),("fault",False,False))
        self.assertEqual(update("fault",good),("fault",False,False,False))
