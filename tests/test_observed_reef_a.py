"""Route, material and sensor evidence on the two powered station departures."""
import json
from pathlib import Path
import unittest

HERE=Path(__file__).resolve().parents[1]/"observed-reef-a"

def read(stem):
    return json.loads((HERE/(stem+".json")).read_bytes())

class ObservedReefApproach(unittest.TestCase):
    def test_two_independent_heads_and_known_false_survey(self):
        old=read("field-trials")
        machined=read("machined-field-trials")
        self.assertEqual(len(machined),12)
        for source,row in zip(old,machined):
            self.assertEqual((source["wheelCase"],source["mode"],source["motionSHA256"]),
                             (row["wheelCase"],row["mode"],row["motionSHA256"]))
            mode=row["mode"]
            self.assertEqual(row["accepted"],mode in ("nominal","shared-survey-shift"))
            if mode=="nominal":
                self.assertGreater(row["raysUsed"]["3"],0)
                self.assertEqual(row["raysUsed"]["4"],0)
                self.assertLess(row["latestInterval"][0],10)
                self.assertLess(row["latestInterval"][1],1)
            if mode=="shared-survey-shift":
                self.assertGreater(abs(row["finalBodyErrorMmDeg"][1]),10)

    def test_station_to_roof_calibration_bound(self):
        for state in read("station-bridge").values():
            self.assertEqual(len(state["fixedMarked"]),5)
            self.assertTrue(all(r[-1] for r in state["fixedMarked"]))
            self.assertTrue(all(r[-1] for r in state["roof"]["roof"]))
            for perturbation in ("roof-050","roof-same"):
                self.assertFalse(any(r[-1] for r in state["roof"][perturbation]))
        faults=read("machined-roof-050-quantized-cargo-nominal-active")
        self.assertTrue(all(r["firstFault"]==[5475,"missing-end-arc"] for r in faults))

    def test_timestep_and_unique_pipe_identity(self):
        for coarse,fine in zip(read("cargo-nominal-active"),read("cargo-nominal-active-fine")):
            self.assertEqual(coarse["sourceFeedSha256"],fine["sourceFeedSha256"])
            self.assertNotEqual(coarse["trajectorySha256"],fine["trajectorySha256"])
            for row in (coarse,fine):
                self.assertEqual(row["piece"],"coral-A")
                self.assertTrue(row["plannedEndpoint"])
                self.assertIsNone(row["physicalStop"])
                self.assertEqual(row["roofFrames"],1244)
                self.assertGreater(row["minGuideGapMm"],10)
                self.assertAlmostEqual(row["finalWallS"],48.16217,places=4)

    def test_actuator_limits_and_jam_stop(self):
        result=read("effort")
        self.assertLess(result["driveAt006Mps"]["2.4"]["phase_A"],8)
        self.assertGreater(result["driveAt006Mps"]["26.25"]["phase_A"],8)
        stops=read("cargo-stops")
        self.assertEqual({r["case"] for r in stops},{"leftSmall","rightSmall"})
        for row in stops:
            self.assertLessEqual(row["inhibitWallS"],row["braking"]["firstServiceWallS"])
            self.assertLess(row["braking"]["stoppedWallS"]-row["inhibitWallS"],1.2)
            self.assertGreater(row["clearance"]["minimumPeerMm"],0)
            self.assertIsNone(row["cameraDecisionWallS"])

    def test_effective_pipe_is_not_released_during_jam_braking(self):
        rows=read("jam-settle")
        for state in rows:
            self.assertEqual(state["piece"],"coral-A")
            self.assertIsNone(state["negativeGuideOrCartridgeStop"])
            self.assertIsNone(state["lostPipe"])
            self.assertGreater(state["minGuideGapMm"],10.)
            self.assertAlmostEqual(state["lastWallS"]-state["wheelsStoppedWallS"],.25,places=5)
            self.assertFalse(state["independentPostTripOpticalAuthorization"])

    def test_reopened_stowed_robot_against_ball(self):
        for source,count in (("stock",993),("machined-stock",1000)):
            for side in read(source).values():
                self.assertEqual(side["registeredBodyCount"],count)
                self.assertEqual(side["includedCableBounds"],629)
                self.assertEqual(side["stockId"],"NAUO201")
                self.assertGreater(side["minBoundWith10Mm1DegMm"],135)
                self.assertGreater(side["peerLowerWith10Mm1DegMm"],1300)

    def test_powered_route_exposes_pack_and_traction_assumptions(self):
        for row in read("field"):
            for pack in row["power"].values():
                self.assertGreater(pack["minTerminalV"],9.5)
                self.assertLess(pack["peakMotorPhaseA"],38)
                self.assertGreater(pack["peakFrictionRequired"],0)
            self.assertIsNone(row["inhibitWallS"])
