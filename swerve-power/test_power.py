import unittest
import json
from pathlib import Path
import numpy as np
from electrical import IDS, branches, source, DRIVE_SUPPLY_R, demand
from loads import G, Support, corner_loads, load_bound
from balance import centers
from corners import limits
from route_grip import accepted_normals


class SwervePowerTests(unittest.TestCase):
    def test_registered_public_configuration(self):
        folder = Path(__file__).resolve().parent

        def read(name):
            return json.loads((folder / name).read_bytes())

        clocks = read("clocks.json")["traces"]
        rows = read("routes.json")["traces"]
        for key in ("positive", "negative", "heavyPositive", "heavyNegative"):
            self.assertEqual(rows[key]["clockSha256"], clocks[key]["sha256"])
            self.assertIsNone(rows[key]["support"]["firstUncovered"])
            self.assertEqual(rows[key]["support"]["samples"], 4220)
            self.assertAlmostEqual(
                rows[key]["support"]["plantMassFromFourNormalsKg"],
                72.0 if key.startswith("heavy") else 65.0,
            )
            for pack in rows[key]["electrical"]:
                self.assertIsNone(pack["actualFirstBlockedWallS"])
                self.assertTrue(
                    all(case["firstInhibitWallS"] is None for case in pack["inverseDemand"])
                )
        self.assertGreater(rows["abrupt"]["consistentAdverse"]["requiredMu"], 0.8)
        geometry = read("controller-fit.json")
        self.assertGreater(geometry["initialRearRightAtMinus141"]["housingMm3"], 150)
        for row in geometry["corners"].values():
            for item in row["fit"].values():
                self.assertTrue(all(c[1] < 1e-4 for c in item["near"].values()))
        wires = read("paths.json")
        self.assertEqual(len(wires["motorAndSensorTargets"]), 8)
        self.assertTrue(
            all(
                v["lowerGapMm"] > 2
                for k in ("groupedSupplyTargets", "motorAndSensorTargets")
                for v in wires[k].values()
            )
        )
        bill = read("installed.json")
        self.assertEqual(
            bill["sourcePhysicalIdentities"]
            + bill["newPhysicalCount"]
            - bill["removedPhysicalCount"],
            bill["installedPhysicalCount"],
        )
        self.assertLess(bill["matched"]["shortR103MarginKg"], 0)
        self.assertIsNone(bill["matched"]["physicalRobotKg"])

    def test_four_independent_pairs_and_isolated_loads(self):
        b = branches()
        self.assertEqual([b[k + "Drive"]["channel"] for k in IDS], [0, 1, 10, 11])
        self.assertEqual([b[k + "Steer"]["can"] for k in IDS], [19, 20, 21, 22])
        self.assertEqual(
            [b[k]["channel"] for k in ("Host", "FixedCamera", "MovingCamera")], [6, 8, 9]
        )
        self.assertEqual(b["Lift"]["channel"], 2)
        self.assertEqual(b["Lift"]["can"], 13)
        self.assertAlmostEqual(DRIVE_SUPPLY_R, 0.003 + 2.5 * (1.55 * 0.003277 + 0.25 * 0.005211))

    def test_stable_bus_root_and_impossible_supply(self):
        names, supply, term, amps = source(
            {"FLDrive": np.array([100.0, 5000.0]), "Host": np.array([20.0, 20.0])}, (11.8, 0.027)
        )
        self.assertAlmostEqual(supply[0] + 0.027 * sum(amps[0]), 11.8)
        for i, k in enumerate(names):
            self.assertAlmostEqual(term[0, i] + branches()[k]["ohm"] * amps[0, i], supply[0])
        self.assertTrue(np.isnan(supply[1]))
        self.assertTrue(np.isnan(term[1]).all())

    def test_signed_coupling_and_supply_are_separate(self):
        vals = np.array([[1.0, 2.0, -2.0, -1.0]])
        d, s = demand(vals, vals, 0.2 * vals, vals, 2 * vals, 0.3 * vals)
        mirrored = demand(-vals, -vals, -0.2 * vals, -vals, -2 * vals, -0.3 * vals)
        for a, b in zip((d, s), mirrored):
            np.testing.assert_allclose(a[0], -b[0])
            np.testing.assert_allclose(a[2], b[2])
        d2, _ = demand(
            vals, vals, 0.2 * vals, vals, 2 * vals, 0.3 * vals, drive_rotor_accel=200 * vals
        )
        self.assertGreater(np.max(abs(d2[0] - d[0])), 0.0)

    def test_four_contact_transfer_and_tripod(self):
        xy = centers()
        rates = 150000 * np.array([0.9, 0.9, 1.1, 1.1])
        m = 65.0
        law = Support(xy, m, [0.25] * 4, 0.31, rates)
        n = law.forces(-1.06625676, -3.46374944, 0.0, 0.0)
        self.assertAlmostEqual(sum(n), m * G)
        self.assertLess(n[-1] * 0.8, 59.37)
        q = np.vstack((np.ones(4), xy.T)) @ n
        np.testing.assert_allclose(q, [m * G, m * 0.31 * 1.06625676, m * 0.31 * 3.46374944])
        test = corner_loads(
            xy, m, [0.0, 0.0, 0.31], [2.0, 1.0], 0, 0, rates, [0.001, -0.001, -0.001, 0.001]
        )
        self.assertAlmostEqual(min(test), 0.0)
        self.assertAlmostEqual(sum(test), m * G)
        with self.assertRaises(ValueError):
            corner_loads(xy, m, [0, 0, 0.4], [20, 20], 0, 0, rates, [0] * 4)

    def test_signed_bound_encloses_a_witness(self):
        xy = centers()
        m = 65.0
        c = [[-0.05, 0.05], [-0.04, 0.04], [0.25, 0.38]]
        lo, hi = load_bound(xy, m, c, [0.5, 1.0], 0.6, 1.0)
        p = corner_loads(
            xy,
            m,
            [0.012, -0.008, 0.31],
            [0.5, 1.0],
            0.6,
            1.0,
            150000 * np.array([0.9, 1.05, 1.0, 1.1]),
            [0.0001, -0.00005, 0.00012, 0.0],
        )
        self.assertTrue(np.all((p >= lo) & (p <= hi)))
        direct = Support(xy, m, [0.25] * 4, 0.31, [1.0, 0.95, 1.0, 1.1]).forces(0.5, 1.0, 0.6, 1.0)
        bound = accepted_normals(
            xy, c, m, np.array([[0.5, 1.0]]), np.array([0.6]), np.array([1.0])
        )[0]
        self.assertTrue(np.all(direct >= bound))

    def test_shared_supply_lowers_available_acceleration(self):
        base = limits(1, 4, 25, "screened", "stowed")
        busy = limits(1, 4, 25, "screened", "actuating")
        self.assertGreater(base["highestContinuousTestedAccelMps2"], 3)
        self.assertLess(busy["highestContinuousTestedAccelMps2"], 1.1)
        self.assertEqual(busy["firstFailedGates"], ["terminalFloor"])


if __name__ == "__main__":
    unittest.main()
