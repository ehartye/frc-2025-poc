import importlib.util
import json
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("receiver_load_path", ROOT / "structural/receiver_load_path.py")
loads = importlib.util.module_from_spec(spec)
spec.loader.exec_module(loads)


class ReceiverLoads(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(loads.CONFIG.read_text())
        self.points = self.config["shelfMountsMm"]

    def test_group_resultant_and_moment_balance(self):
        force = [110, -850]
        position = [165, 210]
        group = loads.fastener_shear(self.points, force, position)
        center = group["centroidMm"]
        forces = group["transmittedLoadsN"]
        for axis in (0, 1):
            self.assertAlmostEqual(sum(f[axis] for f in forces), force[axis])
        torque = sum((p[0]-center[0])*f[1] - (p[1]-center[1])*f[0]
                     for p, f in zip(self.points, forces))
        self.assertAlmostEqual(torque/1000, group["momentNm"])
        self.assertAlmostEqual(torque, (position[0]-center[0])*force[1] - (position[1]-center[1])*force[0])

    def test_centered_load_is_shared_and_stroke_does_not_change_internal_bolt_moment(self):
        at_center = loads.fastener_shear(self.points, [0, 400], [30, 0])
        self.assertEqual(at_center["transmittedLoadsN"], [[0,100]]*4)
        moved = [[x+240,y] for x,y in self.points]
        base = loads.fastener_shear(self.points, [0, -400], [165,210])
        extended = loads.fastener_shear(moved, [0, -400], [405,210])
        self.assertEqual(base["transmittedLoadsN"], extended["transmittedLoadsN"])

    def test_registered_envelope_and_independent_energy_scale(self):
        def elastic_screen(own, peer, speed, crush, offset):
            energy = 0.5 * own*peer/(own+peer) * speed**2
            return {"relativeEnergyJ":energy,"averageCompressionForceN":energy/crush}
        rows = loads.calculate(self.config,json.loads(loads.FIXTURE.read_text()),elastic_screen)
        rows = {r["pose"]:r for r in rows if r["scenario"]=="two-meters-per-second-compliant"}
        self.assertEqual(rows["receive"]["frontBumperSetbackMm"],20)
        self.assertEqual(rows["place"]["frontBumperSetbackMm"],-110)
        self.assertTrue(math.isclose(rows["place"]["carriageYawMomentNm"],-535))
        self.assertIsNone(rows["place"]["structuralPass"])
        self.assertIsNone(rows["place"]["peakForceN"])

    def test_invalid_mounts(self):
        for points in ([[0,0],[0,0]], [[0,0],[math.inf,0]]):
            with self.assertRaises(ValueError):
                loads.fastener_shear(points,[0,1],[1,0])
