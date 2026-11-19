import unittest
from scripts.kickoff_shuttle import evaluate, read


class KickoffWindow(unittest.TestCase):
    def setUp(self):
        self.model = read("rules-v0.json")
        self.session = read("trials.json")

    def test_first_delivery_has_no_return_and_repeats_do(self):
        result = evaluate(self.model, self.session, self.session["trials"][0])
        self.assertEqual(result["piecesByDeadline"], 2)
        self.assertEqual(result["pointsIfEachScores"], 4)
        self.assertAlmostEqual(result["finishTimesSeconds"][2], 17.13723967)
        self.assertAlmostEqual(result["minimumMeanSpeedForGoalInPerSecond"], 73.49835583)
        self.assertFalse(result["goalFits"])

    def test_single_available_algae_caps_the_result(self):
        result = evaluate(self.model, self.session, self.session["trials"][1])
        self.assertEqual(result["piecesByDeadline"], 1)
        self.assertEqual(result["pointsIfEachScores"], 6)
        self.assertTrue(result["goalFits"])
        self.assertAlmostEqual(result["finishTimesSeconds"][0], 6.63253063)
