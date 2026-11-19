import copy
import json
from pathlib import Path
import unittest

from scripts.assess_coral import assess

ROOT = Path(__file__).resolve().parents[1]
KICKOFF = "kickoff-v0"
COMPILED = "archive-compiled"


def pipe(identity="A", **observations):
    return {"id": identity, "observedMs": 500,
            "touchesAlliedRobot": False, "touchesOpponentRobot": False,
            "touchesOwnTrough": True, "directSupporterIds": [],
            "scoredOnAnotherReefLevel": False, "fullySupportedByRobot": False,
            "stuckToRobot": False, "intentionalHerding": False, **observations}


def snap(rows, time=500, complete=True):
    return {"timeMs": time, "inventoryComplete": complete,
            "robotCommandActive": False, "pieces": rows}


def trace(*samples, phase="AUTO"):
    return {"schema": "reefscape-coral-observations/1", "phase": phase,
            "basis": "synthetic independent observations", "samples": list(samples)}


def outcome(rows, edition=COMPILED):
    return assess(trace(snap(rows)),edition)["samples"][0]


class CoralAssessment(unittest.TestCase):
    def test_edition_changes_for_contact_and_other_level(self):
        examples = [
            (pipe(), "yes", "yes"),
            (pipe(touchesOpponentRobot=True), "no", "yes"),
            (pipe(scoredOnAnotherReefLevel=True), "yes", "no"),
            (pipe(touchesAlliedRobot=True), "no", "no"),
            (pipe(touchesAlliedRobot=None), "unknown", "unknown"),
        ]
        for row, v0, compiled in examples:
            for edition, expected in [(KICKOFF,v0),(COMPILED,compiled)]:
                with self.subTest(row=row, edition=edition):
                    self.assertEqual(outcome([row],edition)["pieces"][0]["l1CriteriaAtInstant"],expected)

    def test_support_requires_direct_same_time_identified_trough_contact(self):
        upper = pipe("upper",touchesOwnTrough=False,directSupporterIds=["middle"])
        middle = pipe("middle",touchesOwnTrough=False,directSupporterIds=["base"])
        base = pipe("base")
        for edition in (KICKOFF,COMPILED):
            rows=outcome([upper,middle,base],edition)["pieces"]
            self.assertEqual([r["l1CriteriaAtInstant"] for r in rows],["no","yes","yes"])
            for missing in ([upper], [upper,pipe("middle",observedMs=499)]):
                self.assertEqual(outcome(missing,edition)["pieces"][0]["l1CriteriaAtInstant"],"unknown")
        no_evidence = pipe(touchesOwnTrough=False,directSupporterIds=None)
        self.assertEqual(outcome([no_evidence])["pieces"][0]["l1CriteriaAtInstant"],"unknown")

    def test_contact_only_and_sensor_absence_are_ambiguous_for_control(self):
        row={"id":"A","observedMs":500,"touchesAlliedRobot":True}
        sample=outcome([row])["pieces"][0]
        self.assertEqual(sample["controlledByFocalRobot"],"unknown")
        self.assertEqual(sample["l1CriteriaAtInstant"],"no")
        self.assertEqual(outcome([pipe()])["pieces"][0]["controlledByFocalRobot"],"no")

    def test_g409_counts_unique_concurrent_pieces_and_narrows_push_exception(self):
        held = pipe(fullySupportedByRobot=True)
        herded = pipe("B",intentionalHerding=True,pushesScoredL1WhilePlacing=True)
        self.assertEqual(outcome([held,herded],KICKOFF)["controlledCoralCountScreen"],"over")
        later=outcome([held,herded],COMPILED)
        self.assertEqual(later["controlledCoralCountScreen"],"within")
        self.assertEqual(later["pieces"][1]["controlledByFocalRobot"],"yes")
        herded["pushesScoredL1WhilePlacing"]=None
        self.assertEqual(outcome([held,herded])["controlledCoralCountScreen"],"unknown")
        herded["stuckToRobot"]=True
        self.assertEqual(outcome([held,herded])["controlledCoralCountScreen"],"over")
        held_only=assess(trace(snap([held],complete=False)),COMPILED)["samples"][0]
        self.assertEqual(held_only["controlledCoralCountScreen"],"unknown")

    def test_no_double_count_identity_or_stale_state_and_no_official_score(self):
        model=trace(snap([pipe()]),snap([pipe(observedMs=700)],time=700),snap([pipe(observedMs=700)],time=800))
        result=assess(model,KICKOFF)
        self.assertEqual(result["officialEarnedPoints"],None)
        self.assertEqual([s["pieces"][0]["id"] for s in result["samples"]],["A"]*3)
        self.assertEqual([s["pieces"][0]["l1CriteriaAtInstant"] for s in result["samples"]],["yes","yes","unknown"])
        with self.assertRaises(ValueError): outcome([pipe(),pipe()])
        with self.assertRaises(ValueError): outcome([pipe(directSupporterIds=["A"])])
        with self.assertRaises(ValueError): outcome([pipe(stuckToRobot="true")])
        with self.assertRaises(ValueError): outcome([pipe(observedMs=501)])

    def test_phase_boundaries_and_no_post_buzzer_commands(self):
        for phase,end in (("AUTO",15000),("TELEOP",135000)):
            times=(end-1,end,end+3000,end+3001)
            series=trace(*(snap([pipe(observedMs=t)],time=t) for t in times),phase=phase)
            result=assess(series,KICKOFF)["samples"]
            self.assertEqual([s["clock"] for s in result],["active","assessment-window","assessment-window","outside-window"])
            self.assertEqual(result[-1]["pieces"][0]["l1CriteriaAtInstant"],"outside-window")
            invalid=copy.deepcopy(series);invalid["samples"][1]["robotCommandActive"]=True
            with self.assertRaisesRegex(ValueError,"expired"): assess(invalid,KICKOFF)
        with self.assertRaises(ValueError): assess(trace(snap([]),snap([])),KICKOFF)

    def test_committed_trace_and_project_receipt_have_different_evidence(self):
        sample=json.loads((ROOT/"scoring/coral-observations.json").read_bytes())
        self.assertEqual(len(assess(sample,KICKOFF)["samples"]),5)
        actual=json.loads((ROOT/"cad/station-receiver/contact-receipt.json").read_bytes())
        self.assertEqual(actual["schema"],"receiver-step-first-contact/1")
        with self.assertRaisesRegex(ValueError,"schema"): assess(actual,COMPILED)


if __name__=="__main__":
    unittest.main()
