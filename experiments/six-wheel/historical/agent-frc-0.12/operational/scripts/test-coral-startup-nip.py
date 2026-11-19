import unittest,pathlib,importlib.util,json,math
ROOT=pathlib.Path(__file__).resolve().parents[1]
def load(name,file):
 s=importlib.util.spec_from_file_location(name,ROOT/file);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
c=load('c','scripts/coral-startup-report.py');n=load('n','scripts/open-receiver-nip.py')
class SensitivityTests(unittest.TestCase):
 def test_mass_source_and_grid(self):
  e=json.loads((ROOT/'case-studies/coral-mass-evidence.json').read_text());self.assertEqual(e['massBoundsLb'],[1.1,1.8]);self.assertEqual(e['massBoundsKg'],[v*.45359237 for v in (1.1,1.8)]);r=c.report();self.assertEqual(len(r['cases']),108);self.assertEqual(r,json.loads((ROOT/'case-studies/coral-startup-report.json').read_text()));self.assertGreater(r['summary']['ROLLBACK'],0);self.assertGreater(r['summary']['EXIT'],0)
 def test_source_does_not_supply_friction_or_pose(self):
  for row in c.report()['cases']:self.assertEqual(row['frictionBasis'],'independent-hypothesis-not-manual');self.assertEqual(row['orientationBasis'],'guide-elevation-not-full-tube-pose');self.assertIsNone(row['scoring'])
 def test_insufficient_traction_fails(self):
  for row in c.report()['cases']:
   if row['mu']==.05 and row['pitchDegrees']==30:self.assertEqual(row['outcome'],'ROLLBACK')
 def test_native_report_fresh(self):
  r=n.report();self.assertEqual(r,json.loads((ROOT/'case-studies/open-receiver-nip.json').read_text()));self.assertEqual(len(r['cases']),72);self.assertEqual(set(r['sourceCodeSha256']),set(n.PINS))
 def test_conservative_bound_agrees_with_native_samples(self):
  r=json.loads((ROOT/'case-studies/open-receiver-nip.json').read_text())
  for x in r['cases']:
   self.assertGreaterEqual(x['nativeMinDistanceM']+1e-7,max(0,x['conservativeVerticalGapM']))
   if x['verticalEnvelopePass']:self.assertTrue(x['nativeSamplePass'])
 def test_pitch_interval_and_translation_certificate(self):
  r=json.loads((ROOT/'case-studies/open-receiver-nip.json').read_text())
  for cert in r['intervalCertificates']:
   y=cert['season'];required=cert['minimumSymmetricRetractionM'];self.assertLessEqual(required,.06)
   for i in range(1001):
    pitch=-20+40*i/1000;clear=.06-.001-(n.extent(y,pitch)-n.extent(y,0))-.002;self.assertGreaterEqual(clear,.002-1e-12)
 def test_closed_nip_counterexample_and_sphere_refinement(self):
  r=json.loads((ROOT/'case-studies/open-receiver-nip.json').read_text())['cases'];rows={x['season']:x for x in r if x['bodyCenterXM']==.4 and x['pitchDegrees']==0 and x['retractionEachSideM']==0}
  for y in (2024,2025):self.assertGreater(rows[y]['nativeIntersectionSumM3'],0)
  self.assertGreater(rows[2026]['nativeMinDistanceM'],.009);self.assertEqual(rows[2026]['nativeIntersectionSumM3'],0)
  for x in r:
   if x['bodyCenterXM']==.3496 and x['pitchDegrees']==0 and x['retractionEachSideM']==0:self.assertGreater(x['nativeIntersectionSumM3'],0)
 def test_pitch_invalid_rejected(self):
  for p in (True,float('nan'),float('inf'),21,'20'):
   with self.assertRaises(ValueError):n.extent(2024,p)
if __name__=='__main__':unittest.main()
