"""Exact fixed-pose CORAL/wheel overlap predicate, not a Euclidean gap or force law."""
import math,importlib.util,pathlib,json,unittest
sp=importlib.util.spec_from_file_location('p',pathlib.Path(__file__).with_name('prepare-segmented-replay.py'));p=importlib.util.module_from_spec(sp);sp.loader.exec_module(p)
def coral(x,compression):
 p.g.finite_number(x);p.g.finite_number(compression)
 if not -250<=x<=386 or compression not in (0,5):raise ValueError('fixed pose/domain required')
 R=57.15;half_length=150.8125;c=p.SEASONS[2025];zw=c['z']-compression;zc=R
 if zw<=zc+R:raise ValueError('upper-wheel proof requires wheel center above entire tube')
 near_x=max(abs(x)-half_length,0)
 if near_x>c['radius']:return dict(overlap=False,axis='X',marginMm=near_x-c['radius'])
 margins=[]
 for center in c['centers']:
  near_y=max(abs(center)-c['half'],0)
  if near_y>R:continue
  tube_top=zc+math.sqrt(R*R-near_y*near_y)
  wheel_bottom=zw-math.sqrt(c['radius']**2-near_x**2)
  margins.append(wheel_bottom-tube_top)
 if not margins:raise ValueError('no lateral candidate; model configuration changed')
 margin=min(margins)
 return dict(overlap=margin<0,axis='vertical-clearance',marginMm=margin)
def report():
 c=p.SEASONS[2025];near_y=min(max(abs(y)-c['half'],0) for y in c['centers'])
 top=57.15+math.sqrt(57.15**2-near_y**2)
 reach=math.sqrt(c['radius']**2-(c['z']-5-top)**2)
 return dict(schema='frc-coral-fixed-pose-entry/v1',geometryEvidenceSha256=p.evidence(),
  compressedContactCenterIntervalMm=[-150.8125-reach,150.8125+reach],
  beamCenterIntervalMm=[-150.8125,150.8125],
  entryLeadBeforeBeamMm=reach,calibration=None,engineeringAcceptance='blocked',
  basis='Exact overlap predicate for ideal cylinders at fixed pose; vertical margin is not Euclidean distance, penetration or normal force.',
  samples={str(x):coral(x,5) for x in [-250,-177,-166,-165,-151,-150,0,150,165,166,177,250]})
class GeometryTests(unittest.TestCase):
 def test_full_span_matches_prior_sign(self):
  for compression in (0,5):
   for x in [-125,0,125]:self.assertEqual(coral(x,compression)['overlap'],p.gaps(2025,x,compression)[1]<0)
 def test_ends_close_prior_unknown(self):
  self.assertEqual(p.gaps(2025,-165,5),(None,None));self.assertTrue(coral(-165,5)['overlap']);self.assertFalse(coral(-166,5)['overlap'])
 def test_boundary(self):
  bound=report()['compressedContactCenterIntervalMm'][1]
  self.assertTrue(coral(bound-1e-5,5)['overlap']);self.assertFalse(coral(bound+1e-5,5)['overlap'])
 def test_symmetry_nominal_separation(self):
  for x in range(0,251,5):
   self.assertEqual(coral(x,5),coral(-x,5));self.assertFalse(coral(x,0)['overlap'])
 def test_invalid(self):
  for v in [True,'1',float('nan'),float('inf')]:
   with self.assertRaises(ValueError):coral(v,5)
if __name__=='__main__':
 import sys
 if sys.argv[1:]==['--test']:unittest.main(argv=[sys.argv[0]])
 else:print(json.dumps(report(),indent=2,allow_nan=False))
