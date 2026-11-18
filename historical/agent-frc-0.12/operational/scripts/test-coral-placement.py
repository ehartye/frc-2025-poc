import importlib.util,json,math,pathlib,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
s=importlib.util.spec_from_file_location('placement',ROOT/'scripts/coral-placement.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
class Placement(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.r=m.report();cls.poly=[[x*.0254 for x in p] for p in cls.r['floorSource']['verticesInch']]
 def test_fresh_report(self):
  self.assertEqual(self.r,json.loads((ROOT/'case-studies/coral-placement.json').read_text(encoding='utf-8-sig')))
  self.assertEqual(set(self.r['pins']),set(m.PINS));self.assertEqual(len(self.r['cases']),576)
 def test_stopping_analytic(self):
  r=m.evaluate([.2,.1,m.RADIUS],[.2,0,0],0,.3,self.poly)
  self.assertAlmostEqual(r['stopDistanceM'],.04/(.6*m.G));self.assertEqual(r['landingTimeS'],0)
 def test_negative_controls(self):
  self.assertFalse(m.evaluate([.2,.1,m.RADIUS],[1,0,0],0,0,self.poly)['conditionalContainedStop'])
  self.assertFalse(m.evaluate([.2,.1,m.RADIUS],[1,0,0],15,.5,self.poly)['axialModelApplicable'])
  self.assertLess(m.margin([.45,.08],0,self.poly),0)
  for bad in (True,float('nan'),float('inf'),-1):
   with self.assertRaises(ValueError):m.evaluate([.2,.1,m.RADIUS],[1,0,0],0,bad,self.poly)
 def test_runtime_speed_preserved(self):
  for c in self.r['cases']:
   v=c['improvedRegistration']['exitVelocityMps'];self.assertAlmostEqual(math.sqrt(sum(x*x for x in v)),c['sourceExit']['speed'])
   if c['muHypothesis']==0:self.assertIsNone(c['improved']['stopCenterM'])
 def test_endpoint_proof_independent_samples(self):
  # Explicit horizontal cylinder projected boundary vertices, sampled along entire stop path.
  for c in self.r['cases'][::7]:
   r=c['improved']
   if r['stopCenterM'] is None:continue
   a=math.radians(c['yawDeg']);axis=(math.cos(a),math.sin(a));perp=(-axis[1],axis[0]);observed=math.inf
   for t in (0,.2,.5,.8,1):
    center=[r['landingCenterM'][i]*(1-t)+r['stopCenterM'][i]*t for i in (0,1)]
    for sign in (-1,1):
     for side in (-1,1):
      pt=[center[i]+sign*m.HALF*axis[i]+side*m.RADIUS*perp[i] for i in (0,1)]
      for p,q in zip(self.poly,self.poly[1:]+self.poly[:1]):
       dx,dy=q[0]-p[0],q[1]-p[1];observed=min(observed,(-dy*(pt[0]-p[0])+dx*(pt[1]-p[1]))/math.hypot(dx,dy))
   self.assertAlmostEqual(observed,r['sweptMarginM'],places=12)
 def test_preserved_failures_and_improvements(self):
  self.assertTrue(any(c['baseline']['landingMarginM']<0 for c in self.r['cases']))
  self.assertTrue(any(c['improved']['conditionalContainedStop'] and not c['baseline']['conditionalContainedStop'] for c in self.r['cases']))
  self.assertIsNone(self.r['officialScore'])
if __name__=='__main__':unittest.main()
