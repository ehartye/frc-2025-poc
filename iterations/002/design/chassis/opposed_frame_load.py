"""Evaluate separate longitudinal rail stations, then challenge the local beam carrying their load."""
import json,math
from pathlib import Path
import numpy as np

p=Path(__file__).parent
sources=json.loads((p/'corner-wrench.json').read_bytes())
def group(xs,ys,forceN,point):
    xy=np.array([(x,y) for y in ys for x in xs],dtype=float);mid=xy.mean(axis=0);r=xy-mid
    load=np.array(forceN);F=load[:2];m=np.cross(np.array(point)-[*mid,94.6333628767],load)
    shear=F/len(r)+m[2]*np.column_stack([-r[:,1],r[:,0]])/np.sum(r*r)
    A=np.vstack([np.ones(len(r)),r[:,1],-r[:,0]])
    req=np.array([load[2],m[0],m[1]]);rz=np.linalg.lstsq(A,req,rcond=None)[0]
    residual=A@rz-req
    if max(abs(residual))>.0001:raise ValueError('Invalid tension force and moment balance')
    vals=[]
    for loc,f,z in zip(xy,shear,rz):
        nd=float(np.linalg.norm(f))
        # Mild structural plate + fitted sleeve; no friction or glue credited.
        # Treat a single min-thickness host wall as carrying full bearing.
        cap=9.525*3*240/2
        net=(25.4-9.65)*3*240/2
        vals.append(dict(xyMm=loc.tolist(),shearXYN=f.tolist(),shearN=nd,normalN=float(z),
                        singleWallBearingUtilization=nd/cap,netSectionUtilization=nd/net))
    return {'maxBearingUtilization':max(v['singleWallBearingUtilization'] for v in vals),'maxAbsSignedVerticalN':max(abs(v['normalN']) for v in vals),
            'minimumContactEdgeMm':(25.4-9.65)/2,'forceN':forceN,'applicationMm':point,'forceMomentResidualFzMxMy':residual.tolist(),'rows':vals}

out={}
for k,c in sources['impactCases'].items():
    side='/side/' in k
    # Isotropic transformed comparison of the front/back and left/right
    # rail stations. Sill spans differ by 5 mm, carried through the yaw moment.
    F=c['forceN'][:];pt=c['applicationMm'][:];offset=330.975
    if side:F[0],F[1]=F[1],F[0];pt[0],pt[1]=pt[1],pt[0];offset=335.975
    out[k]=group([-150,-100,100,150],[-offset,offset],F,pt)
worst=max(out,key=lambda k:out[k]['maxBearingUtilization'])
top=max(out,key=lambda k:out[k]['maxAbsSignedVerticalN'])
peak=max(abs(max(v['forceN'],key=abs)) for v in out.values())
def section(b,h,t):
    return (h*b**3-(h-2*t)*(b-2*t)**3)/(6*b)
screens={}
for b,h in [(25.4,25.4),(25.4,50.8),(25.4,76.2)]:
    W=section(b,h,3.)
    screens[f'{b}x{h}']=dict(sectionModulusMm3=W,allowedSpanInThreePointBendingMm=4*120*W/peak,
                            cantileverAtPeakMm=120*W/peak,stockMassKgPerM=(b*h-(b-6)*(h-6))*.0027,
                            maxStressFor200mmSpanMPa=peak*200/(4*W),maxStressFor106mmSpanMPa=peak*106/(4*W))
result=dict(authority=sources['authority'],cases=out,worstBearingCase=worst,worstVerticalCase=top,peakPointLoadN=peak,
 activeWrenchSource='corner-wrench.json',sleeveOuterDiameterMm=9.525,shopDrillMaxMm=9.65,minimumStockWallMm=3.,
 mechanicalDesign={'sleeve':'Shop-turned flanged steel sleeve; Ø9.525 mm shear land across upper strap and rail walls, ≥15.875 mm OD integral washer by 2.0 mm. Source steel condition and tolerance analysis still required.',
 'tappedBacking':'Shop lower steel tapped bar with four 10-32 threads at the positive fittings on each continuous longitudinal member. Two complete opposed rails are needed; port only exists today.',
 'screw':'Existing exact 2023 SDS occurrence 91251A355, 10-32 ×2.5-in SHCS. Underhead 63.5 minus 50.8 rail, 3.175 top plate and 2 integral washer leaves 7.525 mm nominal thread reach in a 9.525 mm lower backing bar. Verify counterseat, thread runout, pullout, torque scatter and tools from installed data.',
 'bumper':'Removable 4 mm 5052-H32 metal backer, separate Ethafoam 220, cloth and metallic frame mount. The corner outside-L is a candidate; a load carried by the named group requires a physically continuous shear path and actual rigid support.'},
 beamScreens=screens,assumptions='Compare full simultaneous force and moment and reversed loading; eight non-collinear positive stations active, zero friction. The independent simply supported beam with a central point load and the single-arm cantilever expose where a thin outside band can fail before its bolts. This is an engineering limit, not a bow to presumed equal force sharing.')
(p/'opposed-frame-load.json').write_text(json.dumps(result,indent=2)+'\n')
print(worst,out[worst]['maxBearingUtilization'],top,out[top]['maxAbsSignedVerticalN'],peak,screens)
