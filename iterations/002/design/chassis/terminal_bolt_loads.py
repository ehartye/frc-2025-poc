"""Apply each simultaneous ring anchor reaction and height to a four-fastener terminal attachment."""
import json
import math
from pathlib import Path
import numpy as np
from perimeter_footprint import structure, cases_and_forces

HERE=Path(__file__).parent


def run():
    duty=json.loads((HERE/'bumper-duty.json').read_bytes());peak=duty['upper']['peakN']
    nodes,faces,anchors,stiff,_,_,_,_=structure(6)
    loads,cases=cases_and_forces(nodes,faces)
    all_dofs=np.array([3*i+j for i in anchors for j in (0,1)])
    # Independent mechanical envelope; sleeves transmit shear across the two nominally touching tube faces.
    # Both bolt x offsets lie at least 18 mm from the saw-cut end of the commissioned 254 mm terminal.
    x_half,z_half=9.,25.4
    pilot=12.65; effective_depth=5.60-.05-3.35
    stock_yield=240.; sleeve_bearing=pilot*effective_depth*stock_yield/2
    edge=127.-109.-12.8/2-.25-.15
    edge_tearout=2*effective_depth*edge*stock_yield/math.sqrt(3)/2
    proof=140000*.006894757293168; sy=105000*.006894757293168
    pitch=25.4/20;diam=6.35;root_d=diam-1.30*pitch
    As=math.pi*(diam-.9743*pitch)**2/4
    strip_area=.5*math.pi*4.97*9.3
    clamp=(6000.,9000.);kbolt=(.15,.35)
    refined=json.loads((HERE/'lower-terminal-anchor-loads.json').read_bytes())['settings']
    loads_to_try=[(1.,'cruise'),(duty['routine']['quasistaticInterpolatedPeakN']/peak,'approach')]
    for scenario,val in refined.items():
        loads_to_try.extend([(val['routineN']/peak,scenario+'-approach'),(val['upperN']/peak,scenario+'-upper')])
    template={}
    for state in ('intact','damaged'):
        for _,impact in loads_to_try:template[(state,impact)]={}
    def retain(result,key,val,ident,extra):
        i=int(np.argmax(val));number=float(val[i])
        if key not in result or number>result[key]['value']:
            result[key]=dict(value=number,**ident,patch=cases[i],simultaneous={k:float(v[i]) for k,v in extra.items()})
    for stiffness in (3000.,30000.):
        K=stiff.copy();K[all_dofs,all_dofs]+=stiffness
        u=np.linalg.solve(K,np.column_stack((peak*loads,np.eye(len(K))[:,all_dofs])))
        base=u[:,:loads.shape[1]];influence=u[:,loads.shape[1]:]
        for missing in (None,*anchors):
            if missing is None:U=base
            else:
                removed=np.array([3*missing,3*missing+1]);g=influence[:,np.isin(all_dofs,removed)]
                U=base+g@np.linalg.solve(np.eye(2)/stiffness-g[removed],base[removed])
            severity='intact' if missing is None else 'damaged'
            for n,(face,xy) in anchors.items():
                if n==missing:continue
                inward=faces[face][2]; tangent=np.array([inward[1],-inward[0]])
                R=-stiffness*U[3*n:3*n+2]
                for factor,label in loads_to_try:
                    result=template[severity,label]
                    for direction in (-1,1):
                        N=direction*factor*(inward@R);T=direction*factor*(tangent@R)
                        for z in (114.3,150.):
                            hz=z-139.7
                            for x in (-x_half,x_half):
                                for h in (-z_half,z_half):
                                    # x and z are coordinates on the bolt plane. Positive axial draws the screw.
                                    axial=N/4+N*hz*h/(4*z_half*z_half)-T*12.7*x/(4*x_half*x_half)
                                    twist=T*hz/(4*(x_half*x_half+z_half*z_half))
                                    st=T/4+twist*h;sz=-twist*x;shear=np.hypot(st,sz)
                                    tension=np.maximum(axial,0)
                                    # Linear spring sharing up to opening; after opening the positive screw load
                                    # is the total applied tensile demand. Ignore any friction in all shear cases.
                                    peak_bolt=np.maximum(clamp[1]+kbolt[1]*2*tension,2*tension)
                                    loss=np.maximum(0,(1-kbolt[0])*tension-clamp[0])
                                    # Full upper tightening torsion stays in the screw, including after load reversal.
                                    d_pitch=diam-.64952*pitch
                                    torque=clamp[1]*d_pitch/2*math.tan(math.atan(pitch/(math.pi*d_pitch))+math.atan(.18/math.cos(math.pi/6)))
                                    tau_t=16*torque/(math.pi*root_d**3)
                                    strength=np.hypot(peak_bolt/As,np.sqrt(3)*tau_t)/proof
                                    data=dict(N=N,T=T,boltAxialServiceN=axial,boltTangentN=st,boltVerticalN=sz)
                                    ident=dict(stationMm=xy.tolist(),missingMm=None if missing is None else nodes[missing].tolist(),heightMm=z,direction=direction,boltLocalXZmm=[x,h],stiffnessNPerMm=stiffness)
                                    for key,values in [('maxServiceAxialN',tension),('maxServiceShearN',shear),('maxBoltProofFraction',strength),
                                        ('maxReceiverThreadShearFraction',peak_bolt/(strip_area*sy/math.sqrt(3)/2)),
                                        ('maxPilotTubeBearingFraction',shear/sleeve_bearing),
                                        ('maxTerminalEndTearoutFraction',abs(st)/edge_tearout),
                                        ('maxExcessDemandOverMinimumServiceClampN',loss)]:
                                        retain(result,key,values,ident,data)
    return dict(assumptions=dict(frameReactionPlaneToJointMm=12.7,boltPatternMm=[18,50.8],operatingPreloadRangeN=clamp,boltStiffnessFraction=kbolt,
        pilotODmm=pilot,minSecondWallEngagementMm=effective_depth,minTangentEndLigamentMm=edge,assumedMinProofMPa=proof,shaftTensileAreaMm2=As,
        maxThreadFriction=.18,factoredExternalTension=2,terminalWidthMm=254),
        cases={f'{s}/{i}':a for (s,i),a in template.items()},
        scope='The 488-based bumper proposal uses four 91251A542 screws at each station and shop-made H1150 tapped nut blocks and shouldered washers. Through-wall pilot takes all modeled tangential shear. Torque, material certificates, preload scatter, actual thread/under-head CAD, full ring net section at holes, wall bending, closed terminal/miter/deck stiffness and loss-induced contact opening remain to be resolved. Current ring reaction model has ideal miters and 3–30 kN/mm independent springs, not a proof of assembled joint stiffness. Small excesses are recorded as failures. Cover/backing detaches for socket entry.')

if __name__=='__main__':
    r=run();(HERE/'terminal-bolt-loads.json').write_text(json.dumps(r,indent=2)+'\n')
    for k,v in r['cases'].items():print(k,{q:round(n['value'],3) for q,n in v.items()})
