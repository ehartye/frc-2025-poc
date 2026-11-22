"""Matched two-sill cartridge footprint and whole-clock support bounds before detailed CAD."""
import json
import math
import sys
import numpy as np
from output_mount import ROOT,read,digest


def solve(check_access=False):
    folder=ROOT/'artifacts/red2-integrated';p=folder/'pan-thrust'
    previous=read(p/'r3-all-clock-station-reactions.json')
    original=read(folder/'single-pivot-spatial-rim-installed-reactions-frame-anchors.json')
    shop=read(folder/'pan-mechanics.json');authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert digest(ROOT/authority['file'])==original['sourceSha256']==authority['sha256']
    assert digest(folder/'single-pivot-spatial-rim-installed-reactions-frame-anchors.json')==previous['sourceSha256']
    # Two side sills are commissioned supports, with the aft beam at the
    # actual x=148 inclined-post datum. The same cartridge holds BOTH bearings.
    xs=[148.,250.]
    fit_file=p/'two-sill-cartridge-fit.json'
    if check_access:
        accepted=read(fit_file)
        clear=accepted['cases']['110.0']['unintendedCommonMm3']
        failed=accepted['cases']['148.0']['unintendedCommonMm3']
        if not clear or any(clear.values()) or not any(failed.values()):raise ValueError('Need matched physical service controls')
        xs=[110.,250.]
    ys=[-242.,242.];ox=sum(xs)/2
    rows={};m=shop['movingShopStockKg'];cy=shop['movingShopCentreMm'][1]/1000
    h=.195775;halfX=(xs[1]-xs[0])/2000;halfY=.242
    J=4*(halfX*halfX+halfY*halfY)
    reset_torque=m*9.80665*math.hypot(shop['movingShopCentreMm'][0]/1000-.205,shop['movingShopCentreMm'][2]/1000-.580)+shop['movingStockInertiaKgM2']*6*(math.radians(72)+.04)/1.6**2
    work=max([reset_torque]+[r['loadPeaks']['shaftDutyNm']['value'] for r in original['runs'].values()])
    ps=previous['perTowerAndStation']
    radii=[ps['pipeAndShopMass/'+t]['radialBearingN']['value'] for t in ['port','starboard']]
    axial=ps['pipeAndShopMass/starboard']['axialBearingN']['value']
    for x in xs:
        for y in ys:
            qx=(x-ox)/1000;qy=y/1000
            def force_per_station(vector):
                a,b, c,d,e,T=vector # signed port X/Z, starboard X/Z/Y, gearbox reaction torque
                port=np.array([a,0.,b]);star=np.array([c,e,d]);net=port+star
                M=np.cross([(.205-ox/1000),h,.1],port)+np.cross([.205-ox/1000,-h,.1],star)+np.array([0.,T,0.])
                return net/4+np.array([-M[2]*qy/J,M[2]*qx/J,M[0]*qy/(4*halfY**2)-M[1]*qx/(4*halfX**2)])
            A=np.column_stack([force_per_station(e) for e in np.eye(6)])
            max_normal=2*sum(np.linalg.norm(A[2,2*i:2*i+2])*rad for i,rad in enumerate(radii))+2*abs(A[2,4])*axial+2*abs(A[2,5])*work
            max_shear=2*sum(np.linalg.norm(A[:2,2*i:2*i+2],ord=2)*rad for i,rad in enumerate(radii))+2*np.linalg.norm(A[:2,4])*axial+2*np.linalg.norm(A[:2,5])*work
            peaks={};events=0
            for label,row in original['runs'].items():
                for tower in ['port','starboard']:
                    frac=(1+(1 if tower=='port' else -1)*cy/h)/2
                    for key,ev in row['anchorRows']['pipeAndShopMass/'+tower].items():
                        rP=np.array(ev['portRadialXZ_N']);rS=np.array(ev['starboardRadialXZ_N'])
                        f=np.array(ev['forceToTowerN'])
                        fabrication=(f[[0,2]]+(rP if tower=='port' else rS))/frac
                        P=-rP+fabrication*(1+cy/h)/2;S=-rS+fabrication*(1-cy/h)/2
                        v=[*P,*S,ev['pipeForceN'][1],-ev['requiredActuatorTorqueY_Nm']]
                        actual=2*force_per_station(v);events+=1
                        for k,num in {'tensionN':actual[2],'compressionN':-actual[2],'shearN':np.linalg.norm(actual[:2])}.items():
                            if k not in peaks or num>peaks[k]['value']:
                                peaks[k]=dict(value=float(num),matchedCase=label,channel=tower+'/'+key,s=ev['timeS'],input=v,reactionN=actual.tolist())
            rows[f'{x:g}/{y:g}']=dict(enclosingAbsoluteNormalN=max_normal,enclosingShearN=max_shear,actualMatchedReactions=peaks,
                entryRows=events,clampRemainingForNominal3000N=3000*.65*.8-max_normal,
                steelThreadedInsertPreloadAndReactionN=3000*1.35+max_normal)
    out=dict(authority=authority,sourceMatched=previous['sourceSha256'],sourceCompleteClock=digest(p/'r3-all-clock-station-reactions.json'),sourceReset=digest(folder/'single-pivot-rim-field/post-impact-reset.csv'),
             chosenAnchorXYZmm=[[x,y,480.] for x in xs for y in ys],sourceRadialBoundsN=radii,sourceAxialBoundN=axial,
             torqueBoundNm=work,resetTorqueBoundNm=reset_torque,loadFactor=2.,rows=rows,
             layout='A removable transverse subframe joins the two bearing towers and gearbox. Four interfaces on the two original opposite carriage sills. No lateral bridge load credit without real section, joint and end connection.',
             limits='Equal original cardinal support stiffness. Geometric and section design, attachment development, beam torsion, chassis-post and rail bending, inter-article installation, local pressure, actual fixed mass and added moving purchases are open. Full-clock component envelope is deliberately wider than the synchronized sampled peak rows.')
    if check_access:out['actualToolAndParkedFitSha256']=digest(fit_file)
    (p/('two-sill-cartridge-selected-duty.json' if check_access else 'two-sill-cartridge-duty.json')).write_text(json.dumps(out,indent=2)+'\n')
    print('all-clock 2x absolute',max(v['enclosingAbsoluteNormalN'] for v in rows.values()),'synchronized tension',max(v['actualMatchedReactions']['tensionN']['value'] for v in rows.values()),'minimum remaining clamp',min(v['clampRemainingForNominal3000N'] for v in rows.values()))
    print('shear bound',max(v['enclosingShearN'] for v in rows.values()))


if __name__=='__main__':solve('--clear-service' in sys.argv)
