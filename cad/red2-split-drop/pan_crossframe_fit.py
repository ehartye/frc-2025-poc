"""Locate crossframe ends and service the real rail before machining a cassette."""
import json
import sys
from pathlib import Path
from output_mount import ROOT,digest,read
from pan_counterbrace import box
from pan_thrust_layout import cylinder,subtract


def run(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import common,bounds
    from showcase_contacts import load_solids,box_gap
    p=ROOT/'artifacts/red2-integrated/pan-thrust';issue=p/'moving-stack-issued'
    reference=read(issue/'r3-moving-proxy-short-01.json')
    authority=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    assert reference['authority']['archiveSha256']==digest(ROOT/authority['file'])
    step=issue/'checked-301.step';assert digest(step)==reference['stepSha256']
    labels={};original,extra=load_solids(step,labels);assert len(original)==301 and not extra
    def find(word):
        k,=[k for k,v in labels.items() if word in v['occurrencePath']];return k
    left=original.copy()
    old={q:find(q.title()+' bearing tower foot') for q in ['port','starboard']}
    for k in old.values():del left[k]
    known={k:bounds(v) for k,v in left.items()}
    results={}
    for near in [148.,110.]:
        # The secondary location is a single predeclared access control, clear
        # of the actual x=148 front inclined post. The shaft remains at x=205.
        parts={}
        for x in [near,250.]:
            b=box((x-15,-231,450),(x+15,231,480))
            cm=box((x-12,-231.2,453),(x+12,231.2,477))
            parts[f'x{x:g} transverse closed section']=subtract(b,cm)
        for s,q in [(1,'port'),(-1,'starboard')]:
            a,b=sorted([s*177.,s*251.])
            parts[q+' continuous tower sole']=box((near-18.,a,480),(268.,b,487))
            for x in [near,250.]:
                y=s*242
                kws={'axis / head withdrawal':cylinder((x,y,397.),(0,0,1),4.1,46.),
                     'key shank 82':cylinder((x,y,361.),(0,0,1),3.5,77.2),
                     'driver envelope':cylinder((x,y,268.),(0,0,1),16.,95.)}
                for k,v in kws.items():parts[f'{q} x{x:g} '+k]=v
        hits={}
        for n,shape in parts.items():
            bb=bounds(shape);hits[n]={}
            for k,v in left.items():
                if box_gap(bb,known[k])>.0001:continue
                ov=common(shape,v)
                if ov>.001:hits[n][k]=ov
        results[str(near)]={'unintendedCommonMm3':hits,'members':['crossmember at '+str(x) for x in [near,250]],'nominalEndGapMm':2.}
    record={'authority':authority,'privateNativeAndSTEP':[reference['archiveSha256'],reference['stepSha256']],
            'originalBodies':len(original),'removed':old,'cases':results,'scope':'Actual 301 stow; two full-width 30×30×3 mm removable transverse beams and two 7 mm extended soles, anticipated lower access and 2 mm stand-off to each sill. Rolling, brace, motion and field qualification remain separate.'}
    (p/'two-sill-cartridge-fit.json').write_text(json.dumps(record,indent=2)+'\n')
    for k,v in results.items():print(k,{n:h for n,h in v['unintendedCommonMm3'].items() if h},flush=True)


if __name__=='__main__':run(Path(sys.argv[1]))
