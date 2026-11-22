"""Test under-frame attachment with manufactured backing bridges and short captive upper nuts."""
import json
import sys
from pathlib import Path
from functools import lru_cache
import numpy as np

from output_mount import ROOT, digest, read, place
from pan_counterbrace import box
from pan_thrust_layout import transform, cylinder, fuse, subtract, translate
from pan_rated_barrel import horizontal_seat


def capsule(x,y,z,length,width,height):
    r=width/2
    a=(length-width)/2
    body=box((x-a,y-r,z),(x+a,y+r,z+height))
    for dx in (-a,a):
        body=fuse(body,cylinder((x+dx,y,z),(0,0,1),r,height))
    return body


def check(plugin, web_shoe=False):
    sys.path.insert(0,str(plugin/'scripts'))
    from mechanical_motion import bounds, common
    from showcase_contacts import load_solids, box_gap
    p=ROOT/'artifacts/red2-integrated/pan-thrust'; source=p/'fastener-option'
    issue=read(p/'fixed-races/pan-r3-fixed-races-01.json')
    register=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    native=p/'fixed-races/checked-297.step'
    inputs=read(source/'sds-threaded-screws.json')
    assert digest(ROOT/register['file'])==issue['authority']['archiveSha256'] and digest(native)==issue['stepSha256']
    labels={}; parts,extra=load_solids(native,labels)
    assert len(parts)==297 and not extra
    def find(word):
        k,=[k for k,row in labels.items() if word in row['occurrencePath']]
        return k
    screws={}
    for key,v in inputs['parts'].items():
        file=source/v['leaf']
        assert digest(file)==v['sha256']
        screws[key],=load_solids(file)[0].values()
    web=4.0 if web_shoe else 2.5
    beams={}
    for x in [186,224]:
        beams[f'{x}/upper']=box((x-12.5,-233,477.5),(x+12.5,233,480))
        beams[f'{x}/lower']=box((x-12.5,-233,455),(x+12.5,233,457.5))
        for n,w in enumerate([x-12.5,x+12.5-web]):
            beams[f'{x}/web{n}']=box((w,-233,457.5),(w+web,233,477.5))
    articles={};sites={}
    for label,sign in [('port',1),('starboard',-1)]:
        foot=find(label.title()+' bearing tower foot')
        sill=find(('left' if sign==1 else 'right')+' upper carriage sill')
        for x in [186,224]:
            for kind,abs_y,bottom,width,wall in [('cross',205,455,25.,web),('sill',242,450,18.,2.)]:
                y=sign*abs_y;site=f'{label}/{x}/{kind}'
                upper=f'{x}/upper' if kind=='cross' else sill
                lower=f'{x}/lower' if kind=='cross' else sill
                def mill(name,tool):
                    container=beams if name in beams else parts
                    container[name]=subtract(container[name],tool)
                def vertical(z,r,h):
                    return cylinder((x,y,z),(0,0,1),r,h)
                # A captive obround nut drops into a shallow matching pocket. Its
                # radial pilot serves the upper wall only; it ends far above the lower wall.
                mill(foot,vertical(479.,3.55,9.))
                mill(foot,capsule(x,y,485.7,16.20,10.20,2.))
                mill(upper,vertical(476.,3.55,4.2))
                mill(lower,vertical(bottom-.1,2.72,3.))
                nut=fuse(vertical(471.,3.50,14.7),capsule(x,y,485.7,16.,10.,4.0))
                if web_shoe:
                    # Relieve the antirotation neck both axially and laterally. Its
                    # lower flat does not carry the clamp; the two shoe rails above
                    # the actual tube webs do. Sill width is free along chassis X.
                    along=12.6 if kind=='cross' else 28.
                    uz=494.7;half=width/2;clear=half-wall
                    ux,uy=(half,along/2) if kind=='cross' else (along/2,half)
                    gx,gy=(clear,along/2+.1) if kind=='cross' else (along/2+.1,clear)
                    shoe=box((x-ux,y-uy,487.),(x+ux,y+uy,uz))
                    shoe=subtract(shoe,box((x-gx,y-gy,486.9),(x+gx,y+gy,487.5)))
                    neck=fuse(vertical(471.,3.50,15.2),capsule(x,y,486.2,16.,10.,1.5))
                    nut=fuse(neck,shoe)
                else:uz=489.7
                nut=subtract(nut,vertical(470.9,2.413,uz-470.9+.2))
                # Backing bar bears at the two web lines. Its middle is relieved
                # 0.5 mm from the bottom skin throughout the 25 or 18 mm span.
                transverse=width/2;length=32.
                dx,dy=(transverse,length/2) if kind=='cross' else (length/2,transverse)
                bar=box((x-dx,y-dy,bottom-7.),(x+dx,y+dy,bottom))
                inside=transverse-wall
                rx,ry=(inside,length/2+.1) if kind=='cross' else (length/2+.1,inside)
                bar=subtract(bar,box((x-rx,y-ry,bottom-.5),(x+rx,y+ry,bottom+.1)))
                bar=subtract(bar,vertical(bottom-7.1,2.65,7.3))
                under=bottom-7.
                articles[site+'/lower-bar']=bar
                articles[site+'/upper-nut']=nut
                articles[site+'/screw']=translate(screws['91251A351'],[x,y,under])
                sites[site]=dict(x=x,y=y,foot=foot,upper=upper,lower=lower,bottom=bottom,seat=under,
                                 tubeWidth=width,wall=wall,barLength=length,
                                 nutTop=uz,webShoe=web_shoe,tipMm=under+38.1,longTipMm=under+57.15)
    whole=parts|beams|articles
    print('Indexing',len(whole),'installed bodies',flush=True)
    boxes={k:bounds(v) for k,v in whole.items() if not k.endswith('/screw')}
    ref=bounds(screws['91251A351'])
    for site,s in sites.items():
        shift=[s['x'],s['y'],s['seat']]*2
        boxes[site+'/screw']=[a+b for a,b in zip(ref,shift)]
    cases={}
    for site,s in sites.items():
        x,y,z=s['x'],s['y'],s['seat'];bar=articles[site+'/lower-bar'];nut=articles[site+'/upper-nut'];screw=articles[site+'/screw']
        obs={}
        for label,shape,remove in [('bar park',bar,[site+'/lower-bar']),('nut park',nut,[site+'/upper-nut']),('screw park',screw,[site+'/screw']),
                ('long screw control',translate(screws['91251A354'],[x,y,z]),[site+'/screw']),
                ('straight underside entry',cylinder((x,y,z-82),(0,0,1),3.5,77.18),[site+'/screw']),
                ('rotational driver body below',cylinder((x,y,z-190),(0,0,1),16.,110.),[site+'/screw']),
                ('lower 40 mm withdrawal',cylinder((x,y,z-45),(0,0,1),4.0,45),[site+'/screw'])]:
            bb=boxes[site+'/screw'] if label=='screw park' else bounds(shape)
            hits={}
            for k,v in whole.items():
                if k in remove or box_gap(bb,boxes[k])>.0001:continue
                amount=common(shape,v)
                if amount>.001:hits[k]=amount
            obs[label]=hits
        cases[site]=dict(**s,lowerLandMm2=horizontal_seat(bar,whole[s['lower']],s['bottom']),
           upperNutLandMm2=horizontal_seat(nut,whole[s['foot']],487. if web_shoe else 485.7),screwHeadLandMm2=horizontal_seat(screw,bar,z),
           screwPenetrationIntoNutMm=s['tipMm']-471.,screwPointBelowNutTopMm=s['nutTop']-s['tipMm'],longTipAboveNutMm=s['longTipMm']-s['nutTop'],
           checks=obs)
        print(site,'lands',cases[site]['lowerLandMm2'],cases[site]['upperNutLandMm2'],'hits',{k:v for k,v in obs.items() if v},flush=True)
    out=dict(authority=register,archiveSha256=issue['archiveSha256'],stepSha256=digest(native),source=inputs,cases=cases,
             scope='Cold derivative plus proposed eight attachment stations. Thread envelopes use nominal major diameter. Upper nut fit, engagement, wrench, weld, strength, thread and actual assembled motion are separate acceptance duties.')
    (p/('underframe-web-shoe-fit.json' if web_shoe else 'underframe-brace-fit.json')).write_text(json.dumps(out,indent=2)+'\n')


if __name__=='__main__':check(Path(sys.argv[1]),'--web-shoe' in sys.argv[2:])
