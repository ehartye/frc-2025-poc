"""Reproduce all 24 catalog intersections and identify every material in the native fixture."""
import hashlib
import itertools
import json
import math
import re
import sys
from pathlib import Path
import numpy as np
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform

ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'artifacts/split-drop-native';HERE=Path(__file__).resolve().parent
def read(p):return json.loads(p.read_bytes())
def ref(p):return dict(file=p.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def write(p,q):p.write_text(json.dumps(q,indent=2)+'\n')
def main(plugin):
    sys.path.insert(0,str(plugin/'scripts'))
    from showcase_contacts import load_solids
    from mechanical_motion import bounds,common
    spec=read(ROOT/'cad/fusion-authority.json')['source']['archive']
    if ref(ROOT/spec['file'])!=spec:raise ValueError('Changed full engineering authority')
    receipt=read(OUT/'r4-load-01.json');archive=OUT/'red2-split-drop-fixture-r4.f3d'
    if ref(archive)['sha256']!=receipt['sha256']:raise ValueError('Changed native cassette')
    sup=OUT/'supplier';base=read(OUT/'r4-home-native.json')
    raw={
       'REV-29-2947':('REV-29-2947.STEP','Alloy steel','Black oxide','rev-button-screw-drawing.pdf','https://www.revrobotics.com/content/docs/10-32-Button-Head-Cap-Screws-DR.pdf'),
       'REV-29-2951':('REV-29-2951.STEP','Alloy steel','Black oxide','rev-button-screw-drawing.pdf','https://www.revrobotics.com/content/docs/10-32-Button-Head-Cap-Screws-DR.pdf'),
       'REV-29-2959':('REV-29-2959.STEP','Alloy steel','Black oxide','rev-button-screw-drawing.pdf','https://www.revrobotics.com/content/docs/10-32-Button-Head-Cap-Screws-DR.pdf'),
       'REV-29-1016':('REV-29-1016.STEP','Zinc-plated steel shell and nylon insert','Zinc on shell','rev-locknut-page.html','https://www.revrobotics.com/REV-29-1016/'),
       'WCP-0291':('WCP-0291.step','6061-T6 aluminum','Surface finish unverified','wcp-gear-racks.csv','https://wcproducts.com/products/gear-racks'),
       'WCP-0290':('WCP-0290.step','4140 steel','Surface finish and temper unverified','wcp-steel-hex-bore-gears.csv','https://wcproducts.com/products/steel-hex-bore-gears'),
       'REV-41-1620':('REV-41-1620.STEP','7075 aluminum','Black anodized','REV-41-1620-DR.pdf','https://www.revrobotics.com/content/docs/REV-41-1620-DR.pdf')}
    sources={};solids={};body_rows=[]
    for sku,(file,mat,finish,evidence,url) in raw.items():
        src=sup/file
        sources[sku]=dict(article=ref(src),source=ref(sup/evidence),provenance=url,material=mat,finish=finish,
                          sharing='Private integration. Redistribution rights unverified.')
        if sku.startswith('REV-29-'):solids[sku]=list(load_solids(src)[0].values())
    readback=receipt['native']['materialReadback']
    for body in base['bodies']:
        key=body['path']+'/'+body['name']
        sku=next((s for s in sources if s in key),None)
        if sku:
            article=sources[sku];expected=article['material']
            if sku=='REV-29-1016':
                expected={'Body1':'Nylon locking insert; polymer grade unverified','Body2':'Zinc-plated steel shell; grade and coating thickness unverified'}[body['name']]
            body_rows.append(dict(nativeIdentity=key,sku=sku,assigned=readback[key],expected=expected))
    if len(body_rows)!=76:raise ValueError('Every installed supplier body must be classified')
    write(HERE/'purchased-materials.json',dict(archive=ref(archive),native=ref(OUT/'r4-load-01.json'),source=spec,
        byArticle=sources,componentBodies=body_rows,
        insertIdentification=dict(sourceSolid='REV-29-1016 [1.1]',fusionBody='Body1',volumeMm3=6.720920867,externalRadiusMm=3.040126,
                                  axialIntervalMm=[3.714242,4.137152],basis='Only the terminal annular insert in the native manufacturer file.'),
        shellIdentification=dict(sourceSolid='REV-29-1016 [1.2]',fusionBody='Body2',volumeMm3=265.159050548,acrossFlatsMm=9.525,thicknessMm=4.365752),
        limits='76 Steel readbacks include 27 wrong material families. 49 other steel assignments still lack the item-specific designation. Supplier grades for the shell and insert remain unpublished.'))
    def group(path):
        label=path.split('+')[1].split(' REV-')[0]
        for a,b in [('Keeper screw','Keeper locknut'),('Captive jaw travel stop','Jaw stop retaining nut'),
                    ('jaw screw','jaw locknut'),('rack screw','rack locknut')]:label=label.replace(a,b)
        return label
    def transform(s,m):
        tr=gp_Trsf();tr.SetValues(*m[:3,:].reshape(12));return BRepBuilderAPI_Transform(s,tr,False).Shape()
    refseat=4.365752
    lengths={'REV-29-2947':19.05,'REV-29-2951':25.4,'REV-29-2959':50.8}
    report=[];cache={}
    for case in ('home','present','clear','before-release'):
        data=read(OUT/('r4-'+case+'-native.json'));b=data['bodies'];rows={}
        for test in data['contacts']['pairs']:
            if max(test['interferenceMm3'],test['temporaryCommonMm3'])<=0:continue
            a,n=[b[i] for i in test['pair']]
            if 'REV-29-1016' in a['path']:a,n=n,a
            sku=next((k for k in lengths if k in a['path']),None)
            if not sku or 'REV-29-1016' not in n['path'] or group(a['path'])!=group(n['path']):raise ValueError('Unexpected or mismatched hardware pair')
            label=group(a['path'])
            screw=np.asarray(a['worldTransform']).reshape(4,4);nut=np.asarray(n['worldTransform']).reshape(4,4)
            relative=np.linalg.inv(nut)@screw
            axis=-relative[:3,2]
            if np.linalg.norm(axis-[0,1,0])>1e-7:raise ValueError('Fastener skew')
            tip=bounds(solids[sku][0])[2]
            tip_center=relative@np.array([0,0,tip,1])
            head_center=relative@np.array([0,0,tip+lengths[sku],1])
            if np.hypot(tip_center[0],tip_center[2])>.005:raise ValueError('Fastener offset')
            mate=rows.setdefault(label,dict(label=label,screw=a['path'],nut=n['path'],sku=sku,
                relativeScrewToNutMm=relative.tolist(),stackMm=-head_center[1],engagementMm=min(refseat,tip_center[1]),tipBeyondNutMm=tip_center[1]-refseat,contacts=[]))
            index={'Body1':0,'Body2':1}[n['name']]
            identity=(sku,index,tuple(np.round(relative.flatten(),5)))
            if identity not in cache:cache[identity]=common(transform(solids[sku][0],relative),solids['REV-29-1016'][index])
            v=cache[identity]
            if max(abs(v-test['interferenceMm3']),abs(v-test['temporaryCommonMm3']))>.02:raise ValueError('Mismatch with independent source thread volume')
            mate['contacts'].append(dict(nutBody=n['name'],sourceOverlapMm3=v,designInterferenceMm3=test['interferenceMm3'],temporaryCommonMm3=test['temporaryCommonMm3']))
        if len(rows)!=24 or not all(len(r['contacts'])==2 and r['tipBeyondNutMm']>=.79375 for r in rows.values()):raise ValueError('Missing engagement or full pitch')
        report.append(dict(pose=case,receipt=ref(OUT/('r4-'+case+'-native.json')),mates=list(rows.values())))
    write(OUT/'r4-qualified-catalog-mates.json',dict(source=spec,archive=ref(archive),materialCatalog=ref(HERE/'purchased-materials.json'),withheld=False,
         purpose='Installed relative poses and exact source geometries reproduce all 48 intentional catalog intersections per pose. Native screw envelopes are smooth major diameter. The thin nylon ring and steel thread minor bore each intersect that envelope.',
         toleranceMm3=.02,minimumProjectionMm=.79375,pitchMm=.79375,limits='This qualifies the named CAD contact and thread coverage only. Installation torque, repeated nylon retention, installed tool access, damage and connector loosening still need separate verification.',poses=report))
    print(len(body_rows),'supplier solids;',len(cache),'source Boolean cases,',len(report)*24,'pose-mates')
if __name__=='__main__':main(Path(sys.argv[1]).resolve())
