"""Estimate a folded removable cross-shaped shear deck in place of six complete through tubes."""
import json
import hashlib
import math
from pathlib import Path

HERE=Path(__file__).parent
def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def estimate():
    d=json.loads((HERE/'crossframe-budget.json').read_bytes())['studies']['pocketedAluminum']
    total=d['fieldKg']
    groups={}
    for article in d['items']:
        name=article['name']
        groups[name]=groups.get(name,0)+article['kg']
    removal={
        'continuous crossbeam':groups['continuous crossbeam']*2/3,
        'segmented longitudinal tie':groups['segmented longitudinal tie'],
        'captured end block':groups['captured end block']*10/12,
        'ten purchased end and side screws plus load washers':groups['ten purchased end and side screws plus load washers']*10/12,
        '18 grid splices, four rail ties, six miter splice joints and fastening allowance':groups['18 grid splices, four rail ties, six miter splice joints and fastening allowance']
    }
    t=3.175;rho=2680e-9;w=230.;x=689.2;y=679.2;lip=25.;flange=76.2
    area=w*(x+y)-w*w
    lipped_length=2*(x+y-2*w)
    stock=dict(crossWebKg=area*t*rho, eightEdgeReturnsKg=lipped_length*lip*t*rho, fourEndFacesKg=4*w*flange*t*rho)
    # Conserve a separate positive joint allowance. No frame qualifies on anonymous mass alone.
    provisions=dict(remainingMiterSplicesAndLowerRailTiesKg=1.35,
                    lowerOrUpperRowScrewsAndLoadWashersAtRemainingTenStationsKg=.82,
                    fourReturnCornerGussetsAndTwelveDistributedBearingSeatsKg=1.45,
                    formedEndClosestPointAllowanceKg=.22)
    field=total-sum(removal.values())+sum(stock.values())+sum(provisions.values())
    grossMass=groups['removable bumper kit, one alliance color installed']
    t_min=3.00; width=230.; edge=24.
    E=70000.;nu=.33
    rects=[(width,t_min, t_min/2)]+[(t_min,edge,t_min+edge/2)]*2
    A=sum(w*h for w,h,zc in rects); z=sum(w*h*zc for w,h,zc in rects)/A
    I=sum(w*h**3/12+w*h*(zc-z)**2 for w,h,zc in rects)
    L=max(x,y)
    load=11700.   # entire full-speed assumed reaction on one arm
    P_unguided=math.pi**2*E*(width*t_min**3/12)/L**2
    P_returned=math.pi**2*E*I/L**2
    local=4*math.pi**2*E/(12*(1-nu**2))*(t_min/width)**2
    roll=load*(177.8-114.3)/1000
    # Action of a single 3 mm panel end returning 76.2 mm vertically, before any corner gussets.
    # Finite contact spans 230 mm in this upper-bound generous width; narrower contact worsens bending.
    plain_flange=6*roll*1000/(width*t_min*t_min)
    section=dict(minimumThicknessMm=t_min,edgeReturnMm=edge,widthMm=width,developedAreaMm2=A,
       normalN=load,axialStressMPa=load/A,sectionSecondMomentMm4=I,simpleColumnUnguidedFlatCriticalN=P_unguided,
       idealPinnedChannelEulerN=P_returned,simplySupportedWebElasticBucklingMPa=local,
       flatWebCondition='Both return lines must be effectively supported. The central intersection has interrupted lips and requires an actual bearing/splice/plate support. These are ideal analytical section modes.',
       endFaceMomentAtLowerContactNm=roll,unbracedThinEndFaceStressMPa=plain_flange,minYieldMPa=160.,factor=2)
    return dict(previousBudgetSha256=sha(HERE/'crossframe-budget.json'),fieldMassFromKg=total,
       removalKg=removal,newDevelopedStockKg=stock,remainingAttachmentAllowancesKg=provisions,
       chargedElsewhere='All 12 internal steel spreaders and locating sleeves from the heavy budget are retained. One 2×3 central throughmember, two pocketed end blocks and two ten-screw end kits retained. Other ten stations require positive direct end-to-perimeter and chassis-web ties.',
       fieldMassTargetKg=field,inspectionBareTargetKg=field-grossMass-6.17-.816466,
       roomTo65Kg=65-field,roomToBareInspectionKg=52.16312255-(field-grossMass-6.17-.816466),
       assumedBodyAtModerateSpeedKg=field,
       source='https://www.atlassteels.com.au/documents/Atlas_Aluminium_datasheet_5052_rev_Oct_2013.pdf',
       section=section,decision='Retain the one crossmember and investigate folded load paths within this bounded mass. Reject an unbraced bent end: at the low contact it cannot turn the simultaneous force and moment into the top web. Size opposed diagonal end gussets, root shear and fastener bearing on two separated heights before commissioning it in a native assembly. Real bend radius, panel service/dropout, lateral buckling, corner-splice stiffness, passive bumper support, notch fatigue, pan mounting and occupied match remain open.')

if __name__=='__main__':
    r=estimate();(HERE/'folded-crossframe-screen.json').write_text(json.dumps(r,indent=2)+'\n')
    print(r['fieldMassTargetKg'],r['inspectionBareTargetKg'],r['section'])
