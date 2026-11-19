"""Student shop drawings and 1:1 planar layouts in millimetres, projected from the registered native design."""
from pathlib import Path
import csv,html,json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
# The common shelf coordinate system U=X+100, V=Y+210 starts at the right-rear corner.
OPS=[
    ("Pipe sight port",[140,200],[210],12,"THRU","Existing optical axes"),
    ("Shelf to receiver",[20,240],[30,390],5.3,"THRU","Existing M5 clearance"),
    ("Shelf counterbore",[20,240],[30,390],9.2,"3 deep TOP","Match existing M5 stack"),
    ("Saddle boss",[79,102.5,126],[10,410],8.2,"THRU","All six steel bosses enter from underside"),
    ("Saddle keeper screw",[88,117],[10,410],3.4,"THRU","M3 retainers stay with shelf"),
    ("Keeper countersink",[88,117],[10,410],6,"90 deg TOP","Head flush to 0.10 below local surface"),
]
def drawing_svg(title,views,width=820,height=355):
    return f'<svg role="img" aria-label="{html.escape(title)}" viewBox="0 0 {width} {height}"><title>{html.escape(title)}</title>'+''.join(views)+'</svg>'
def line(a,b,c,d,klass="cut"):
    return f'<line class="{klass}" x1="{a}" y1="{b}" x2="{c}" y2="{d}"/>'
def rect(a,b,w,h,klass="cut"):
    return f'<rect class="{klass}" x="{a}" y="{b}" width="{w}" height="{h}"/>'
def circle(x,y,r,klass="cut"):
    return f'<circle class="{klass}" cx="{x}" cy="{y}" r="{r}"/>'
def txt(x,y,label):
    return f'<text x="{x}" y="{y}">{html.escape(str(label))}</text>'
def dim(x1,y1,x2,y2,label):
    return line(x1,y1,x2,y2,"dim")+line(x1-3,y1-3,x1+3,y1+3,"dim")+line(x2-3,y2-3,x2+3,y2+3,"dim")+txt((x1+x2)/2+4,(y1+y2)/2-6,label)
def table(headers,rows):
    return '<table><thead><tr>'+''.join('<th>'+html.escape(str(h))+'</th>' for h in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(str(c))+'</td>' for c in r)+'</tr>' for r in rows)+'</tbody></table>'
def dxf(outlines,features,path):
    out=[]
    def put(*x):out.extend(map(str,x))
    put(0,"SECTION",2,"HEADER",9,"$ACADVER",1,"AC1015",9,"$INSUNITS",70,4,0,"ENDSEC",0,"SECTION",2,"ENTITIES")
    for points in outlines:
        for (x,y),(x2,y2) in zip(points,[*points[1:],points[0]]):
            put(0,"LINE",8,"OUTLINE",10,x,20,y,30,0,11,x2,21,y2,31,0)
    for x,y,d,layer in features:
        put(0,"CIRCLE",8,layer,10,x,20,y,30,0,40,d/2)
    put(0,"ENDSEC",0,"EOF")
    path.write_text("\n".join(out)+"\n",encoding="ascii",newline="\n")

def run():
    sections=[]
    def page(title,body):
        sections.append('<section><header>FIXED-LOW CAMERA SUPPORT · SHOP CANDIDATE</header><h1>'+title+'</h1>'+body+'<footer>All dimensions in mm · 2025 POC · '+str(len(sections)+1)+'/5 · Use dimensions and verified datums for manufacture.</footer></section>')
    # The full plan is laid landscape with +U vertical and +V horizontal for the wide shelf.
    plan=[rect(105,40,420,265),dim(105,322,525,322,"420 along V"),dim(60,40,60,305,"265 U"),
          txt(108,28,"Front U265 · upper edge R6"),txt(106,345,"C: V0 right edge"),txt(540,300,"B: U0 rear edge"),
          txt(555,80,"Facing the top of the shelf"),txt(555,105,"Dark circles: through features"),txt(555,130,"Orange: top recess or countersink")]
    for kind,us,vs,d,depth,desc in OPS:
        for u in us:
            for v in vs:plan.append(circle(105+v,305-u,d/2,"op" if "TOP" in depth or "deep" in depth else "cut"))
    plot=drawing_svg("Top plan of the new receiver shelf",plan)
    rows=[(name,", ".join(map(str,us)),", ".join(map(str,vs)),f"Ø{d:g}",depth) for name,us,vs,d,depth,_ in OPS]
    page("01 · Replacement receiver shelf",plot+table(("Operation","U","V","Finish","Depth / side"),rows)+
         '<p>Cut one 265 × 420 × 15 blank in the project PE candidate. A is the underside, B the square rear face and C the right side. U runs forward, V toward robot left. The upper front R6 is tangent at U259 on the top and 9 above A at the front. Break burrs without rounding the datum edges or ports. Inspect PE grade, thermal movement, exposure, friction and creep on the actual material.</p><p>General ±0.20. Machine all six Ø8.20 +0.05/0 bores from A in one setup with centers within ±0.04 per axis. Inspect the finished boss pads against the top surface. Use a new blank. The prior +26/+29 holes overlap.</p>')
    plan=[rect(60,124,590,120),dim(60,265,650,265,"59"),dim(26,124,26,244,"12"),
          txt(62,98,"Underside rail · drawing origin at its right-rear corner")]
    for x,dia in ((6,8),(15,3),(29.5,8),(44,3),(53,8)):
        plan.append(circle(60+10*x,184,dia*5));plan.append(txt(50+10*x,293,x))
    plan += [rect(62,40,295,16),rect(82,12,40,28),rect(197,12,40,28),rect(317,12,40,28),txt(398,35,"Section: bosses 15 above 4-thick rail"),
             txt(398,54,"Center peg Ø6 × 2 above the middle boss")]
    page("02 · Threaded saddle and foot datums",drawing_svg("One of two machined saddle bars",plan)+
         table(("Quantity / stock","Machine feature","Functional tolerance"),[
         ("2 steel saddles, 59 × 12 × 21 rough","Base 59 × 12 × 4. Ø8 bosses on the centerline at U6, 29.5, 53. Outside bosses tapped M4 × 0.7 ≥7 usable. Center has Ø6 locator, 1.8 full + 0.2 lead chamfer.","Boss Ø8.00 −0.02/0. Locator Ø6.00 −0.01/0. Boss pitch ±0.03. Match boss height to each finished PE shelf."),
         ("Both bars","Tap M3 × 0.5 through base at U15, 44. Install from the underside. Four M3 × 20 countersunk screws retain both bars while the frame is off.","Installed metal clamp pads flush to +0.04 relative to local PE, coplanarity ≤0.04. Inspect for rocking and mark each side."),
         ("Left foot, round","Ø6.08 +0.02/0 through at local (32.5,9); mates with left 6 mm peg.","Right foot slot 6.08 +0.02/0 wide × 8.08 overall; two end centers 2.00 apart along Y. Both centerlines ±0.03."),
         ("2 feet, 65 × 18 blanks","3 thick. Bolt clearance Ø4.5 at local U9,56 on V9, measured from untrimmed blank. Trim 1.00 from inboard edge full length. From U41 to U65 trim an additional 2.00.","Mirror both cuts. Edges finish at |Y|192 and 194. Inside corner R0.5 max. Keep weld toes inside the inspected outline; inspect grip and dashpot sweep."),
         ])+'<p>Center boss shoulder ends at A + 15. Do not clamp through polymer. Inspect the actual lot and machine to the measured thickness. Rebuilding both bars and feet in one fixture is the controlling step for the datum pattern.</p>')
    view=[rect(70,136,235,16),rect(70,154,235,75,"pe"),rect(70,230,235,20,"cut"),rect(139,154,44,75,"cut"),
          rect(130,70,62,66),rect(139,73,44,59,"pe"),circle(147,82,4),circle(174,82,4),
          rect(145,117,32,19),rect(151,136,20,39),txt(320,84,"Two Ø1.5 cross pins keep the M4 head inside the ear pocket."),
          txt(320,122,"Ø7 head, 3 mm AF drive; insert before cross-drilling and pinning."),txt(320,151,"The head seats on the 3 mm aluminum foot."),
          txt(320,188,"Ø8 steel boss passes through the shelf and closes the preload loop."),
          txt(320,231,"The tapped base stays with the shelf during service."),txt(70,291,"Section through one main attachment · dimensions below govern")]
    page("03 · Captive screws and camera plate",drawing_svg("Enlarged captive fastener section",view,height=315)+
         table(("Member","Dimensions and operations","Assembly / inspection"),[
         ("Four foot ears","Integral square 10.8 centered around each M4. Floor at foot top. Cavity Ø8.30 through ear. Height 13. Two horizontal Ø1.5 nominal pin bores along Y at X±2.75, Z+11 above floor.","Hold inner opening 4.00 nominal, 3.85 minimum. Check a Ø3.60 gauge in the 3 mm hex drive corridor. Keep head travel ≥5.10. Use four M4 × 8 socket screws."),
         ("Camera plate, 1 × 6061","65 X × 86 Y × 3 base, origin X−5,Y−43. Through window local U19…41, V16…70. Two Ø4.5 holes U53, V5.5 and 80.5. Keep upper rib lands U15…51, V9…13 and 73…77, each 3 high.","Centered keeper ears 10.8 square, height 10.5 above plate top. Bore Ø8.30. Pin axes X±2.75 and Z+9 above plate top. Cut and finish two M4 screws to 6.50 underhead. Verify ≥3.60 free unscrew travel."),
         ("12 steel pins","Nominal Ø1.5 × 10.8 flush. Model represents the installed stock envelope. Fit the bore to the selected spring-pin article and sample coupon.","Keep pins parallel, flush ±0.10. Inspect for axial walk after vibration and repeated key insertion. Replace damaged pins and verify capture by pulling each released screw."),
         ])+'<p>Threads are conventional major-diameter CAD envelopes. Cut taps to the drawing callouts. The camera screw has 3.5 mm nominal engagement. Verify delivered thread depth, bottom clearance, drive size and head dimensions before fitting the captors. Both camera and frame can be removed without loose screws.</p>')
    v=[rect(100,260,80,7),rect(620,260,80,7),rect(135,65,18,195),rect(650,65,18,195),rect(135,51,533,14),
       rect(340,65,103,7),rect(352,72,79,17),dim(144,295,659,295,"400 between post axes"),dim(75,64,75,260,"359.5 cut"),
       txt(270,35,"Upper crossbar: 415 × 15 × 15"),txt(260,178,"Pupil: X48, Y0, Z842 at slide 0"),txt(260,201,"Baseline along Y · optical axis downward")]
    page("04 · Welded bridge and service",drawing_svg("Front elevation of the removable bridge",v,height=325)+
         table(("Stock and datum","Fabrication and acceptance"),[
         ("2 vertical posts","15 × 15 × 2 aluminum square tube, 359.5 long; outer X −5…+10. Center Y ±200. End square ≤0.05. Pair stock and machine/weld with pads on one fixture."),
         ("1 crossbar","15 × 15 × 2, length 415, left/right overhang 7.5 from post centerlines. Bottom Z862.5, top Z877.5. Seat both posts at Z503. Use a qualified aluminum joining procedure; inspect heat-affected stock, joint play and distortion."),
         ("Camera mount","Plate underside Z859.5. Optical housing envelope X34…62, Y±45.5, Z842…859.5. Two mount holes separated by 75. Do not locate the pupils by the camera cover alone. Align from the two-plane optical measurement."),
         ("Bench and removal","Install the 12 crosspins only after the 6 screws. Fit and witness the 4 permanent M3 retainers before attaching the 4 M4 frame screws. Verify the round left and relieved right locators. Support and disconnect the loom before lifting. Remove the camera on a padded bench."),
         ])+'<p>Model origin in robot coordinates at the zero-stroke fixed-low pose. Keep the field-facing footprint, service headroom and cable bend clear. Crown loading, damaged bearings, heat-affected weld allowable and as-built shelf twist require fixture verification.</p>')
    target=[rect(70,40,340,220)]
    for u,v in ((15,15),(95,15),(15,135),(95,135),(26,46)):target.append(circle(70+v*2.26,260-u*2,4))
    target+= [txt(435,70,"110 × 150 × 3 matte target"),txt(435,101,"U along X · V along Y"),txt(435,132,"Center at X48, Y0"),txt(435,163,"Measure top at Z525 and Z685"),txt(435,194,"Use the same target at the two heights")]
    page("05 · Independent calibration target",drawing_svg("Two-height metrology plate",target,height=277)+
         table(("Feature","Prepare and verify"),[
         ("Dot pattern","Five Ø4 high-contrast circular fiducials on the top. Measured centers in target U,V: (15,15), (95,15), (15,135), (95,135), (26,46). Verify target dimensions and flatness against independent shelf A/B/C."),
         ("Bench setup","Position target plate with its rear/right corner at X−7,Y−75. Support independently, leaving it visible from both pupils. Use one measured plate at a time. Measure its six-coordinate pose at both heights; an unsurveyed printed scale supplies no calibration."),
         ("Measurements","Record a distortion-corrected image of all five identified points in each eye per height. Survey the two plate elevations, XY origin, yaw and scale. Bound the fitted metric coordinates at each evaluation ray, including target and pixel uncertainty."),
         ("Load / repeat","Repeat on reseat, at the intended cable routing and at 0/185/370 stroke. Repeat under an independently constrained two-gravity equivalent in both horizontal directions. Record reference patch, six-coordinate residual and uncertainty for each run."),
         ])+'<p>See the metrology procedure for error budgets, computed hold cases and release criteria. These drawings establish a testable fixture and interface; inspection records require actual equipment, observations and a calibrated uncertainty budget.</p>')
    style='''<style>@page{size:A4 landscape;margin:10mm}*{box-sizing:border-box}body{margin:0;background:#e7ebee;color:#122634;font:14px/1.35 Arial,sans-serif}section{position:relative;max-width:1120px;margin:18px auto;padding:22px 35px;background:white;break-after:page}header,footer{font-size:10px;letter-spacing:.06em;color:#52636a}h1{font-size:27px;margin:.2em 0 .35em}svg{width:100%;max-height:290px}.cut,.op,.dim{fill:none;stroke:#194b60;stroke-width:1.6}.op{stroke:#ad5a14}.dim{stroke:#72818a;stroke-width:1}.pe{fill:#dde8e7;stroke:#386565}svg text{font:13px Arial;fill:#193c4d}table{border-collapse:collapse;width:100%}td,th{text-align:left;vertical-align:top;border-bottom:1px solid #ccd8dc;padding:4px 7px}th{background:#e5edef}p{margin:.55em 0}footer{margin-top:9px}@media print{body{background:#fff;font-size:10.5px}section{max-width:none;margin:0;padding:0}svg{max-height:215px}h1{font-size:21px}section:last-child{break-after:auto}}@media(max-width:650px){section{padding:12px}svg{min-width:460px}section{overflow:auto}table{font-size:12px}}</style>'''
    (HERE/"drawings.html").write_text('<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Camera support shop drawings</title>'+style+''.join(sections)+'</html>\n',encoding="utf8",newline="\n")
    holes=[(u,v,d,("COUNTERBORE" if "counterbore" in label else "COUNTERSINK" if "countersink" in label else "THRU")) for label,us,vs,d,_,_ in OPS for u in us for v in vs]
    dxf([[(0,0),(265,0),(265,420),(0,420)]],holes,HERE/"shelf-layout.dxf")
    dxf([[(0,0),(110,0),(110,150),(0,150)]],[(x,y,4,"DATUM_DOT") for x,y in ((15,15),(95,15),(15,135),(95,135),(26,46))],HERE/"calibration-target.dxf")
    with (HERE/"shelf-operations.csv").open("w",newline="",encoding="ascii") as f:
        wr=csv.writer(f,lineterminator="\n");wr.writerow(("operation","U_mm","V_mm","diameter_mm","depth","note"))
        wr.writerows((label,u,v,d,depth,note) for label,us,vs,d,depth,note in OPS for u in us for v in vs)
if __name__=="__main__":run()
