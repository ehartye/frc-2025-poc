"""Generate machinist drawings and coordinate schedules from the checked feature list."""
import csv
from html import escape
from pathlib import Path

from profiles import profiles

HERE=Path(__file__).resolve().parent


def dxf(part):
    data=[]
    def put(*fields):
        data.extend(str(v) for v in fields)
    def line(x1,y1,x2,y2):
        put(0,"LINE",8,"STOCK",10,x1,20,y1,30,0,11,x2,21,y2,31,0)
    put(0,"SECTION",2,"HEADER",9,"$ACADVER",1,"AC1015",9,"$INSUNITS",70,4,0,"ENDSEC",0,"SECTION",2,"ENTITIES")
    l,w,_=part.size
    for (x,y),(u,v) in zip([(0,0),(l,0),(l,w),(0,w)],[(l,0),(l,w),(0,w),(0,0)]):line(x,y,u,v)
    for op in part.holes:
        layer="CB_TOP" if op.depth else "THRU"
        r=op.diameter/2
        if not op.travel:
            put(0,"CIRCLE",8,layer,10,op.u,20,op.v,30,0,40,r)
        else:
            a,b=op.u-op.travel/2,op.u+op.travel/2
            for y in [op.v-r,op.v+r]:
                put(0,"LINE",8,layer,10,a,20,y,30,0,11,b,21,y,31,0)
            for x,start,end in [(a,90,270),(b,270,90)]:
                put(0,"ARC",8,layer,10,x,20,op.v,30,0,40,r,50,start,51,end)
    put(0,"ENDSEC",0,"EOF")
    return "\n".join(data)+"\n"


def diagram(p):
    length,width,depth=p.size
    scale=min(985/length,270/width)
    x0,y0=(1100-length*scale)/2,43
    h=width*scale
    bits=[f'<svg viewBox="0 0 1100 360" role="img" aria-label="{escape(p.title)} faces and marked feature centers">',
          f'<rect x="{x0}" y="{y0}" width="{length*scale}" height="{h}" fill="#edf3f7" stroke="#123d54" stroke-width="2"/>',
          f'<text x="{x0+length*scale/2}" y="{y0-13}" text-anchor="middle">U · {length:.2f} mm</text>',
          f'<text x="{x0-12}" y="{y0+h/2}" transform="rotate(-90 {x0-12} {y0+h/2})" text-anchor="middle">V · {width:.2f}</text>',
          f'<text x="{x0}" y="{y0+h+24}">(0, 0) · B/C datum intersection</text>']
    drawn=set()
    for i,item in enumerate(p.holes,1):
        x,y=x0+item.u*scale,y0+(width-item.v)*scale
        r=item.diameter*scale/2
        color='#a2491a' if item.depth else '#16485c'
        if item.travel:
            travel=item.travel*scale
            bits.append(f'<rect x="{x-r-travel/2}" y="{y-r}" width="{travel+2*r}" height="{2*r}" rx="{r}" stroke="{color}" fill="white"/>')
        else:
            bits.append(f'<circle cx="{x}" cy="{y}" r="{r}" stroke="{color}" fill="none"/>')
        if (x,y) not in drawn:
            bits.append(f'<path d="M{x-3} {y}h6 M{x} {y-3}v6" stroke="#248998" stroke-width="0.8"/>')
            drawn.add((x,y))
    return ''.join(bits)+'</svg>'


CSS="""
@page {size:A3 landscape;margin:10mm} *{box-sizing:border-box} body{margin:0;background:#d6dfe3;color:#172f3c;font:14px Arial,sans-serif}
article{width:400mm;min-height:271mm;max-width:100%;padding:10mm 13mm;margin:10px auto;background:white;break-after:page}
header{display:flex;justify-content:space-between;align-items:start;border-bottom:2px solid #163b4f;padding-bottom:10px}
h1{font-size:31px;line-height:1.13;margin:4px 0} h2{font-size:17px;color:#11445c;margin:16px 0 7px}
small{letter-spacing:.09em;text-transform:uppercase;color:#3c616e} .stamp{font:13px monospace;text-align:right}
svg{max-height:68mm;width:100%;font:14px Arial,sans-serif;color:#123d54;fill:currentColor}
.sides{display:grid;grid-template-columns:1fr 1fr;gap:16px} table{width:100%;border-collapse:collapse;font:13px monospace}
td,th{border-bottom:1px solid #c7d6dd;text-align:right;padding:4px} td:first-child,th:first-child{text-align:left}
th{background:#eaf1f4} li{margin:5px 0} footer{color:#47626f;border-top:1px solid #bfcfd6;padding-top:8px;font-size:12px;margin-top:12px}
@media print{body{background:#fff}article{padding:3mm 4mm;margin:0;max-width:none}article:last-child{break-after:auto}}
"""


def ops_table(ops):
    rows="".join(f'<tr><td>{escape(h.label)}</td><td>{h.u:.2f}</td><td>{h.v:.2f}</td><td>{h.diameter:.2f}</td><td>{h.travel:.2f}</td><td>{h.depth if h.depth else "thru"}</td></tr>' for h in ops)
    return '<table><thead><tr><th>Operation</th><th>U</th><th>V</th><th>Ø</th><th>Travel</th><th>Depth</th></tr></thead><tbody>'+rows+'</tbody></table>'


def sheet(p,index):
    l,w,t=p.size
    split=(len(p.holes)+1)//2
    details="".join('<li>'+escape(s)+'</li>' for s in p.details)
    return f"""<article><header><div><small>2025 station receiver · shop review</small>
    <h1>{escape(p.title)}</h1></div><div class="stamp">DWG {index:02d} · REV A<br>mm · diagram NTS<br>Qty {len(p.ids)} per robot</div></header>
    {diagram(p)}<div class="sides"><div><h2>Finished stock</h2>{l:.2f} × {w:.2f} × {t:.2f} mm. View along the thickness axis.
    <p>{', '.join(p.ids)}</p></div><div><h2>Datums and operations</h2><ul>{details}</ul></div></div>
    <h2>Coordinate drilling schedule · all positions from B and C</h2><div class="sides">{ops_table(p.holes[:split])}{ops_table(p.holes[split:])}</div>
    <footer>Nominals reference the blue 401-body assembly at 3fc3f27. See README tolerance and fit gates. “Travel” is distance between semicircle centers. Use listed dimensions. Sheet scale changes with printer settings.</footer></article>"""


def generate():
    parts=profiles()
    path=HERE/"patterns"
    path.mkdir(exist_ok=True)
    sheets=[]
    with (HERE/"operations.csv").open("w",newline="") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["drawing","operation","uMm","vMm","diameterMm","slotCenterTravelMm","topCounterboreDepthMm"])
        for i,p in enumerate(parts,1):
            (path/(p.key+".dxf")).write_text(dxf(p),newline="\n")
            for op in p.holes:w.writerow([p.key,op.label,op.u,op.v,op.diameter,op.travel,op.depth])
            sheets.append(sheet(p,i))
    with (HERE/"cuts.csv").open("w",newline="") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["drawing","qty","lengthUmm","widthVmm","thickTmm","tubeWallMm","perimeterEdgeRadiusMm","exactSourceIds"])
        for p in parts:w.writerow([p.key,len(p.ids),*p.size,p.wall,p.radius," ".join(p.ids)])
    (HERE/"drawings.html").write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>401-body robot shop drawings</title><style>'+CSS+'</style><main>'+''.join(sheets)+'</main></html>',encoding="utf-8",newline="\n")


if __name__=="__main__":
    generate()
