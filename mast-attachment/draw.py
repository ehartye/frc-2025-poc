"""Plot both plates in true proportion from their editable X/Z machining coordinates."""
from mast import HERE,RAIL,MAST,profile

def draw():
    shapes=['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="650" viewBox="0 0 1200 650" font-family="Arial,sans-serif" role="img" aria-labelledby="title desc">',
        '<title id="title">Mast bridge plate shop datums</title><desc id="desc">Left and right robot views with mast and rail hole centers, right cable relief, deck height and service travel.</desc>',
        '<rect width="1200" height="650" fill="#f4f6ef"/>']
    def text(x,y,label,size=18,color="#203039",bold=False):
        style=' font-weight="bold"' if bold else ''
        shapes.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{color}"{style}>{label}</text>')
    text(35,44,"Mast to inner rail · shop splice pair",30,bold=True)
    text(35,76,"Robot X/Z in millimetres. View each side from outside; dimensions govern. The fore/aft hole pattern is shared.")
    for s,offset in ((1,53),(-1,639)):
        body,ribs=profile(s)
        def pos(p):return (offset+44+(p[0]+136)*1.72,146+(275-p[1])*1.72)
        def poly(points,fill):
            pts=" ".join(f"{x:.1f},{y:.1f}" for x,y in map(pos,points))
            shapes.append(f'<polygon points="{pts}" fill="{fill}" stroke="#273f46" stroke-width="1.25"/>')
        text(offset,121,"LEFT  +Y" if s>0 else "RIGHT  −Y  /  cable relief",22,bold=True)
        poly(body,"#aacfc0")
        left_open=[(-108,150),(-71,216),(-71,171),(-61,150)]
        right_open=[(19,150),(-19,215),(-19,171),(-29,150)]
        for window in ([left_open] if s>0 else [])+[right_open]:
            cut=[]
            for a,b in zip(window,window[1:]+window[:1]):
                if a[1]>=194:cut.append(a)
                if (a[1]<194)!=(b[1]<194):
                    cut.append((a[0]+(b[0]-a[0])*(194-a[1])/(b[1]-a[1]),194))
            poly(cut,"#f4f6ef")
        for rib in ribs:poly(rib,"#317970")
        for pts,diam in ((RAIL,6.50),(MAST,6.50)):
            for p in pts:
                x,y=pos(p)
                shapes.append(f'<circle cx="{x}" cy="{y}" r="{diam*1.72/2}" fill="#f4f6ef" stroke="#162f38"/>')
        for z in (150,154):
            left,y=pos((-136,z));right,_=pos((69,z))
            shapes.append(f'<path d="M{left:.1f} {y:.1f} H{right:.1f}" fill="none" stroke="#794413" stroke-width="1" stroke-dasharray="5 3"/>')
        text(offset,432,"Dashed lines show the 4 mm deck.",16)
    text(35,479,"Hole centers",21,bold=True)
    text(35,510,"Rail   X −116 and +42  ×  Z128 and 141.5. Mast   X −51 and −33  ×  Z194 and 263.")
    text(35,540,"Splice Ø6.50 throughout. Mast Ø10.10 both walls for Ø10/6.20 × 25 sleeves. Rail Ø6.50 through its 6 mm web.")
    text(35,570,"Stock 8 mm 7075-T7351. Base 3 mm, ribs 8 mm total. Match the STEP contours; deburr and gauge real stock.")
    text(35,600,"Deck slots X −135…+68, |Y|235.0…238.6. Keep the original rail land. Lift the disengaged splice 35 mm.",17)
    shapes.append('</svg>')
    (HERE/"shop.svg").write_text("\n".join(shapes)+"\n",encoding="utf8",newline="\n")

if __name__=="__main__":draw()
