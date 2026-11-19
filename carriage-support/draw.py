"""Draw the support in the same robot coordinates as the released machining exchange."""
from support import HERE,OUTLINE,OPEN,REAR_WINDOW,BOLTS

def draw():
    ink='#19343d';teal='#246956'
    rows=['<svg xmlns="http://www.w3.org/2000/svg" width="1160" height="690" viewBox="0 0 1160 690" font-family="Arial,sans-serif" role="img" aria-labelledby="t d">',
          '<title id="t">Fixed receiver carriage support machining datums</title><desc id="d">Paired bracket profile with mast hole axes, sectioned front and rear interfaces, and forward service travel.</desc>',
          '<rect width="1160" height="690" fill="#f8f7ef"/>']
    def text(x,y,s,size=18,color=ink):rows.append(f'<text x="{x}" y="{y}" fill="{color}" font-size="{size}">{s}</text>')
    def side(x,z):return 52+(x+105)*2.32,136+(437-z)*2.32
    def poly(loop,color):
        pts=' '.join('%0.1f,%0.1f'%side(x,z) for x,z in loop)
        rows.append(f'<polygon points="{pts}" fill="{color}" stroke="{ink}" stroke-width="1.3"/>')
    def dot(x,z,r):
        a,b=side(x,z);rows.append(f'<circle cx="{a:.1f}" cy="{b:.1f}" r="{r*2.32}" fill="#f8f7ef" stroke="{ink}"/>')
    text(32,43,'Fixed receiver carriage support',31)
    text(32,75,'Robot millimetres · X forward, Y left, Z up · mirror the mate about Y0')
    text(32,111,'Side view at the inboard mast face',20)
    poly(OUTLINE,'#b9dac8');poly(OPEN,'#f8f7ef');poly(REAR_WINDOW,'#f8f7ef')
    poly([(-31,341),(167,417),(159,420),(-34,347)],teal)
    for x,z in BOLTS:dot(x,z,3.25)
    text(720,147,'Forward bridge seat',21)
    text(720,180,'X167, |Y|182 and 197, top Z438')
    text(720,210,'Ø5.50 through, 90° seat to Ø11.00.')
    text(720,240,'Set the actual head flush and in contact.')
    text(720,292,'Rear crossbar seat',21)
    text(720,325,'X−87, |Y|196. Pad bears at Z412.')
    text(720,355,'M6 × 1 from below, full thread ≥12 mm.')
    text(32,440,'Mast rows',21)
    text(32,470,'X−53 and −35 × Z350 and 399. Drill both mast walls Ø10.10 for Ø10/6.20 sleeves, nominal length 25.')
    text(32,500,'Bracket holes Ø6.50. The two pads register on the inboard mast face at |Y|210. Gauge with both masts supported.')
    text(32,550,'Remove the four mast bolts, rear screw and two flush front screws per side. Leave both original height locks in place.')
    text(32,582,'After confirming independent support, draw the unbolted plate 535 mm forward. Reverse this path for installation.')
    text(32,632,'Use the paired STEP solids for the rib, radii, lands and local socket relief. Do not scale the drawing.',17)
    rows.append('</svg>');(HERE/'shop.svg').write_text('\n'.join(rows)+'\n',encoding='utf8',newline='\n')
if __name__=='__main__':draw()
