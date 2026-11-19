"""Produce a student-readable projection from the connector and cable coordinates."""
from pathlib import Path
import numpy as np
from routes import HERE,read,values

def run():
    p=read(HERE/"route-controls.json");k=read(HERE/"support.json")
    pieces=['<svg xmlns="http://www.w3.org/2000/svg" width="940" height="545" viewBox="0 0 940 545" role="img" aria-labelledby="title desc"><title id="title">Removing the two camera leads</title><desc id="desc">Top projection of the actual data and power curves with the removable adapter and clamps enlarged at right.</desc><style>text{font:16px sans-serif;fill:#172b37}.h{font-size:23px;font-weight:bold}.sm{font-size:13px}.data{stroke:#00759a}.power{stroke:#b14514}</style><rect width="940" height="545" fill="#f7f7f2"/>']
    def label(x,y,t,css=""):pieces.append(f'<text x="{x}" y="{y}" class="{css}">{t}</text>')
    def rect(x,y,w,h,fill):pieces.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}"/>')
    def curve(coords,css,sw):
        points=" ".join(f"{x:.2f},{y:.2f}" for x,y in coords)
        pieces.append(f'<polyline points="{points}" class="{css}" fill="none" stroke-width="{sw}" stroke-linecap="round"/>')
    label(30,36,"Connect the moving stereo camera","h")
    label(30,63,"Top view at zero stroke. Both plug shells withdraw toward the rear.")
    pieces.append('<defs><clipPath id="plan"><rect x="30" y="84" width="330" height="375"/></clipPath></defs><g clip-path="url(#plan)">')
    scale=1.06
    def top(x,y,z=0):return 260+scale*x,105-scale*y
    def shape(b,fill):
        x,y=top(b[0],b[4]);rect(x,y,scale*(b[3]-b[0]),scale*(b[4]-b[1]),fill)
    shape([-5,-207.5,862.5,10,207.5,877.5],"#d8dbd2")
    rect(249,73,27,32,"#f7f7f2")
    for name, data in k["parts"].items():
        shape(data["box_mm"],"#909a99")
    for name,col in (("data","data"),("power","power")):
        xyz=values(p[name],np.linspace(0,1,241))
        curve([top(*v) for v in xyz],col,4.4)
    shape([29.5,-45.5,842.05,57.5,45.5,859.5],"#6f7c85")
    pieces.append("</g>")
    label(31,489,"Carrier endpoint to removable frame")
    label(31,513,"Blue · USB data     Orange · independent 5 V","sm")
    # Enlarged plan of the adjacent sockets and service motion below the upper rail.
    rect(387,92,518,360,"#ffffff")
    label(409,122,"Two disconnects on the adapter","h")
    rect(765,194,109,148,"#6f7c85");label(789,277,"Head")
    rect(706,216,59,105,"#c6cfbd");label(714,273,"Y")
    rect(562,227,144,26,"#90cedc");rect(595,288,111,24,"#e5ac88")
    label(410,228,"J1 / data")
    label(410,297,"J3 / power")
    curve([(662,258),(618,258),(628,252)],"data",3);curve([(662,321),(618,321),(628,315)],"power",3)
    label(415,360,"Release the jacket clamps before pulling each plug.")
    label(415,390,"Lift the soldered transition board with its power lead.")
    label(415,420,"Support both cables outside the frame’s vertical lift.")
    label(402,488,"Inspect the physical harness at both ends of the 370 mm stroke.")
    label(402,514,"The drawing shows routing and service order. Use STEP and datums for fabrication.","sm")
    pieces.append("</svg>")
    (HERE/"routing.svg").write_text("".join(pieces)+"\n",encoding="utf8",newline="\n")
if __name__=="__main__":run()
