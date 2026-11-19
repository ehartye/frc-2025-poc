"""Render an offline, adjustable-clock field replay from the registered differential-wheel CSV."""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts import registered_transit as field


def render(input_csv,receipt_path,output):
    rows=[]
    with input_csv.open(encoding="utf-8",newline="") as src:
        prev=None
        for row in csv.DictReader(src):
            t=float(row["clock_s"])
            name=row["phase"]
            if prev is not None and t==prev[0] and name==prev[4]:
                continue
            point=[round(t,3),round(float(row["field_x_m"]),4),round(float(row["field_y_m"]),4),
                   round(float(row["heading_rad"]),5),name,round(float(row["left_mps"]),3),round(float(row["right_mps"]),3)]
            if prev is None or name!=prev[4] or t-prev[0]>=.1:
                rows.append(point)
                prev=point
    receipt=json.loads(receipt_path.read_text())
    data={"track":rows,"trips":[[p["sector_m"],p["support_confirmation_s"]] for p in receipt["trips"]],
          "static":receipt["parked"],"obs":field.obstacles(),"field":[field.PROFILE["field"][k]*field.IN for k in ("length","width")],
          "station":field.STATION,"lip":field.LIP,"duration":receipt["elapsed_s"]}
    page="""<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>REEFSCAPE Â· Station to reef</title><style>
*{box-sizing:border-box}body{margin:0;background:#f6f4ed;color:#202e30;font:16px/1.45 system-ui,sans-serif}main{max-width:1060px;margin:auto;padding:24px}h1{font-size:clamp(28px,5vw,50px);line-height:1.05;letter-spacing:-.04em;margin:0 0 12px}h2{font-size:18px}.board{background:#14262b;border-radius:12px;overflow:hidden}svg{display:block;width:100%;height:auto}.controls{padding:14px 0;display:flex;align-items:center;gap:12px;flex-wrap:wrap}button{border:1px solid #344d4f;background:#264c49;color:white;font:inherit;border-radius:5px;padding:7px 14px}input{flex:1;min-width:120px}output{font-variant-numeric:tabular-nums}p{max-width:80ch}text{font:4px system-ui;fill:#e5f2ea}.readout{display:flex;gap:20px;flex-wrap:wrap;margin:0 0 12px;color:#415552}.readout b{color:#133d3c}@media(max-width:520px){main{padding:14px}.readout{gap:8px}}
</style><main><h1>From the station to the reef</h1>
<p>Follow the red robot through a two-pipe TELEOP rehearsal. The lines show the loaded and empty routes. The other five robots hold their modeled starting poses.</p>
<div class="board"><svg id="field" viewBox="-5 -5 185 91" aria-label="Registered field, robots and routes" role="img"></svg></div>
<div class="controls"><button id="play">Play</button><input id="clock" type="range" min="0" step=".05" value="0" aria-label="Rehearsal clock"><output id="time"></output></div>
<div class="readout"><span>Phase <b id="phase"></b></span><span>Left wheel <b id="left"></b></span><span>Right wheel <b id="right"></b></span></div>
<p>Green markers show when a pipe meets the modeled sustained floor-and-lip support condition. Time, footprint, wheel limits, station and target registration come from the project experiment. This sampled replay displays assumed partner positions and uncalibrated drive/contact dynamics.</p>
<script type="application/json" id="source">DATA</script><script>
const d=JSON.parse(document.querySelector('#source').textContent),ns='http://www.w3.org/2000/svg',svg=document.querySelector('#field'),clock=document.querySelector('#clock');
const H=d.field[1],xy=(x,y)=>[x*10,(H-y)*10],s=(tag,attrs,parent=svg)=>{const n=document.createElementNS(ns,tag);for(const [k,v] of Object.entries(attrs))n.setAttribute(k,v);parent.append(n);return n;};
const polygon=(points,fill,stroke)=>s('polygon',{points:points.map(p=>xy(...p).join(',')).join(' '),fill,stroke,'stroke-width':.22});
s('rect',{x:0,y:0,width:d.field[0]*10,height:H*10,fill:'#294346',stroke:'#a6b7a8','stroke-width':.4});for(const [name,p] of d.obs)polygon(p,name.includes('reef')?'#9e9e75':'#53666a','#b8bc94');
for(const [name,p] of d.static)polygon(p,name.includes('blue')?'#5193c4':'#c17064','#e7d5bc');
for(const label of ['loaded-coral-A','loaded-coral-B','empty-first-approach','empty-return-coral-A','empty-return-coral-B']){
 const p=d.track.filter(row=>row[4]===label);s('polyline',{points:p.map(r=>xy(r[1],r[2]).join(',')).join(' '),fill:'none',stroke:label.startsWith('loaded')?'#f6cb6b':'#94cbbb','stroke-width':.48,'stroke-dasharray':label.startsWith('loaded')?'none':'1.2 .65',opacity:.84});}
for(const [label,p] of [['Red station',d.station],['L1 trough',d.lip]]){const [x,y]=xy(...p);s('text',{x:x-2,y:y+6,'text-anchor':'end'}).textContent=label;}
const pipes=d.trips.map(([off,t])=>{const [x,y]=xy(d.lip[0]+.065,d.lip[1]+off);return [s('line',{x1:x,y1:y-1.51,x2:x,y2:y+1.51,stroke:'#9cf7a8','stroke-width':1.12}),t];});
const robot=s('g',{});s('rect',{x:-4.25,y:-4.25,width:8.5,height:8.5,rx:.6,fill:'#f18768',stroke:'#fff1cf','stroke-width':.45},robot);s('path',{d:'M 1 -1.8 L 3.5 0 L 1 1.8',fill:'none',stroke:'#331d18','stroke-width':.6},robot);
clock.max=d.duration;const time=document.querySelector('#time'),play=document.querySelector('#play');let running=false,last,cursor=0;
function draw(){const t=Number(clock.value);let a=0,b=d.track.length-1;while(a<b){const m=Math.ceil((a+b)/2);if(d.track[m][0]<=t)a=m;else b=m-1;}const r=d.track[a],q=d.track[Math.min(a+1,d.track.length-1)],f=r[4]===q[4]&&q[0]>r[0]?Math.min(1,(t-r[0])/(q[0]-r[0])):0;const lerp=i=>r[i]+f*(q[i]-r[i]),[x,y]=xy(lerp(1),lerp(2));robot.setAttribute('transform','translate('+x+' '+y+') rotate('+(-lerp(3)*180/Math.PI)+')');time.value=t.toFixed(2)+' s';document.querySelector('#phase').textContent=r[4].replaceAll('-',' ');document.querySelector('#left').textContent=lerp(5).toFixed(2)+' m/s';document.querySelector('#right').textContent=lerp(6).toFixed(2)+' m/s';for(const [p,show] of pipes)p.setAttribute('visibility',t>=show?'visible':'hidden');}
function advance(now){if(!running)return;cursor=Math.min(d.duration,cursor+(now-last)/1000);clock.value=cursor;last=now;draw();if(Number(clock.value)>=d.duration-.05){running=false;play.textContent='Play';}else requestAnimationFrame(advance);}
play.onclick=()=>{running=!running;if(running&&+clock.value>=d.duration-.05)clock.value=0;play.textContent=running?'Pause':'Play';cursor=Number(clock.value);last=performance.now();if(running)requestAnimationFrame(advance);};clock.oninput=()=>{cursor=Number(clock.value);draw();};draw();
</script></main></html>"""
    raw=json.dumps(data,separators=(",",":"),allow_nan=False)
    output.write_text(page.replace("DATA",raw),encoding="utf-8",newline="\n")


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("input_csv","receipt","output"):
        p.add_argument(name,type=Path)
    args=p.parse_args()
    render(args.input_csv,args.receipt,args.output)
