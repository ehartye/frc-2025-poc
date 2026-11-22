"""Write field-interaction.html: timestamped physics playback of the B'x receive, carry sled and eject trials.

    python -I -B field_playback.py PROJECT_ROOT

Reads evidence/field-trials.json (the solved free-piece traces) and pocket_geometry.py (the same surfaces the
trials integrate against and build_layout.py extrudes), and writes a self-contained side-view player at scale.
Nothing here re-simulates: every frame is a recorded sample of the trial.
"""
import hashlib
import html
import json
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pocket_geometry as pg  # noqa: E402

REL = "iterations/002/design/preliminary-pocket"
src = ROOT / REL / "evidence" / "field-trials.json"
trials = json.loads(src.read_text(encoding="utf-8"))
digest = hashlib.sha256(src.read_bytes()).hexdigest()
layout = ROOT / REL / "layout.json"
layout_sha = hashlib.sha256(layout.read_bytes()).hexdigest() if layout.exists() else None
sens = {s["id"]: s for s in trials["timestepSensitivity"]}

runs = []
for p in trials["playback"]:
    r = trials[p["kind"]][p["index"]]
    s = sens[p["id"]]
    nip = (f"worm drive at {r.get('busV')} V bus, torque-limited (contact reaction fed back)" if r.get("rollers") == "motor"
           else f"imposed nip surface speed {r.get('nipMps')} m/s (ramped over 0.1 s)")
    commanded = {"eject": f"{nip}; trough at {r.get('standOffMm')} mm stand-off; lip {r.get('lip')}; robot withdraws 120 mm at 0.3 m/s once the pipe rests",
                 "receive": f"arrival {r.get('arrivalMps')} m/s on the inlet chute with rolling spin; nip {r.get('rollers')} {r.get('drive') or ''} {'' if r.get('busV') is None else str(r.get('busV')) + ' V'}",
                 "sled": f"half-sine {r.get('g')} g for 50 ms ({r.get('direction')}); nip {r.get('rollers')}"}[p["kind"]]
    metric = {"eject": dict(decision=r["outcome"], departure=r.get("departure"), maxShaftDemandNm=r.get("maxShaftDemandNm"),
                            minNipMpsInNip=r.get("minNipMpsInNip"), maxCurrentA=r.get("maxCurrentA")),
              "receive": dict(decision=r["outcome"], seatedAfterFlapS=r.get("seatedAfterFlapS"), maxRiseMm=r.get("maxRiseMm"),
                              maxBiteN=r.get("maxBiteN"), maxFlapDeg=r.get("maxFlapDeg")),
              "sled": dict(decision=r["outcome"], maxTravelMm=r.get("maxTravelMm"), maxBiteN=r.get("maxBiteN"))}[p["kind"]]
    runs.append(dict(id=p["id"], title=p["title"], kind=p["kind"], standOff=r.get("standOffMm"), lip=r.get("lip", "deployed"),
                     flap=r.get("flap", True), hood=r.get("hood", True), e=r.get("e"), mu=r.get("mu"), muTread=r.get("muTread"),
                     kTread=r.get("kTreadNmm"), commanded=commanded, metric=metric,
                     halfStep=dict(sameOutcome=s["sameOutcome"], base=s["base"], half=s["half"]),
                     trace=r["trace"]))

geom = dict(R=pg.R, inlet=pg.INLET, hood=pg.HOOD, flapHinge=pg.FLAP_HINGE, flapLength=pg.FLAP_LENGTH,
            lower=pg.LOWER_ROLLER, upper=pg.UPPER_ROLLER, rollerR=pg.ROLLER_R, coreR=pg.CORE_R,
            lip=pg.lip_profile(), lipHinge=pg.LIP_HINGE, lipStowDeg=pg.LIP_STOW_DEG, frameFront=pg.FRAME_FRONT,
            bumperFace=pg.BUMPER_FACE, bumper=[43.0, 181.0], trough=pg.TROUGH, troughBase=pg.BUMPER_FACE)
data = json.dumps(dict(runs=runs, geom=geom, source=dict(file=f"{REL}/evidence/field-trials.json", sha256=digest),
                       layout=dict(file=f"{REL}/layout.json", sha256=layout_sha), model=trials["model"]),
                  separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")

page = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>B'x field interaction</title>
<style>
:root{--blue:#0b3d91;--blue2:#1a5fd0;--gold:#f2b705;--ink:#111827;--muted:#4b5563;--bg:#f7f8fb;--card:#fff;--line:#d6dbe6;--bad:#b42318;--ok:#067647}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--ink:#e5e7eb;--muted:#9ca3af;--bg:#0b1220;--card:#111a2e;--line:#24314f}}
*{box-sizing:border-box}body{margin:0;font:14px/1.45 system-ui,Segoe UI,Roboto,sans-serif;background:var(--bg);color:var(--ink)}
header{background:var(--blue);color:#fff;padding:12px 16px;border-bottom:4px solid var(--gold)}header h1{margin:0;font-size:18px}
header p{margin:4px 0 0;font-size:12px;opacity:.9}main{display:grid;grid-template-columns:minmax(0,1fr) 340px;gap:12px;padding:12px 16px}
@media (max-width:900px){main{grid-template-columns:1fr}}
.card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px}
canvas{width:100%;height:auto;display:block;background:var(--card);border-radius:6px;touch-action:none}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:8px 0}button,select{font:inherit;padding:5px 10px;border-radius:6px;border:1px solid var(--line);background:var(--card);color:var(--ink)}
button.primary{background:var(--blue2);color:#fff;border-color:var(--blue2)}input[type=range]{flex:1;min-width:160px}
dl{display:grid;grid-template-columns:auto 1fr;gap:2px 8px;margin:0}dt{color:var(--muted)}dd{margin:0;word-break:break-word}
.tag{display:inline-block;padding:1px 6px;border-radius:4px;font-size:12px;font-weight:600}.ok{background:#d1fadf;color:var(--ok)}.bad{background:#fee4e2;color:var(--bad)}
small{color:var(--muted)}a{color:var(--blue2)}
</style></head><body>
<header><h1>Red 2 B'x pocket: field-interaction playback</h1>
<p>Recorded free-piece trials (solved pipe motion; commanded actuator inputs labelled). Side view, robot frame, +X toward the reef, millimetres.
<span id="src"></span></p></header>
<main><section class="card">
<div class="row"><label for="run">Trial</label><select id="run"></select>
<button id="play" class="primary">Play</button><button id="reset">Reset view</button>
<label for="speed">Speed</label><select id="speed"><option value="0.25">0.25x</option><option value="0.5">0.5x</option><option value="1" selected>1x</option></select></div>
<canvas id="c" width="1200" height="620" aria-label="Side view of the pocket, nip, lip, trough and the pipe"></canvas>
<div class="row"><input id="t" type="range" min="0" max="1000" value="0" aria-label="Trial time"><span id="time">0.000 s</span></div>
<small>Drag to pan, wheel or pinch to zoom, double-click or Reset to fit. Keys: space play/pause, left/right step, R reset.
Pipe ring: 114.3 mm OD; the gold mark turns with the solved spin. Nip: tread circles (dashed) and rigid cores (filled).</small>
</section>
<aside class="card" id="side"></aside></main>
<script id="data" type="application/json">DATA</script>
<script>
const D=JSON.parse(document.getElementById('data').textContent),G=D.geom;
const cv=document.getElementById('c'),cx=cv.getContext('2d'),sel=document.getElementById('run'),slider=document.getElementById('t');
document.getElementById('src').textContent=' Source '+D.source.file+' sha256 '+D.source.sha256.slice(0,12)+(D.layout.sha256?' | layout '+D.layout.sha256.slice(0,12):'');
D.runs.forEach((r,i)=>{const o=document.createElement('option');o.value=i;o.textContent=r.title;sel.appendChild(o)});
let run=D.runs[0],k=0,playing=false,last=0,acc=0,view=null;
function fit(){const xs=[-380,run.kind==='eject'?820:420],zs=[380,900];const s=Math.min(cv.width/(xs[1]-xs[0]),cv.height/(zs[1]-zs[0]));view={s:s,ox:-xs[0]*s+(cv.width-(xs[1]-xs[0])*s)/2,oz:zs[1]*s+(cv.height-(zs[1]-zs[0])*s)/2}}
const X=x=>view.ox+x*view.s,Z=z=>view.oz-z*view.s;
function css(n){return getComputedStyle(document.documentElement).getPropertyValue(n).trim()}
function poly(pts,stroke,w,fill){cx.beginPath();pts.forEach((p,i)=>i?cx.lineTo(X(p[0]),Z(p[1])):cx.moveTo(X(p[0]),Z(p[1])));if(fill){cx.closePath();cx.fillStyle=fill;cx.fill()}cx.strokeStyle=stroke;cx.lineWidth=w;cx.stroke()}
function circ(c,r,stroke,fill,dash){cx.beginPath();cx.arc(X(c[0]),Z(c[1]),r*view.s,0,7);cx.setLineDash(dash||[]);if(fill){cx.fillStyle=fill;cx.fill()}cx.strokeStyle=stroke;cx.lineWidth=1.5;cx.stroke();cx.setLineDash([])}
function rotp(p,o,deg){const a=deg*Math.PI/180,x=p[0]-o[0],z=p[1]-o[1];return[o[0]+x*Math.cos(a)+z*Math.sin(a),o[1]-x*Math.sin(a)+z*Math.cos(a)]}
function spinAngle(i){let a=0;for(let j=1;j<=i;j++){a+=run.trace[j][3]*(run.trace[j][0]-run.trace[j-1][0])}return a}
function draw(){const ink=css('--ink'),muted=css('--muted'),blue=css('--blue2'),gold=css('--gold');cx.clearRect(0,0,cv.width,cv.height);
 cx.strokeStyle=css('--line');cx.lineWidth=1;cx.beginPath();cx.moveTo(X(-400),Z(0));cx.lineTo(X(900),Z(0));cx.stroke();
 // scale cue
 poly([[-360,400],[-260,400]],ink,3);cx.fillStyle=ink;cx.font='12px system-ui';cx.fillText('100 mm',X(-360),Z(400)+16);
 const fr0=run.trace[k],ox=fr0[5]||0;cx.save();cx.translate(ox*view.s,0);
 poly([[G.frameFront,380],[G.frameFront,900]],muted,1);cx.fillStyle=muted;cx.fillText('frame face',X(G.frameFront)+4,Z(890));
 poly(G.inlet,blue,3);if(run.hood)poly(G.hood,blue,2);
 const fr=run.trace[k],phi=fr[4]||0;if(run.flap){const h=G.flapHinge,tip=rotp([h[0],h[1]-G.flapLength],h,-phi);poly([h,tip],gold,3)}
 [G.lower,G.upper].forEach(c=>{circ(c,G.rollerR,blue,null,[5,4]);circ(c,G.coreR,blue,'rgba(26,95,208,.25)')});
 const lip=run.lip==='deployed'?G.lip:(run.lip==='stowed'?G.lip.map(p=>rotp(p,G.lipHinge,G.lipStowDeg)):null);if(lip)poly(lip,gold,3);
 poly([[G.frameFront,G.bumper[0]],[G.bumperFace,G.bumper[0]],[G.bumperFace,G.bumper[1]],[G.frameFront,G.bumper[1]]],muted,1,'rgba(120,120,120,.15)');
 cx.restore();
 if(run.standOff!==null&&run.standOff!==undefined){const t=G.trough,xl=G.troughBase+run.standOff,xr=xl+t.riserX,fl=x=>t.floor0+t.slope*(x-xl);
  poly([[xl,300],[xl,t.lipTop],[xl+t.lipDepth,t.lipTop],[xl+t.lipDepth,fl(xl+t.lipDepth)],[xr,fl(xr)],[xr,t.riserTop],[xr+t.riserT,t.riserTop]],ink,2.5);
  cx.fillStyle=ink;cx.fillText('L1 trough (target.json section), stand-off '+run.standOff+' mm',X(xl),Z(300)+16)}
 // path so far
 cx.strokeStyle='rgba(242,183,5,.6)';cx.lineWidth=1;cx.beginPath();for(let j=0;j<=k;j+=2){const p=run.trace[j];j?cx.lineTo(X(p[1]),Z(p[2])):cx.moveTo(X(p[1]),Z(p[2]))}cx.stroke();
 const c=[fr[1],fr[2]];circ(c,G.R,ink,'rgba(230,180,80,.55)');circ(c,50.8,ink,css('--card'));const a=spinAngle(k);
 cx.beginPath();cx.moveTo(X(c[0]),Z(c[1]));cx.lineTo(X(c[0]+G.R*Math.cos(a)),Z(c[1]+G.R*Math.sin(a)));cx.strokeStyle=gold;cx.lineWidth=3;cx.stroke();
 document.getElementById('time').textContent=fr[0].toFixed(3)+' s | nip '+(fr[6]||0).toFixed(2)+' m/s | robot offset '+(fr[5]||0).toFixed(0)+' mm';slider.value=Math.round(1000*k/(run.trace.length-1))}
function side(){const r=run,m=r.metric,ok=(/^(captured|seated)/.test(m.decision)||(r.kind==='sled'&&/^retained/.test(m.decision)))&&!/throat/.test(m.decision);
 const rows=[['Decision',`<span class="tag ${ok?'ok':'bad'}">${m.decision}</span>`],['Kind',r.kind],['Commanded input',r.commanded],
  ['Contact',`e ${r.e}, mu ${r.mu}, tread mu ${r.muTread}, tread ${r.kTread} N/mm per roller`],['Members',`lip ${r.lip}, flap ${r.flap?'installed':'removed'}, hood ${r.hood?'installed':'removed'}`]];
 if(m.departure)rows.push(['Tip departure',`${m.departure.speed.toFixed(3)} m/s at t ${m.departure.t.toFixed(3)} s, centre Z ${m.departure.z.toFixed(1)} mm`]);
 if(m.seatedAfterFlapS!==undefined)rows.push(['Seated after flap',m.seatedAfterFlapS===null?'not seated':m.seatedAfterFlapS.toFixed(3)+' s (G1: within 0.5 s)']);
 if(m.maxRiseMm!==undefined)rows.push(['Peak rise',m.maxRiseMm+' mm (hood at 10 mm)']);
 if(m.maxTravelMm!==undefined)rows.push(['Peak travel',m.maxTravelMm+' mm from the seat']);
 if(m.maxBiteN!==undefined)rows.push(['Peak bite force',m.maxBiteN+' N (both treads)']);
 if(m.maxShaftDemandNm!==undefined&&m.maxShaftDemandNm!==null)rows.push(['Peak shaft demand',m.maxShaftDemandNm+' N m (both rollers)']);
 if(m.minNipMpsInNip!==undefined&&m.minNipMpsInNip!==null)rows.push(['Slowest nip speed in the nip',m.minNipMpsInNip+' m/s (motor current peak '+m.maxCurrentA+' A)']);
 rows.push(['Half timestep',(r.halfStep.sameOutcome?'same decision':'DIFFERENT decision')+`: ${JSON.stringify(r.halfStep.half)}`]);
 document.getElementById('side').innerHTML='<h2 style="margin:0 0 8px;font-size:15px">'+r.title+'</h2><dl>'+rows.map(([a,b])=>`<dt>${a}</dt><dd>${b}</dd>`).join('')+
 '</dl><p><small>Model: planar penalty-contact pipe with spin, stick-slip Coulomb friction, torque-limited NEO 550 through a self-locking worm (or the B planetary); robot withdraws 120 mm and the trough is assessed 2 s after rest. Not modelled: yaw, pipe-end contact, ovality, peers, lip and hood compliance, measured e, mu and tread stiffness. '+
 'Package player: <a href="package-motion.html">package-motion.html</a>.</small></p>'}
function load(i){run=D.runs[i];k=0;playing=false;document.getElementById('play').textContent='Play';fit();side();draw()}
sel.onchange=()=>load(+sel.value);
document.getElementById('play').onclick=()=>{playing=!playing;document.getElementById('play').textContent=playing?'Pause':'Play';last=performance.now()};
document.getElementById('reset').onclick=()=>{fit();draw()};
slider.oninput=()=>{k=Math.round(slider.value/1000*(run.trace.length-1));draw()};
function tick(now){if(playing){const sp=+document.getElementById('speed').value,dt=(now-last)/1000*sp;last=now;const t0=run.trace[k][0]+dt;while(k<run.trace.length-1&&run.trace[k+1][0]<=t0)k++;if(k>=run.trace.length-1){playing=false;document.getElementById('play').textContent='Play'}draw()}requestAnimationFrame(tick)}
let drag=null;cv.onpointerdown=e=>{drag=[e.clientX,e.clientY];cv.setPointerCapture(e.pointerId)};cv.onpointerup=()=>drag=null;
cv.onpointermove=e=>{if(!drag)return;const r=cv.width/cv.getBoundingClientRect().width;view.ox+=(e.clientX-drag[0])*r;view.oz+=(e.clientY-drag[1])*r;drag=[e.clientX,e.clientY];draw()};
cv.onwheel=e=>{e.preventDefault();const r=cv.width/cv.getBoundingClientRect().width,mx=e.offsetX*r,mz=e.offsetY*r,f=e.deltaY<0?1.15:1/1.15;view.ox=mx-(mx-view.ox)*f;view.oz=mz-(mz-view.oz)*f;view.s*=f;draw()};
cv.ondblclick=()=>{fit();draw()};
document.onkeydown=e=>{if(e.target.tagName==='SELECT')return;if(e.key===' '){e.preventDefault();document.getElementById('play').click()}else if(e.key==='ArrowRight'){k=Math.min(k+5,run.trace.length-1);draw()}else if(e.key==='ArrowLeft'){k=Math.max(k-5,0);draw()}else if(e.key==='r'||e.key==='R'){fit();draw()}};
load(0);requestAnimationFrame(tick);
</script></body></html>
"""
out = ROOT / REL / "field-interaction.html"
if out.exists():
    raise FileExistsError(out)
out.write_text(page.replace("DATA", data), encoding="utf-8")
print("wrote", out.relative_to(ROOT), len(runs), "runs")
