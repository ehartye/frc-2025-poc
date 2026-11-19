/* Field coordinates are metres, with +X toward the red wall and +Y upward on the map. */
const {field, runs} = window.SWERVE_REPLAY;
const $ = id => document.getElementById(id);
const canvas = $("field"), wheel = $("wheel");
const scale = Math.min((canvas.width - 38)/field.size[0], (canvas.height-30)/field.size[1]);
const px = ([x,y]) => [17+x*scale,canvas.height-15-y*scale];
function line(ctx,a,b,color,width) {
  ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
}
function polygon(ctx,points,fill,stroke) {
  ctx.beginPath();points.forEach((p,i)=>i ? ctx.lineTo(...p) : ctx.moveTo(...p));ctx.closePath();
  ctx.fillStyle=fill;ctx.fill();ctx.strokeStyle=stroke;ctx.lineWidth=1.5;ctx.stroke();
}
function robot(ctx,p,color,active) {
  const [x,y,h]=p,c=Math.cos(h),s=Math.sin(h);
  const point=(a,b)=>px([x+c*a-s*b,y+s*a+c*b]);
  polygon(ctx,[point(.425,.425),point(.425,-.425),point(-.425,-.425),point(-.425,.425)],color,active?"#d5fff3":"#78939c");
  line(ctx,point(.04,0),point(.38,0),active?"#103b35":"#dfedf1",active?4:2);
}
function drawField(run,k) {
  const ctx=canvas.getContext("2d");
  ctx.clearRect(0,0,canvas.width,canvas.height);
  const [length,width]=field.size;
  polygon(ctx,[px([0,0]),px([length,0]),px([length,width]),px([0,width])],"#243740","#9ab3b9");
  for(let x=1;x<length;x+=1)line(ctx,px([x,0]),px([x,width]),"#30434b",1);
  for(let y=1;y<width;y+=1)line(ctx,px([0,y]),px([length,y]),"#30434b",1);
  for(const p of field.planes){
    const a=p.angle*Math.PI/180,normal=[Math.cos(a),Math.sin(a)],hits=[];
    for(const x of [0,length]){const y=p.point[1]-normal[0]*(x-p.point[0])/normal[1];if(y>=0&&y<=width)hits.push(px([x,y]));}
    for(const y of [0,width]){const x=p.point[0]-normal[1]*(y-p.point[1])/normal[0];if(x>=0&&x<=length)hits.push(px([x,y]));}
    if(hits.length>=2)line(ctx,hits[0],hits[1],"#c1cdd0",2);
  }
  for(const [,point] of field.obstacles)polygon(ctx,point.map(px),"#425565","#779298");
  line(ctx,px([field.tape,0]),px([field.tape,width]),"#d2c898",1.5);
  for(const [,kind,x,y] of field.stock){
    ctx.fillStyle=kind==="floor-pipe"?"#e7c180":"#e7954a";ctx.beginPath();ctx.arc(...px([x,y]),kind==="floor-pipe"?3:6,0,Math.PI*2);ctx.fill();
  }
  const dock=px(field.station);ctx.strokeStyle="#cce7bd";ctx.lineWidth=2;ctx.beginPath();ctx.arc(...dock,6,0,Math.PI*2);ctx.stroke();
  ctx.fillStyle="#d5ebd2";ctx.font="16px system-ui";ctx.textAlign="right";ctx.fillText("Station",dock[0]-11,dock[1]-8);
  for(const [id,t] of Object.entries(field.peers))robot(ctx,t[k],id.startsWith("red")?"#a55c62":"#4b779d",false);
  const trail=run.samples.slice(0,k+1);
  ctx.strokeStyle="#90d7c6";ctx.lineWidth=2;ctx.beginPath();
  trail.forEach((r,i)=>i?ctx.lineTo(...px(r.slice(1,3))):ctx.moveTo(...px(r.slice(1,3))));ctx.stroke();
  robot(ctx,run.samples[k].slice(1,4),"#66d7ba",true);
}
function drawModules(row) {
  const ctx=wheel.getContext("2d");ctx.clearRect(0,0,wheel.width,wheel.height);
  ctx.fillStyle="#273d48";ctx.fillRect(57,23,185,220);ctx.strokeStyle="#7d9ba4";ctx.strokeRect(57,23,185,220);
  const positions=[[73,42],[227,42],[73,223],[227,223]];
  for(let i=0;i<4;i++){
    const [x,y]=positions[i],a=row[4+i],speed=row[8+i],v=[-Math.sin(a),-Math.cos(a)];
    line(ctx,[x-v[0]*16,y-v[1]*16],[x+v[0]*16,y+v[1]*16],"#b8c9d4",13);
    const tail=[x,y],tip=[x+v[0]*speed*40,y+v[1]*speed*40];
    if(Math.abs(speed)>.015){
      const c=speed>0?"#61e0bc":"#ffb761";line(ctx,tail,tip,c,4);
      const d=[Math.sign(speed)*v[0],Math.sign(speed)*v[1]];
      polygon(ctx,[tip,[tip[0]-8*d[0]-4*d[1],tip[1]-8*d[1]+4*d[0]],[tip[0]-8*d[0]+4*d[1],tip[1]-8*d[1]-4*d[0]]],c,c);
    }
    ctx.fillStyle="#deecee";ctx.textAlign="center";ctx.font="14px system-ui";
    ctx.fillText(["FL","FR","RL","RR"][i]+" "+speed.toFixed(2),x,i<2?95:269);
  }
  ctx.textAlign="center";ctx.font="12px system-ui";ctx.fillStyle="#a6b9c3";ctx.fillText("m/s",150,140);
}
let timer;
function draw() {
  const run=runs[$("case").value],k=Number($("clock").value),r=run.samples[k];
  drawField(run,k);drawModules(r);
  $("wall").value=r[0].toFixed(2)+" s";
  const state=run.events.filter(e=>e[0]<=r[0]).at(-1);
  $("event").value=state?state[1]:"Travel from the starting tape";
  $("endpoint").textContent=run.stationLockWallS===null?"The dock gate withheld permission.":("In this trial the controller confirmed the station at "+run.stationLockWallS.toFixed(3)+" s.");
  if(k===run.samples.length-1)stop();
}
function stop(){clearInterval(timer);timer=null;$("play").textContent="Play";}
$("case").addEventListener("change",()=>{stop();$("clock").value=0;draw();});
$("clock").max=runs.plus.samples.length-1;
$("clock").addEventListener("input",()=>{stop();draw();});
$("play").addEventListener("click",()=>{
  if(timer){stop();return;}
  if(Number($("clock").value)===Number($("clock").max))$("clock").value=0;
  $("play").textContent="Pause";
  timer=setInterval(()=>{$("clock").value=Number($("clock").value)+1;draw();},100);
});
draw();

