// A fresh coupled control experiment using the frozen finite contact surfaces at their source clock.
import {readFileSync,writeFileSync,existsSync} from 'node:fs';
import {createHash} from 'node:crypto';
const sha=b=>createHash('sha256').update(b).digest('hex');
const read=p=>JSON.parse(readFileSync(p));
const root='iterations/002/design/strategy/';
const source=readFileSync('cad/red2-split-drop/single_pivot_spatial.mjs');
if(sha(source)!=='7a921f81ed82efc811b15e8b53a4cb752581594c8cc8f7d38baf1e91187f1fd2')throw Error('Historical contact solver changed');
const changedDock=process.argv.includes('--stand-off-12');
const inset=process.argv.includes('--inset-perimeter');
const scheduleFile=root+(inset?'inset-route-12mm.json':changedDock?'pan-route-12mm.json':'pan-route-schedule.json');
const schedules=read(scheduleFile);
const shop=read('artifacts/red2-integrated/pan-mechanics.json');
let body=source.toString().split('const mode=process.argv[2]')[0];
function replace(needle,change){
  if(body.split(needle).length!==2)throw Error('Lost unique source site '+needle);
  body=body.replace(needle,change);
}
replace('const theta=settings.stowDeg*deg+sweep*u*u*(3-2*u),rate=u===0||u===1?0:sweep*6*u*(1-u)/duration;',
 `const theta=g.q,rate=g.v; const target=settings.stowDeg*deg+sweep*u*u*(3-2*u),vr=u===0||u===1?0:sweep*6*u*(1-u)/duration;`);
replace('const original=source[scenario],clock=source.clocks[scenario][id],adverse=scenario===\'adverse\';',
 `const original=source[scenario],clock=source.clocks[scenario][id],adverse=scenario==='adverse',g=globalThis.panControl;`);
replace('const elapsed=t-clock.carry;force[0]+=M*(adverse?1.5:.9)*(elapsed>0&&elapsed<.7?-1:elapsed>.9&&elapsed<1.6?1:0);',
 `const elapsed=t-clock.carry,e=g.event(t), spin=e.w, angularAccel=e.a;
 const inertialX=-e.x+angularAccel*p[1]+spin*spin*p[0]+2*spin*v[1];
 force[0]+=M*inertialX; force[1]+=M*(-angularAccel*p[0]+spin*spin*p[1]-2*spin*v[0]);`);
replace('const w=add(scale(L,1/J),a,dot(L,a)*(1/I-1/J));','const w=add(scale(L,1/J),a,dot(L,a)*(1/I-1/J));w[2]-=spin;');
replace('const driveAccel=(adverse?1.5:.9)*(elapsed>0&&elapsed<.7?-1:elapsed>.9&&elapsed<1.6?1:0);',
 'const driveAccel=-e.x+angularAccel*shop.movingShopCentreMm[1]/1000+spin*spin*(rx+px);');
replace('const actuatorY=-inertia*alpha-my-shopMomentY;',
 `const actuatorY=g.step(t,dt,target,vr,alpha,shopMomentY,my);`);
replace('v=add(v,force,dt/M);p=add(p,v,dt);L=add(L,moment,dt);',
 'v=add(v,force,dt/M);p=add(p,v,dt);L=add(L,add(moment,cross([0,0,spin],L),-1),dt);');
replace('const newW=add(scale(L,1/J),a,dot(L,a)*(1/I-1/J));a=add','const newW=add(scale(L,1/J),a,dot(L,a)*(1/I-1/J));newW[2]-=spin;a=add');
replace('if(dwell>.3 && norm(v)<.0001 && norm(w)<.002){','if(false){');
replace('thetaRad:theta};','thetaRad:theta,panRateRadS:rate,controller:g.state()};');
process.argv.push('--installed-reactions');
const {testCase}=await import('data:text/javascript;base64,'+Buffer.from(body+"\nconst mode='rim'; export {testCase};").toString('base64'));
const ratio=(68/13)*(76/21)*(84/29);
const nominalAngle=70*Math.PI/180;
const output=root+(inset?'inset-pan-12mm.json':changedDock?'limited-pan-12mm.json':'limited-pan.json');
const runs={};
const clamp=(x,l,h)=>Math.max(l,Math.min(h,x));
for(const [caseId,sched] of Object.entries(schedules.cases)){
  const [scenario,piece]=caseId.split('/');
  for(const amp of (caseId==='adverse/coral-A'?[30,6]:[30])){
    const eta=sched.panEfficiency;let idx=0,next=-Infinity,command=0,integral=0;
    const bins={};
    const g={q:nominalAngle,v:0,peakCurrentA:0,saturatedSeconds:0,peakMotorTerminalV:0,
      state(){return {integral,command,next};},
      event(t){
        while(idx<sched.events.length && t>=sched.events[idx].end)idx++;
        const row=sched.events[idx];
        if(!row||t<row.start)return {x:0,w:0,a:0};
        return row.kind==='turn'?{x:0,w:row.initialSpeed+row.accel*(t-row.start),a:row.accel}:{x:row.accel,w:0,a:0};
      },
      step(t,dt,target,vr,ar,shopTorque,pipeTorque){
        const J=shop.movingStockInertiaKgM2;
        const torquePerAmp=ratio*eta*.97/100, maximum=(amp-1.4)*torquePerAmp;
        if(t>=next-1e-9){
          const err=target-this.q;
          const feedback=65*err+2.5*(vr-this.v)+integral;
          const requested=-J*ar-feedback-shopTorque;
          command=clamp(requested,-maximum,maximum);
          if(Math.abs(requested)<maximum||Math.sign(requested)===Math.sign(err))integral=clamp(integral+15*err*.020,-2,2);
          next=t+.020;
        }
        // 9 V is a conservative independent branch-voltage trial, checked afterwards against
        // the simultaneous battery replay. Retain measured motor speed and motor-side current.
        const ke=(12-.12*1.4)/(11000*Math.PI/30);
        const phaseLimit=clamp((9-ke*Math.abs(this.v)*ratio)/(.12+.016),0,amp);
        const tq=clamp(command,-Math.max(0,phaseLimit-1.4)*torquePerAmp,Math.max(0,phaseLimit-1.4)*torquePerAmp);
        const phase=1.4+Math.abs(tq)/torquePerAmp;const terminal=phase*(.12+.016)+ke*Math.abs(this.v)*ratio;
        const watts=phase*terminal/.9+.7;
        const tick=Math.floor(t*50);
        if(!(tick in bins)||watts>bins[tick][0])bins[tick]=[watts,phase,terminal,t,this.q,this.v,tq];
        this.peakCurrentA=Math.max(this.peakCurrentA,phase);this.peakMotorTerminalV=Math.max(this.peakMotorTerminalV,terminal);
        if(phase>=amp-1e-6)this.saturatedSeconds+=dt;
        this.v+=-(tq+pipeTorque+shopTorque)/J*dt;this.q+=this.v*dt;
        return tq;
      }};
    globalThis.panControl=g;
    const r=testCase(scenario,piece,.0001,scenario==='nominal'?1:-1,1,scenario==='nominal'?-.04:.04,17);
    runs[caseId+'/'+amp]={actuatorModel:{phaseCurrentCapA:amp,efficiency:eta,torquePerAmpNm:ratio*eta*.0097,gearRatio:ratio,feedbackPeriodS:.020,
      assumedGains:[65,2.5,15],controller:'Analytical NEO 550 current limit and 9 V branch lower-bound. Geometric shop-gravity compensation only; no pipe torque measured by feedback.'},
      peakPhaseA:g.peakCurrentA,peakMotorTerminalV:g.peakMotorTerminalV,saturatedSeconds:g.saturatedSeconds,
      readyS:r.ready,carry:r.atCarry,lost:r.lost,maxCarrySupportGapS:r.maxCarryGapS,free:r.free,
      releaseMarginS:r.free?sched.clock.releaseS-r.free.wallS:null,clock:sched.clock,bins};
    writeFileSync(output,JSON.stringify({sourceSha256:sha(source),scheduleSha256:sha(readFileSync(scheduleFile)),
      historicalStockSha256:sha(readFileSync('artifacts/red2-integrated/pan-mechanics.json')),runs,
      scope:'Standardized motor-side phase current, 1.4 A reserve. Four real signed route schedules include yaw and translational inertial forcing at the same time as finite pipe contact and pan acceleration. An actual scoring-panel field capture, installed mechanism inertia/efficiency and motor controller calibration are separate. Commanded motor current and voltage are modeled; no physical hardware was run.'},null,2)+'\n');
    console.log(caseId,amp,r.ready,r.free?.wallS,r.lost,r.maxCarryGapS,g.peakCurrentA,g.peakMotorTerminalV);
  }
}

