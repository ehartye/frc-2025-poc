import {createHash} from 'node:crypto';
import {readFileSync,writeFileSync} from 'node:fs';
const sha=b=>createHash('sha256').update(b).digest('hex');
const get=p=>JSON.parse(readFileSync(p));
const file='cad/red2-split-drop/single_pivot_spatial.mjs';
const source=readFileSync(file);
if(sha(source)!=='7a921f81ed82efc811b15e8b53a4cb752581594c8cc8f7d38baf1e91187f1fd2')throw Error('Wrong archived load excitation');
const dutyFile='artifacts/red2-integrated/single-pivot-spatial-rim-installed-reactions-duty.json';
const duty=get(dutyFile);
const mechanics='artifacts/red2-integrated/pan-mechanics.json';
const J=get(mechanics).movingStockInertiaKgM2;
const projectTrace=get('iterations/002/design/strategy/pan-on-match-clock.json');
const output=process.argv[2];
let history={};
let observations=[];
let wallOrigin=null;
const ratio=(68/13)*(76/21)*(84/29);
globalThis.driveStop=(t,duration,required,feed)=>{
 if(!feed)return;
 if(wallOrigin===null)wallOrigin=t;
 for(const o of observations){
   // Only replay actual sampled loads during acquisition. Stable long
   // waits advance at the same fixed step and preserve the unloaded return.
   const N=Math.max(1,Math.ceil(duration/.0001)),h=duration/N;
   for(let j=0;j<N;j++){
     const command=Math.max(-o.maxTorque,Math.min(o.maxTorque,required-65*o.theta-2.5*o.omega));
     const compression=Math.max(0,o.theta*.100);
     const reaction=Math.max(0,o.stiffness*1e3*compression+o.damping*(compression>0?.100*o.omega:0));
     const acceleration=(command-required-.100*reaction)/J;
     o.omega+=h*acceleration;o.theta+=h*o.omega;
     if(compression>o.peakCompressionM){o.peakCompressionM=compression;o.atS=t+j*h;}
     o.peakTipDisplacementMm=Math.max(o.peakTipDisplacementMm,340*Math.abs(o.theta));
     o.peakStopN=Math.max(o.peakStopN,reaction);
     o.activeSeconds+=o.theta>0&&o.stiffness>0?h:0;
   }
 }
};
let code=source.toString().split('const mode=process.argv[2]')[0]
 .replace('previousActuatorY=actuatorY;account(dt,actuatorY);','globalThis.driveStop(t,dt,actuatorY,t<clock.carry);previousActuatorY=actuatorY;account(dt,actuatorY);')
 .replace('if(windows)account(next-t,previousActuatorY);','if(windows){globalThis.driveStop(t,next-t,previousActuatorY,t<clock.carry);account(next-t,previousActuatorY);}');
code+="\nconst mode='rim';export {testCase};";
process.argv.push('--installed-reactions','--drive-windows');
const {testCase}=await import('data:text/javascript;base64,'+Buffer.from(code).toString('base64'));
for(const id of Object.keys(projectTrace.runs)){
  const [scenario,piece,dy,ys,pitch,dt,n]=id.split('/');
  wallOrigin=null;
  observations=[];
  for(const eta of [.45,.60]){
    for(const [stiffness,damping] of [[0,0],[300,0],[300,80],[900,80]]){
      observations.push({stiffness,damping,eta,maxTorque:28.6*.97/100*ratio*eta,theta:0,omega:0,peakCompressionM:0,peakStopN:0,peakTipDisplacementMm:0,activeSeconds:0});
    }
  }
  const r=testCase(scenario,piece,+dt,+dy,+ys,+pitch,+n);
  if(JSON.stringify(r.loadPeaks)!==JSON.stringify(duty.runs[id].loadPeaks))throw Error('Input force history changed');
  history[id]=observations;
  const result={sourceSolver:sha(source),archivedDuty:sha(readFileSync(dutyFile)),movingStockInertiaKgM2:J,
    maxControllerPhaseA:30,rotorFrictionReserveA:1.4,tangentLeverM:.100,servoKpNmRad:65,servoKdNmsRad:2.5,
    numerator:'actual original signed acquisition torque at its original high-rate source clock',history,
    scope:'One-way transient. Each replay has the exact same original imposed pipe load. Simulated stow deviation feeds the illustrative PD/stop dynamics but does not feed back into pipe contact, rolling, guide deformation, or a measured bumper force curve. Positive stow angle compresses the single stop. Reverse deflection stays unsupported. Spring N/mm and viscous N s/m are design sensitivities. Original 30 A total limit throughout. A fixed shift of this same feed waveform from rest preserves these excursions and loads.'};
  writeFileSync(output,JSON.stringify(result,null,2)+'\n'); console.log(id,observations.map(x=>Math.round(x.peakTipDisplacementMm*1000)/1000));
}
