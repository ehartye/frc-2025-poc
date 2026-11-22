import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const output=resolve(process.argv[2]);
const hash=buffer=>createHash('sha256').update(buffer).digest('hex');
const code=readFileSync('cad/red2-split-drop/single_pivot_spatial.mjs');
if(hash(code)!=='7a921f81ed82efc811b15e8b53a4cb752581594c8cc8f7d38baf1e91187f1fd2') throw Error('Different historical solver');
const oldPath='artifacts/red2-integrated/single-pivot-spatial-rim-installed-reactions-duty.json';
const raw=readFileSync(oldPath);
const historical=JSON.parse(raw);
const retain={};
function checkpoint() {
  writeFileSync(output,JSON.stringify({archivedSolver:hash(code),archivedDuty:hash(raw),description:'Real signed torque and angular rate grouped into 20 ms wall-clock cells. Extreme row retains signed torque, time, original programmed angle rate, and the observed phase. Explicitly includes the verified static holds.',runs:retain},null,2)+'\n');
}
globalThis.observePan=(t,end,torque,velocity,phase) => {
  const start=Math.floor(t*50),last=Math.floor(end*50);
  for(let key=start;key<=last;key++){
    const row=globalThis.observed.get(key);
    if(!row || Math.abs(torque)>Math.abs(row[0])) globalThis.observed.set(key,[torque,velocity,phase]);
  }
};
let text=code.toString().split('const mode=process.argv[2]')[0]
  .replace('previousActuatorY=actuatorY;account(dt,actuatorY);',
    "globalThis.observePan(t,t+dt,actuatorY,rate,t<clock.carry?'feed':t<start?'carry':'tip'); previousActuatorY=actuatorY;account(dt,actuatorY);")
  .replace('if(windows)account(next-t,previousActuatorY);',
    "if(windows){globalThis.observePan(t,next,previousActuatorY,0,t<clock.carry?'feed':'carry');account(next-t,previousActuatorY);}");
if(!text.includes('globalThis.observePan(t,next')||!text.includes('globalThis.observePan(t,t+dt'))throw Error('Phase observation sites missing');
text+='\nconst mode="rim"; export { testCase };';
process.argv.push('--installed-reactions','--drive-windows');
const {testCase}=await import('data:text/javascript;base64,'+Buffer.from(text).toString('base64'));
const chosen=['nominal/coral-A/1/1/-0.04/0.0001/17','adverse/coral-B/-1/1/0.04/0.0001/17','nominal/coral-B/1/1/-0.04/0.0001/17','adverse/coral-A/-1/1/0.04/0.0001/17'];
for(const id of chosen){
  const [scenario,piece,dy,ys,pitch,dt,n]=id.split('/');
  globalThis.observed=new Map();
  const actual=testCase(scenario,piece,+dt,+dy,+ys,+pitch,+n),ref=historical.runs[id];
  for(const key of ['loadPeaks','ready','free','duty','stepCount']) if(JSON.stringify(actual[key])!==JSON.stringify(ref[key]))throw Error('Archived control changed '+key+' '+id);
  retain[id]={stepCount:ref.stepCount,releaseS:ref.free.wallS||ref.free.t,bins:Object.fromEntries(globalThis.observed)};
  checkpoint();console.log(id,ref.stepCount,globalThis.observed.size);
}
