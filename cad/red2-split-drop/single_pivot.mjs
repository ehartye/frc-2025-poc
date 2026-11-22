import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';

const read = f=>JSON.parse(readFileSync(f));
const hash = f=>createHash('sha256').update(readFileSync(f)).digest('hex');
const source=read('cad/fusion-authority.json').source.archive;
if(hash(source.file)!==source.sha256)throw Error('Changed native master');
const launch='artifacts/split-drop-native/launch-inputs.json';
const data=read(launch);
const M=.816466266, R=.05715, I=M*(R*R+.0508*.0508)/2, G=9.80665;
const K=60000,D=1.2*Math.sqrt(K*M);
const deg=Math.PI/180;
const label=process.argv[2]||'baseline';
const tuning={baseline:[-12,2.17477264],shallow:[-4,3.2],medium:[-6,2.8],slow:[-8,2.5],levelled:[-2,3.57337264],short:[-2,3.50],shallowShort:[-1.5,4.00]}[label];
if(!tuning)throw Error('Unregistered angular duty');
const base={axis:[.205,.580],fixed:[[-.340,.795],[-.255,.685],[.075,.592]],
            pan:[[-.100,.110],[-.09,0],[['short','shallowShort'].includes(label)?.340:.370,0]],stowDeg:70,dumpDeg:tuning[0],rateDurationS:1.60};
// Both sides use the same profile in this sagittal concept screen. Fixed rear feeds
// and pivoting rails are different actual part families, with the y-offset/hardware
// and off-axis custody left for the independent spatial test.
function pose(t,start,reset,model){
  function smooth(u){
    if(u<=0)return [0,0];
    if(u>=1)return [1,0];
    return [u*u*(3-2*u),6*u*(1-u)/model.rateDurationS];
  }
  let [q,qd]=smooth((t-start)/model.rateDurationS);
  if(reset!==null && t>reset){
    const [r,rd]=smooth((t-reset)/model.rateDurationS);
    q=1-r;qd=-rd;
  }
  const travel=(model.dumpDeg-model.stowDeg)*deg;
  return [model.stowDeg*deg+travel*q,travel*qd];
}
function run(id,clock,source,dt,model,offset,finalPitch){
  const lip=clock.feed+source.feedToLipS,motionStart=clock.release-offset;
  let x=source.x,z=source.z,vx=source.vx,vz=source.vz,omega=source.omega;
  let first=null,ready=null,release=null,lastPan=null,lastForce=null,peak=0,freeStart=null,previous=null;
  let dwell=0,carryHeld=true,stuck=false,firstTiltState=null,maxRootTorque=0,settledAtCarry=false;
  let final=null;
  const code=id==='adverse'?'adverse':'nominal';
  const mu=id==='adverse'?.15:.35,accel=id==='adverse'?1.5:.9;
  const end=clock.release+1;
  for(let k=0,t=lip;t<=end;k++,t=lip+k*dt){
    const [theta,rate]=pose(t,motionStart,null,model);
    const co=Math.cos(theta),si=Math.sin(theta),[px,pz]=model.axis;
    const ray=([u,h])=>[px+co*u-si*h,pz+si*u+co*h];
    const projected=model.pan.map(ray);
    const since=t-clock.carry;
    const actualPitch=finalPitch*Math.max(0,Math.min(1,(t-motionStart+2)));
    let fx=-M*G*Math.sin(actualPitch)+M*accel*(since>0 && since<.7?-1:since>.9 && since<1.6?1:0);
    let fz=-M*G*Math.cos(actualPitch),trq=0,normal=0,panForce=0,panOverlap=false,maxDepth=0,pivotTrq=0;
    for(const [isPan,profile] of [[false,model.fixed],[true,projected]]){
      for(let i=1;i<profile.length;i++){
        const [x0,z0]=profile[i-1],[x1,z1]=profile[i],dx=x1-x0,dz=z1-z0;
        const u=Math.max(0,Math.min(1,((x-x0)*dx+(z-z0)*dz)/(dx*dx+dz*dz)));
        const cx=x0+u*dx,cz=z0+u*dz,dist=Math.hypot(x-cx,z-cz);
        if(dist>=R)continue;
        if(dist<1e-9)throw Error('Pipe center penetrated a supporting rail');
        if(isPan)panOverlap=true;
        const nX=(x-cx)/dist,nZ=(z-cz)/dist,tX=nZ,tZ=-nX;
        const rvx=vx+(isPan?rate*(cz-pz):0),rvz=vz-(isPan?rate*(cx-px):0);
        const fn=Math.max(0,K*(R-dist)-D*(rvx*nX+rvz*nZ));
        const ft=-mu*fn*Math.tanh((rvx*tX+rvz*tZ-omega*R)/.005);
        const hx=fn*nX+ft*tX,hz=fn*nZ+ft*tZ;
        normal+=fn;
        if(isPan){
          panForce+=fn;
          pivotTrq+=(cx-px)*hz-(cz-pz)*hx;
          if(fn>0)lastForce=t;
        }
        fx+=hx;fz+=hz;trq-=ft*R;
        peak=Math.max(peak,fn);maxDepth=Math.max(maxDepth,R-dist);
      }
    }
    if(first===null && normal>0)first=t;
    if(panOverlap) lastPan=t;
    if(panForce>0 && Math.hypot(vx,vz)<.035 && Math.abs(omega)<.7)dwell+=dt;else dwell=0;
    if(dwell>=.12 && ready===null)ready=t;
    if(t>=clock.carry && t<clock.carry+dt)settledAtCarry=ready!==null && dwell>=.12;
    if(since>0 && since<1.6 && (!panOverlap || Math.hypot(x-px,z-pz)>.30))carryHeld=false;
    if(t>motionStart && firstTiltState===null)firstTiltState={wallS:t,x,z,vx,vz,omega,dwellS:dwell};
    if(t>=motionStart)maxRootTorque=Math.max(maxRootTorque,Math.abs(pivotTrq));
    const localX=(x-px)*co+(z-pz)*si;
    if(t>motionStart && localX>model.pan.at(-1)[0] && !panOverlap && lastPan!==null){
      if(freeStart===null){freeStart=t;previous={wallS:t,x,z,vx,vz,omega,actualTrayDeg:theta/deg};}
      if(t-freeStart>=.002){release=previous;break;}
    }else freeStart=null;
    final={x,z,vx,vz,omega,wallS:t,localX};
    if(z<.12 || x>1.25 || x < -.9){stuck=true;break;}
    vx+=fx/M*dt;vz+=fz/M*dt;omega+=trq/I*dt;x+=vx*dt;z+=vz*dt;
  }
  return {case:code,dt,model,finalPitch,firstContactS:first,readyS:ready,carryDeadline:clock.carry,
          settledAtCarry,heldDuringCarry:carryHeld,firstTiltState,release,targetReleaseS:clock.release,
          releasedVsClockS:release?release.wallS-clock.release:null,lastActualPanOverlapS:lastPan,lastForceS:lastForce,
          peakContactN:peak,maxPipeMomentAboutShaftNm:maxRootTorque,escapeBeforeRelease:stuck,final};
}
const results={};
for(const [name,clocks] of Object.entries(data.clocks)){
  for(const [id,c] of Object.entries(clocks)){
    for(const step of [.0002,.0001])for(const pitch of [-.04,0,.04]){
      results[name+'/'+id+'/'+step+'/'+pitch]=run(name,c,data[name],step,['levelled','short','shallowShort'].includes(label)?{...base,dumpDeg:base.dumpDeg-pitch/deg}:base,tuning[1],pitch);
    }
  }
}
const output={authority:source,initialConditionsSha256:hash(launch),basis:'Sagittal comparison on the exact original station lip, Red 2 feed and carry clock. Posed physical rails integrate contact and a freely rolling pipe. Latency, high-angle guide/swept fixture, drive and three-dimensional transfer are unresolved.',results};
writeFileSync('artifacts/red2-integrated/single-pivot-'+label+'-acquisition.json',JSON.stringify(output,null,2)+'\n');
for(const [k,r] of Object.entries(results))console.log(k,r.readyS,r.heldDuringCarry,r.releasedVsClockS,r.release?.x,r.release?.z,r.maxPipeMomentAboutShaftNm);
