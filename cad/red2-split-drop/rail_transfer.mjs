// Integrate the same transverse annulus from the station lip to bilateral release.
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import path from 'node:path';
const root=process.cwd(),out='artifacts/split-drop-native';
const read=f=>JSON.parse(readFileSync(path.join(root,f)));
const sha=f=>createHash('sha256').update(readFileSync(path.join(root,f))).digest('hex');
const register=read('cad/fusion-authority.json').source.archive;
const physical=process.argv.includes('--physical-release'),revision=physical?'r5':'r4';
const railEdge=process.argv.includes('--rail-edge');
const native=read(out+'/'+revision+'-load-01.json');
if(sha(register.file)!==register.sha256 || native.authority.archiveSha256!==register.sha256 ||
   sha(out+'/red2-split-drop-fixture-'+revision+'.f3d')!==native.sha256 ||
   sha(native.step.file)!==native.step.sha256)throw Error('Changed design authority');
const feed=read(out+'/launch-inputs.json');
const block=read('experiments/red2-successor/blockouts.json');
const baseline=Object.values(block).find(v=>v?.['split-drop'])['split-drop'].receiveProfileMm.map(p=>p.map(v=>v/1000));
const M=.816466266,R=.05715,H=.1508125,G=9.80665;
const I=M*(R**2+.0508**2)/2,J=M*(3*(R**2+.0508**2)+4*H**2)/12;
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const plus=(a,b,s=1)=>a.map((v,i)=>v+s*b[i]);
const scale=(a,s)=>a.map(x=>x*s);
const norm=a=>Math.hypot(...a);
const detailed=process.argv[3]==='challenge';
const dtList=detailed?[.0001,.00005]:[.0002,.0001];
const motion=(time,distance,limit,acceleration)=>{
  const ramp=limit/acceleration,cruise=distance/limit-ramp,total=2*ramp+cruise;
  if(time<=0)return [0,0];
  if(time<ramp)return [.5*acceleration*time**2,acceleration*time];
  if(time<ramp+cruise)return [.5*limit*ramp+limit*(time-ramp),limit];
  if(time<total)return [distance-.5*acceleration*(total-time)**2,acceleration*(total-time)];
  return [distance,0];
};
function interval(range,origin,direction,low,high){
  if(Math.abs(direction)<1e-12)return origin>=low && origin<=high;
  const a=(low-origin)/direction,b=(high-origin)/direction;
  range[0]=Math.max(range[0],Math.min(a,b));range[1]=Math.min(range[1],Math.max(a,b));
  return range[0]<=range[1];
}
function execute(caseId,clock,dt,variant,offsetSign=1,yawSign=1){
  const asDrawn=variant==='r5-as-drawn';
  const cheek=['moving-cheeks','steeper-saddle','slow-carry','r5-as-drawn'].includes(variant);
  const lower=['lower-pan','driven-pan'].includes(variant)||cheek;
  const poly=lower?[[-.340,.795],[-.255,.685],[baseline[2][0],variant==='steeper-saddle'||asDrawn?.602:baseline[2][1]],...baseline.slice(3)]:baseline;
  const initial=feed[caseId],adverse=caseId==='adverse';
  const atLip=clock.feed+initial.feedToLipS;
  const theta=adverse?.045*yawSign:0,mu=adverse?.15:.35;
  let p=[initial.x,offsetSign*(adverse?.025:.003),initial.z],v=[initial.vx,0,initial.vz];
  let a=[Math.sin(theta),Math.cos(theta),0],moment=scale(a,I*initial.omega);
  const openAt=clock.release-(.075+(.0408125-.0225)/.6),presentAt=openAt-1.20;
  const history=[];let first=null,settled=null,free=null,dwell=0,peak=0,maxDepth=0;
  let lostSince=null,oneSide=0,oneSideMax=0,minCover=Infinity;
  let energyPeak=0,guideForceN=0,carryHeld=true,lossDuration=0,lossMax=0,lossTotal=0;
  let guideStrike=null,guideMoment=null,guideAboveRibs=null;
  let rimEdgePeakN=0,rimEdgePeakDepthMm=0;
  const priorRelease=[];
  const slow=variant==='slow-carry'&&adverse;
  const carryRamp=slow?.9:.7,carryCoast=slow?.1:.2,carryEnd=2*carryRamp+carryCoast;
  const rigidState=(w)=>({p:[...p],v:[...v],axis:[...a],omega:[...w],angularMomentum:[...moment]});
  // The route's long stopped intervals need no high-frequency integration
  // after the same two contacts have reached a stationary equilibrium.
  for(let t=atLip;t<=clock.release+.42;t+=dt){
    const local=t-atLip,slide=motion(t-presentAt,.404,.55,2),jaw=motion(t-openAt,.090,.60,8);
    const w=plus(scale(moment,1/J),a,dot(moment,a)*(1/I-1/J));
    const sinceCarry=t-clock.carry;
    const accel=(slow?1.05:adverse?1.5:.9)*(sinceCarry>0 && sinceCarry<carryRamp?-1:sinceCarry>carryRamp+carryCoast && sinceCarry<carryEnd?1:0);
    let force=[M*accel,0,-M*G],torque=[0,0,0],loads=[0,0],allLoads=[0,0],contactDepth=0,cheekLoads=[0,0],occupied=[false,false];
    function apply(arm,n,depth,weight,sign,moving,belt=null){
      if(depth<=0)return;
      occupied[sign>0?1:0]=true;
      const own=plus(v,cross(w,arm));own[0]-=moving?slide[1]:0;own[1]-=moving?sign*jaw[1]:0;
      if(belt && variant==='driven-pan'&&!moving)for(let k=0;k<3;k++)own[k]-=.48*belt[k];
      const vn=dot(own,n),fn=weight*Math.max(0,60000*depth-1.2*Math.sqrt(60000*M)*vn);
      if(fn===0)return;
      const slip=plus(own,n,-vn),sl=norm(slip);
      const f=plus(scale(n,fn),slip,-mu*fn*Math.tanh(sl/.005)/Math.max(1e-15,sl));
      force=plus(force,f);torque=plus(torque,cross(arm,f));loads[sign>0?1:0]+=Math.max(0,n[2])*fn;
      allLoads[sign>0?1:0]+=fn;
      if(moving && n[2]===0){
        cheekLoads[sign>0?1:0]+=fn;
        if(!guideStrike || fn>guideStrike.normalN){
          const tip=plus(p,arm);
          guideStrike={normalN:fn,wallS:t,side:sign,localXmm:(tip[0]-slide[0])*1000,
            localZmm:tip[2]*1000,depthMm:depth*1000};
        }
        if(process.argv.includes('--strike') && asDrawn){
          const tip=plus(p,arm),x=(tip[0]-slide[0])*1000,z=tip[2]*1000;
          const ribs=[-61,4,70,136,202,253,286,309];
          const j=ribs.findIndex(value=>value>=x);
          const end=j<=0;
          const a=j===-1?309:j===0?-61:ribs[j-1],b=end?a:ribs[j];
          const between=end?0:(x-a)*(b-x)/(b-a);
          const moment=fn*(end?Math.abs(x-a):between);
          const ribTop=q=>{
            const i=poly.findIndex((p,k)=>k>=2 && k+1<poly.length && p[0]*1000<=q && q<=poly[k+1][0]*1000);
            const u=(q-1000*poly[i][0])/(1000*(poly[i+1][0]-poly[i][0]));
            return 147+1000*(poly[i][1]+u*(poly[i+1][1]-poly[i][1]));
          };
          const record={wallS:t,normalN:fn,side:sign,localXmm:x,localZmm:z,ribsMm:[a,b],momentNmm:moment};
          if(z>Math.min(ribTop(a),ribTop(b))){
            const allowed=ribs.filter(q=>ribTop(q)>=z && poly.slice(2).every(([u,h])=>
              u*1000<=Math.min(q,x) || u*1000>=Math.max(q,x) || h*1000+150>=z));
            const closest=allowed.sort((a,b)=>Math.abs(a-x)-Math.abs(b-x))[0];
            const cantilever=closest===undefined?null:fn*Math.abs(closest-x);
            if(!guideAboveRibs || cantilever===null || (guideAboveRibs.cantileverMomentNmm!==null && cantilever>guideAboveRibs.cantileverMomentNmm)){
              guideAboveRibs={...record,singleReachableRibMm:closest,cantileverMomentNmm:cantilever};
            }
          } else if(!guideMoment || moment>guideMoment.momentNmm)guideMoment=record;
        }
      }
      contactDepth=Math.max(contactDepth,depth);
    }
    function beds(moving){
      if(moving||variant==='narrow')return [-1,1].map(sign=>({sign,y:sign*(.116+(moving?jaw[0]:0)),width:.012,weight:.5}));
      return Array.from({length:12},(_,j)=>({sign:j<6?-1:1,y:-.165+j*.030,width:.030,weight:1/12}));
    }
    let rims=null;
    if(railEdge && physical && t>=openAt){
      // The pipe-end disk can graze the inner edge of a real 12 mm rail even
      // after the face-normal support generator has passed its finite width.
      // Intersect each actual inclined edge segment with the complete finite
      // annular wall. Keep one deepest supplemental witness on each jaw.
      rims=[null,null];
      for(let i=2;i+1<poly.length;i++){
        const dx=poly[i+1][0]-poly[i][0],dz=poly[i+1][1]-poly[i][1],edge=[dx,0,dz];
        const axialRate=dot(edge,a),radialRate=plus(edge,a,-axialRate),A=dot(radialRate,radialRate);
        for(const sign of [-1,1])for(const side of [.110,.122]){
          const from=plus([poly[i][0]+slide[0],sign*(side+jaw[0]),poly[i][1]],p,-1);
          const axial0=dot(from,a),radial0=plus(from,a,-axial0),range=[0,1];
          if(!interval(range,axial0,axialRate,-H,H))continue;
          const B=dot(radial0,radialRate),C=dot(radial0,radial0)-R*R,disc=B*B-A*C;
          if(A<1e-16 || disc<=0)continue;
          const low=Math.max(range[0],(-B-Math.sqrt(disc))/A),high=Math.min(range[1],(-B+Math.sqrt(disc))/A);
          if(low>=high)continue;
          for(const q of [.01,.25,.5,.75,.99]){
            const u=low+(high-low)*q,arm=plus(from,edge,u),h=axial0+u*axialRate;
            const radial=plus(arm,a,-h),rho=norm(radial);
            const outer=R-rho,inner=rho-.0508,cap=H-Math.abs(h),depth=Math.min(outer,inner,cap);
            if(depth<=0)continue;
            const id=sign>0?1:0;
            occupied[id]=true;
            if(!rims[id] || depth>rims[id].depth){
              const n=depth===cap?scale(a,-Math.sign(h)):scale(radial,(depth===outer?-1:1)/rho);
              rims[id]={arm,n,depth,sign};
            }
          }
        }
      }
    }
    for(let i=0;i<poly.length-1;i++){
      const moving=i>=2,shift=moving?slide[0]:0;
      const x=poly[i][0]+shift,z=poly[i][1],dx=poly[i+1][0]-poly[i][0],dz=poly[i+1][1]-z;
      const len=Math.hypot(dx,dz),tang=[dx/len,0,dz/len],n=[-dz/len,0,dx/len];
      const along=dot(n,a),radial=Math.sqrt(Math.max(1e-15,1-along*along));
      const offset=scale(plus(n,a,-along),-R/radial);
      const spot=plus(p,offset),relative=[spot[0]-x,spot[1],spot[2]-z];
      const signed=dot(relative,n),slope=dot(tang,a);
      for(const {sign,y,width,weight} of beds(moving)){
        const range=[-H,H];
        if(!interval(range,spot[1],a[1],y-width/2,y+width/2)
          ||!interval(range,dot(relative,tang),slope,0,len)
          ||!interval(range,signed,along,-.02,.00005))continue;
        const fraction=Math.max(0,Math.min(1,(range[1]-range[0])*Math.abs(a[1])/width));
        if(fraction===0)continue;
        for(const u of [.211325,.788675]){
          const h=range[0]+(range[1]-range[0])*u;
          const arm=plus(offset,a,h),depth=-(signed+h*along);
          apply(arm,n,depth,fraction*.5*weight,sign,moving,tang);
        }
      }
    }
    // Check the finite horizontal Y edges at the ends of the inclined faces.
    // The Voronoi tests exclude regions already supported by either face.
    for(let i=0;i<poly.length;i++){
      const moving=i>=2,x=poly[i][0]+(moving?slide[0]:0),z=poly[i][1];
      for(const {sign,y,width,weight} of beds(moving)){
        for(const u of [-width/Math.sqrt(12),width/Math.sqrt(12)]){
          const edge=[x,y+u,z],to=plus(edge,p,-1),s=dot(to,a);
          if(Math.abs(s)>H)continue;
          const radial=plus(to,a,-s),rho=norm(radial);
          if(rho>=R||rho<.0508)continue;
          let eligible=true;
          if(i){
            const d=poly[i-1];
            if(-radial[0]*(poly[i][0]-d[0])-radial[2]*(poly[i][1]-d[1])<0)eligible=false;
          }
          if(i<poly.length-1){
            const d=poly[i+1];
            if(-radial[0]*(d[0]-poly[i][0])-radial[2]*(d[1]-poly[i][1])>0)eligible=false;
          }
          if(eligible)apply(to,scale(radial,-1/rho),R-rho,.5*weight,sign,moving);
        }
      }
    }
    if(variant==='wide-pan-guides'||lower){
      const x0=-.36,x1=-.035,rate=(.158-.192)/(x1-x0),nd=Math.hypot(1,rate);
      for(const sign of [-1,1]){
        const n=[rate/nd,-sign/nd,0],s=dot(n,a),rad=Math.sqrt(Math.max(1e-14,1-s*s));
        const arm=plus(scale(a,-Math.sign(s)*H),plus(n,a,-s),-R/rad);
        const tip=plus(p,arm),depth=-(.192+rate*(tip[0]-x0)-sign*tip[1])/nd;
        if(tip[0]<(asDrawn?-.340:x0) || tip[0]>(asDrawn?-.068:x1) || depth<=0)continue;
        const i=tip[0]<-.255?0:1;
        const frac=(tip[0]-poly[i][0])/(poly[i+1][0]-poly[i][0]);
        const bed=poly[i][1]+frac*(poly[i+1][1]-poly[i][1]);
        if(tip[2]>=bed-.006 && tip[2]<=bed+.120)apply(arm,n,depth,.7,sign,false);
      }
    }
    if(cheek){
      // Each vertical inner face is fixed to its own translating jaw. It
      // arrests lateral roll-off in the cradle and opens with the support.
      for(const sign of [-1,1]){
        const n=[0,-sign,0],s=dot(n,a),rad=Math.sqrt(Math.max(1e-14,1-s*s));
        const arm=plus(scale(a,-Math.sign(s)*H),plus(n,a,-s),-R/rad);
        const tip=plus(p,arm),x=tip[0]-slide[0];
        const depth=sign*tip[1]-(.160+jaw[0]);
        const i=poly.findIndex((q,j)=>j>=2 && j+1<poly.length && q[0]<=x && x<=poly[j+1][0]);
        if(i<0||depth<=0)continue;
        const frac=(x-poly[i][0])/(poly[i+1][0]-poly[i][0]);
        const bed=poly[i][1]+frac*(poly[i+1][1]-poly[i][1]);
        const bore=asDrawn&&[.555,.570].some(z=>Math.hypot(x+.044,tip[2]-z)<.006);
        if(!bore && tip[2]>=(asDrawn?.542:bed-.006) && tip[2]<=bed+.150)apply(arm,n,depth,.7,sign,true);
      }
    }
    if(rims){
      for(const [k,witness] of rims.entries()){
        if(!witness || allLoads[k]>.005)continue;
        const before=allLoads[k],{arm,n,depth,sign}=witness;
        apply(arm,n,depth,.5,sign,true);
        rimEdgePeakN=Math.max(rimEdgePeakN,allLoads[k]-before);
        rimEdgePeakDepthMm=Math.max(rimEdgePeakDepthMm,1000*depth);
      }
    }
    if(first===null && loads.some(v=>v>.02))first=t;
    if(t<openAt && loads.every(v=>v>.03) && loads[0]+loads[1]>=.75*M*G && norm(v)<.015 && norm(w)<.3)dwell+=dt;else dwell=0;
    if(settled===null&&dwell>.12)settled=t;
    if(sinceCarry>=0 && sinceCarry<=carryEnd && (!loads.every(v=>v>.03) || loads[0]+loads[1]<.5*M*G)){
      carryHeld=false;lossDuration+=dt;lossTotal+=dt;lossMax=Math.max(lossMax,lossDuration);
    }else lossDuration=0;
    if(t>=openAt){
      const alive=allLoads.map((q,k)=>q>.005 || (physical && occupied[k]));
      if(railEdge){
        priorRelease.push({wallS:t,state:rigidState(w),jawMm:jaw[0]*1000,sideLoadsN:[...allLoads],occupied:[...occupied]});
        if(priorRelease.length>18)priorRelease.shift();
      }
      oneSide=alive[0]!==alive[1]?oneSide+dt:0;oneSideMax=Math.max(oneSideMax,oneSide);
      if(!alive.some(Boolean)){
        if(lostSince===null)lostSince={wallS:t,state:rigidState(w),jawMm:jaw[0]*1000,...(railEdge?{precedingFrames:[...priorRelease]}:{})};
        if(t-lostSince.wallS>.008 && free===null)free=lostSince;
      }else{lostSince=null;free=null;}
    }
    peak=Math.max(peak,...loads);guideForceN=Math.max(guideForceN,...cheekLoads);maxDepth=Math.max(maxDepth,contactDepth);
    if(t<presentAt)minCover=Math.min(minCover,.210-(Math.abs(p[1])+H*Math.abs(a[1])+R*Math.sqrt(Math.max(0,1-a[1]**2))));
    if(history.length===0 || t-history.at(-1).wallS>=(t>=openAt?.002:.05)){
      history.push({wallS:t,...rigidState(w),jawMm:jaw[0]*1000,slideMm:slide[0]*1000,normalN:loads});
    }
    if(free && t-free.wallS>.020)break;
    if(p[2]<.15 || Math.abs(p[0])>1.2)break;
    if(sinceCarry>2.0 && t<presentAt-1 && norm(v)<.0001 && norm(w)<.001 && loads.every(v=>v>.1)){
      t=presentAt-.1;
    }
    v=plus(v,force,dt/M);p=plus(p,v,dt);moment=plus(moment,torque,dt);
    a=plus(a,cross(w,a),dt);a=scale(a,1/norm(a));
    energyPeak=Math.max(energyPeak,.5*M*dot(v,v)+.5*dot(w,moment)+M*G*p[2]);
  }
  return {clock,dt,atLip,firstContactS:first,stationReadyS:settled,readyVsCarryS:settled===null?null:clock.carry-settled,
    openAt,free,unilateralSupportS:oneSideMax,peakPerRailN:peak,maxContactDepthMm:1000*maxDepth,
    admissionWidthMarginMm:minCover*1000,guideForceN,...(railEdge?{rimEdgePeakN,rimEdgePeakDepthMm}:{}),...(process.argv.includes('--strike')?{guideStrike,guideMoment,guideAboveRibs}:{}),carryHeld:!!free&&carryHeld,
    longestSupportInterruptionMs:1000*lossMax,combinedSupportInterruptionMs:1000*lossTotal,carryPulseS:carryEnd,energyPeakJ:energyPeak,history};
}
const runs={},variant=process.argv[2]||'narrow';
if(!['narrow','wide-pan','wide-pan-guides','lower-pan','driven-pan','moving-cheeks','steeper-saddle','slow-carry','r5-as-drawn'].includes(variant))throw Error('Unknown entrance');
for(const c of ['nominal','adverse'])for(const [id,clock] of Object.entries(feed.clocks[c]))for(const dt of dtList){
  for(const [os,ys] of detailed&&c==='adverse'?[[1,1],[-1,1],[1,-1],[-1,-1]]:[[1,1]]){
    const k=c+'-'+id+'-'+dt+(detailed?'-'+os+'-'+ys:'');runs[k]=execute(c,clock,dt,variant,os,ys);
  }
}
writeFileSync(path.join(root,out,revision+'-spatial-'+variant+(detailed?'-challenge':'')+(process.argv.includes('--strike')?'-strike':'')+(physical?'-physical':'')+(railEdge?'-edge':'')+'.json'),JSON.stringify({source:register.sha256,fixture:native.sha256,
  basis:'Finite transverse pipe with rigid-body inertia and independently moving finite top faces. The two rear ramp sections are provisional shop geometry. No field score is emitted by this solver.',
  runs}));
for(const [k,r] of Object.entries(runs))console.log(k,JSON.stringify({ready:r.stationReadyS,release:r.free?.wallS,cargo:r.free?.state.p,asymmetric:r.unilateralSupportS,depth:r.maxContactDepthMm}));
