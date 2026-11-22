import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';

const read=f=>JSON.parse(readFileSync(f)),hash=f=>createHash('sha256').update(readFileSync(f)).digest('hex');
const dir='artifacts/red2-integrated/',master=read('cad/fusion-authority.json').source.archive;
const sourceFile='artifacts/split-drop-native/launch-inputs.json',source=read(sourceFile);
const foundation=read('cad/red2-split-drop/chassis-foundation-r2.json');
if(hash(master.file)!==master.sha256||foundation.authority.archiveSha256!==master.sha256||
   hash(dir+'red2-integrated-foundation-r2.f3d')!==foundation.nativeArchiveSha256)throw Error('Changed native sources');
const H=.1508125,R=.05715,M=.816466266,I=M*(R*R+.0508**2)/2,J=M*(3*(R*R+.0508**2)+4*H*H)/12,G=9.80665;
const deg=Math.PI/180;
const scale=(p,s)=>p.map(v=>v*s),add=(p,v,k=1)=>p.map((q,i)=>q+k*v[i]);
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2],norm=p=>Math.hypot(...p);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const settings={pivot:[.205,.58],pan:[[ -.100,.110],[-.090,0],[.340,0]],stowDeg:70,worldDumpDeg:-2,
  tipStartBeforeReleaseS:3.5,tipDurationS:1.6,feed:[[-.340,.795],[-.255,.685],[-.035,.6229999999999999],[.075,.592]],
  feedHalfWidthsM:[.192,.192,.082,.082],floorHalfWidthM:.161,wallSlotHalfWidthM:.096,wallLimitM:.161,wallHeightM:.120};
const guarded=!process.argv.includes('--without-entry-guides');
const reactions=process.argv.includes('--installed-reactions');
const windows=process.argv.includes('--drive-windows');
const frameAnchors=process.argv.includes('--frame-anchors');
if(frameAnchors && !reactions)throw Error('Frame reaction capture requires installed reaction geometry');
if(windows && !reactions)throw Error('Drive windows require measured contact reactions');
const shop=reactions?read(dir+'pan-mechanics.json'):null;
if(reactions && (shop.foundation!==foundation.nativeArchiveSha256 || shop.shopPlan!==hash(dir+'pan-shop-plan.json')))throw Error('Installed shop stock changed');
function testCase(scenario,id,dt,dy,ys,pitch,stations){
  const original=source[scenario],clock=source.clocks[scenario][id],adverse=scenario==='adverse';
  const lip=clock.feed+original.feedToLipS,start=clock.release-settings.tipStartBeforeReleaseS;
  const mu=adverse?.15:.35,angle0=(adverse?.045*ys:0);
  let p=[original.x,dy*(adverse?.025:.003),original.z],v=[original.vx,0,original.vz],a=[Math.sin(angle0),Math.cos(angle0),0],L=scale(a,I*original.omega);
  let ready=null,dwell=0,first=null,lost=false,everTouch=false,atCarry=null,carryInterruption=0,maxCarryGap=0,free=null,lastOccupied=null;
  let openState=null,peak=0,maxRim=0,maxSide=0,maxShaft=0,stable=false,unilateral=0;
  let guideImpact=null,panImpact=null,firstAir=null,final=null,stepCount=0;
  const duration=settings.tipDurationS;
  const loadPeaks={},duty={},ratio=(68/13)*(76/21)*(84/29);
  if(windows)for(const eta of [.45,.60,.75])for(const amps of [20,30,40,60]){
    const name=eta+'/'+amps;
    duty[name]={efficiency:eta,torqueCurrentA:amps,shaftLimitNm:.97/100*amps*ratio*eta,
                allRecordedS:0,aboveLimitS:0,longestOverLimitS:0,aboveBlockS:0,
                excessAngularImpulseNms:0,integratedTorqueSquaredNm2s:0,peakAbove:0};
  }
  let previousActuatorY=0;
  const anchorRows={};
  let anchorSamples=0;
  function retainAnchor(rows,key,value,build){
    if(!rows[key] || value>rows[key].value)rows[key]={value,...build()};
  }
  function recordAnchors(sample,tower,side,fx,fy,fz,stationType){
    const xOffset=[-.019,.019],yOffset=[-.0185*side,.0185*side];
    const dy=side*(.195775-.2235),dz=.100;
    const mx=dy*fz-dz*fy, my=dz*fx, mz=-dy*fx;
    const sumX2=4*xOffset[0]**2,sumY2=4*yOffset[0]**2,polar=sumX2+sumY2;
    const name=stationType+'/'+tower;
    const result=anchorRows[name] ||= {};
    const foot=()=>({...sample(),forceToTowerN:[fx,fy,fz],momentAtFourBoltCenterNm:[mx,my,mz]});
    retainAnchor(result,'netFootUpliftN',Math.max(fz,0),foot);
    retainAnchor(result,'netFootCompressionN',Math.max(-fz,0),foot);
    retainAnchor(result,'radialBearingN',Math.hypot(fx,fz),foot);
    retainAnchor(result,'axialBearingN',Math.abs(fy),foot);
    for(let i=0;i<2;i++)for(let j=0;j<2;j++){
      const x=xOffset[i],y=yOffset[j];
      const xForce=fx/4-mz*y/polar,yForce=fy/4+mz*x/polar;
      const zForce=fz/4+mx*y/sumY2-my*x/sumX2;
      const shear=Math.hypot(xForce,yForce);
      const clamp=Math.max(zForce,0);
      const prefix=(i?'x224/':'x186/')+(j?'sill/':'cross/');
      const detail=()=>({...foot(),boltPointMm:[i?224:186,side*(j?242:205),480],forceN:[xForce,yForce,zForce],
        shearN:shear,tensileN:clamp,compressionN:Math.max(-zForce,0),normalOnSupportN:zForce,
        footBearingMPa:shear/(4.826*7),singleHostWallSleeveBearingMPa:shear/(11*(j?2:2.5))});
      retainAnchor(result,prefix+'upliftN',clamp,detail);
      retainAnchor(result,prefix+'compressionN',Math.max(-zForce,0),detail);
      retainAnchor(result,prefix+'torsionInclusiveShearN',shear,detail);
      retainAnchor(result,prefix+'combinedEquivalentN',Math.hypot(clamp,Math.sqrt(3)*shear),detail);
    }
  }
  function account(duration,required){
    for(const row of Object.values(duty)){
      row.allRecordedS+=duration;
      row.integratedTorqueSquaredNm2s+=required*required*duration;
      const over=Math.abs(required)-row.shaftLimitNm;
      if(over>0){
        row.aboveLimitS+=duration;row.aboveBlockS+=duration;
        row.longestOverLimitS=Math.max(row.longestOverLimitS,row.aboveBlockS);
        row.excessAngularImpulseNms+=over*duration;row.peakAbove=Math.max(row.peakAbove,over);
      }else row.aboveBlockS=0;
    }
  }
  function registerLoad(name,value,sample){
    if(!loadPeaks[name] || value>loadPeaks[name].value)loadPeaks[name]={value,...sample()};
  }
  for(let t=lip;t<clock.release+1.;t+=dt){
    ++stepCount;
    const bodyPitch=pitch*Math.max(0,Math.min(1,t-start+2));
    const u=Math.min(1,Math.max(0,(t-start)/duration));
    const sweep=(settings.worldDumpDeg-settings.stowDeg)*deg-pitch;
    const theta=settings.stowDeg*deg+sweep*u*u*(3-2*u),rate=u===0||u===1?0:sweep*6*u*(1-u)/duration;
    const c=Math.cos(theta),s=Math.sin(theta),[px,pz]=settings.pivot;
    const local=(x,z)=>[(x-px)*c+(z-pz)*s,-(x-px)*s+(z-pz)*c];
    let force=[-M*G*Math.sin(bodyPitch),0,-M*G*Math.cos(bodyPitch)],moment=[0,0,0];
    const elapsed=t-clock.carry;force[0]+=M*(adverse?1.5:.9)*(elapsed>0&&elapsed<.7?-1:elapsed>.9&&elapsed<1.6?1:0);
    const w=add(scale(L,1/J),a,dot(L,a)*(1/I-1/J));
    let panLoad=0,normalSum=0,sideLoads=[0,0],occupied=false,pivotMoment=0;
    const movingForce=[0,0,0],movingMoment=[0,0,0];
    function contact(n,arm,d,weight,mobile,side,isWall=false){
      if(d<0 || d>.025)return;
      if(mobile)occupied=true;
      const point=add(p,arm),own=add(v,cross(w,arm));
      if(mobile){own[0]+=rate*(point[2]-pz);own[2]-=rate*(point[0]-px);}
      const vn=dot(own,n),fn=Math.max(0,60000*d-1.2*Math.sqrt(60000*M)*vn)*weight;
      const slip=add(own,n,-vn),speed=norm(slip),f=add(scale(n,fn),slip,-mu*fn*Math.tanh(speed/.005)/Math.max(speed,1e-12));
      force=add(force,f);moment=add(moment,cross(arm,f));normalSum+=fn;peak=Math.max(peak,fn);
      if(mobile){
        panLoad+=fn*Math.max(0,n[2]);sideLoads[side>0?1:0]+=fn*Math.max(0,n[2]);
        if(reactions){
          const torque=cross([point[0]-px,point[1],point[2]-pz],f);
          for(let i=0;i<3;i++){movingForce[i]-=f[i];movingMoment[i]-=torque[i];}
        }
        pivotMoment+=(point[0]-px)*f[2]-(point[2]-pz)*f[0];
      }
      if(isWall){
        maxSide=Math.max(maxSide,fn);
        const previous=mobile?panImpact:guideImpact;
        if(!previous || fn>previous.normalN){
          const witness={timeS:t,normalN:fn,pointM:point,normal:n,reactionN:scale(f,-1),side};
          if(mobile)panImpact=witness;else guideImpact=witness;
        }
      }
    }
    const pan=settings.pan.map(([x,z])=>[px+c*x-s*z,pz+s*x+c*z]);
    function member(profile,mobile){
      for(let i=0;i<profile.length-1;i++){
        const [x0,z0]=profile[i],[x1,z1]=profile[i+1];
        const dx=x1-x0,dz=z1-z0,len=Math.hypot(dx,dz),tx=dx/len,tz=dz/len;
        const n=[-tz,0,tx],proj=dot(a,n),r=scale(add(n,a,-proj),-R/Math.sqrt(Math.max(1e-14,1-proj*proj)));
        const sx=p[0]+r[0],sz=p[2]+r[2],sy=p[1]+r[1],t0=(sx-x0)*tx+(sz-z0)*tz;
        const projectedAxis=a[0]*tx+a[2]*tz,base=(sx-x0)*n[0]+(sz-z0)*n[2];
        if(Math.abs(base)>H*Math.abs(proj)+.026 && (base>0||base<-.026))continue;
        for(let j=0;j<stations;j++){
          const h=H*(2*j/(stations-1)-1),u=t0+h*projectedAxis, y=sy+h*a[1],gap=base+h*proj;
          if(u<0||u>len||gap>0||gap<-.025)continue;
          const width=mobile?settings.floorHalfWidthM:settings.feedHalfWidthsM[i]+(settings.feedHalfWidthsM[i+1]-settings.feedHalfWidthsM[i])*u/len;
          if(Math.abs(y)>width || (mobile&&i===0&&Math.abs(y)<settings.wallSlotHalfWidthM))continue;
          const weight=(j===0||j===stations-1?.5:1)/(stations-1);
          contact(n,add(r,a,h),-gap,weight,mobile,y);
        }
      }
      // End edges admit actual annular material only. Examine finite side
      // stations across real sheet width with the adjacent faces' Voronoi test.
      for(let i=0;i<profile.length;i++){
        const [xx,zz]=profile[i];
        if(Math.hypot(p[0]-xx,p[2]-zz)>R+H*Math.hypot(a[0],a[2])+.002)continue;
        if(mobile && i===1)continue; // concave corner is constrained by its two faces
        const width=mobile?settings.floorHalfWidthM:settings.feedHalfWidthsM[i];
        for(let j=0;j<stations;j++){
          const yy=width*(2*j/(stations-1)-1);
          if(mobile&&i===0&&Math.abs(yy)<settings.wallSlotHalfWidthM)continue;
          const arm=[xx-p[0],yy-p[1],zz-p[2]],h=dot(arm,a),radial=add(arm,a,-h),rho=norm(radial);
          if(Math.abs(h)>H||rho>=R||rho<.0508)continue;
          const n=scale(radial,-1/rho);
          if(i>0 && (xx-profile[i-1][0])*n[0]+(zz-profile[i-1][1])*n[2]<0)continue;
          if(i+1<profile.length&&(profile[i+1][0]-xx)*n[0]+(profile[i+1][1]-zz)*n[2]>0)continue;
          contact(n,arm,R-rho,(j===0||j===stations-1?.5:1)/(stations-1),mobile,yy);
          maxRim=Math.max(maxRim,R-rho);
        }
      }
    }
    member(settings.feed,false);member(pan,true);
    if(guarded){
      // The same stationary entrance guide geometry that retained the skewed
      // admitted article on R5. End it before the rotating rear-wall path.
      const startX=-.340,endX=-.035,taper=(.158-.192)/(endX-startX),nd=Math.hypot(1,taper);
      for(const sign of [-1,1]){
        const n=[taper/nd,-sign/nd,0],proj=dot(n,a),rad=Math.sqrt(Math.max(1e-14,1-proj*proj));
        const arm=add(scale(a,-Math.sign(proj)*H),add(n,a,-proj),-R/rad);
        const tip=add(p,arm),dep=-(.192+taper*(tip[0]-startX)-sign*tip[1])/nd;
        if(tip[0]<startX||tip[0]>endX||dep<=0)continue;
        const j=tip[0]<-.255?0:1,low=settings.feed[j],high=settings.feed[j+1];
        const bed=low[1]+(tip[0]-low[0])*(high[1]-low[1])/(high[0]-low[0]);
        if(tip[2]>=bed-.006&&tip[2]<=bed+.120)contact(n,arm,dep,.7,false,sign,true);
      }
    }
    for(const sign of [-1,1]){
      const n=[0,-sign,0],d=dot(n,a),rad=Math.sqrt(Math.max(0,1-d*d));
      const arm=add(scale(a,-Math.sign(d)*H),add(n,a,-d),-R/Math.max(1e-12,rad));
      const pt=add(p,arm),[along,high]=local(pt[0],pt[2]);
      if(mode==='rim' && along>-.26 && along<.51 && high>.04 && high<.21){
        // The top is finite stock, including the outside 20 mm square stringer.
        // Examine both long upper edges against the finite cylindrical wall and
        // both annular end faces before a deep side-face contact can activate.
        const axisX=[c,0,s],up=[-s,0,c],peakDepth=[];
        for(const yy of [sign*.161,sign*.186]){
          const x0=-.09,x1=.34;
          const center=[px+up[0]*.120,yy,pz+up[2]*.120];
          const u=add(center,p,-1),h0=dot(u,a),dh=dot(axisX,a);
          const radial=add(u,a,-h0),slope=add(axisX,a,-dh);
          const A=dot(slope,slope),B=dot(radial,slope),C=dot(radial,radial);
          const breaks=[x0,x1,-B/A];
          if(Math.abs(dh)>1e-10)for(const h1 of [-H,H])breaks.push((h1-h0)/dh);
          for(const radius of [.0508,R]){
            const disc=B*B-A*(C-radius*radius);
            if(disc>=0)breaks.push((-B-Math.sqrt(disc))/A,(-B+Math.sqrt(disc))/A);
          }
          const knots=breaks.filter(q=>q>=x0&&q<=x1).sort((l,r)=>l-r);
          let best=null;
          for(let j=0;j<knots.length-1;j++)for(let k=0;k<=4;k++){
            const q=knots[j]+(knots[j+1]-knots[j])*k/4,h=h0+dh*q;
            const arm=add(u,axisX,q),v0=add(arm,a,-h),rho=norm(v0);
            const distances=[H-Math.abs(h),R-rho,rho-.0508];
            const dep=Math.min(...distances);
            if(dep<=0 || (best && best.dep>=dep))continue;
            const n=distances[0]===dep?scale(a,-Math.sign(h)):scale(v0,(distances[1]===dep?-1:1)/rho);
            // Normal force from an exposed upper corner must point into the
            // upper half space and away from that corner's stock.
            if(dot(n,up)<-.000001 || n[1]*(yy===sign*.161?-sign:sign)<-.000001)continue;
            best={arm,n,dep};
          }
          if(best)peakDepth.push(best);
        }
        if(peakDepth.length){
          const best=peakDepth.sort((l,r)=>r.dep-l.dep)[0];
          contact(best.n,best.arm,best.dep,.7,true,sign,true);
        }
        const d0=dot(up,a),rv=add(up,a,-d0),r0=scale(rv,-R/Math.max(1e-12,norm(rv)));
        for(let j=0;j<stations;j++){
          const hh=H*(2*j/(stations-1)-1),arm=add(r0,a,hh),v0=add(p,arm);
          const [xx,zz]=local(v0[0],v0[2]),yy=sign*v0[1],depth=.120-zz;
          if(xx>=-.09&&xx<=.340&&yy>=.161&&yy<=.186&&depth>=0&&depth<.012)
            contact(up,arm,depth,(j===0||j===stations-1?.5:1)/(stations-1),true,sign,true);
        }
      }
      if(along>=-.09&&along<=.340&&high>=0&&high<=settings.wallHeightM){
        contact(n,arm,sign*pt[1]-settings.wallLimitM,.7,true,sign,true);
      }
    }
    if(reactions){
      // Bearing reactions use the same instant and both moments about the
      // pivot. These are pipe-only radial loads on two ideal journal supports.
      const h=.195775;
      const [fx,fy,fz]=movingForce,[mx,my,mz]=movingMoment;
      const port=[(-fx+mz/h)/2,(-fz-mx/h)/2],starboard=[(-fx-mz/h)/2,(-fz+mx/h)/2];
      const [cx,,cz]=shop.movingShopCentreMm;
      const rx=c*(cx/1000-px)-s*(cz/1000-pz),rz=s*(cx/1000-px)+c*(cz/1000-pz);
      const driveAccel=(adverse?1.5:.9)*(elapsed>0&&elapsed<.7?-1:elapsed>.9&&elapsed<1.6?1:0);
      const mass=shop.movingShopStockKg,inertia=shop.movingStockInertiaKgM2;
      const alpha=t>=start&&t<start+duration?sweep*(6-12*u)/(duration*duration):0;
      // The physical opening axis is -Y. Its required actuator torque includes
      // the pan fabrication and the simultaneous, finite pipe contact wrench.
      const shopMomentY=mass*(rz*(-G*Math.sin(bodyPitch)+driveAccel)+rx*G*Math.cos(bodyPitch));
      const actuatorY=-inertia*alpha-my-shopMomentY;
      const sample=()=>({timeS:t,phase:t<clock.carry?'feed':t<start?'carry':'tip',
        shopPhaseDeg:theta/deg,pipeForceN:[...movingForce],pipeMomentNm:[...movingMoment],
        portRadialXZ_N:port,starboardRadialXZ_N:starboard,shopExternalMomentY_Nm:shopMomentY,
        shopInertialTorqueY_Nm:-inertia*alpha,requiredActuatorTorqueY_Nm:actuatorY});
      if(frameAnchors){
        anchorSamples++;
        const shopFx=mass*(-G*Math.sin(bodyPitch)+driveAccel+alpha*rz+rate*rate*rx);
        const shopFz=mass*(-G*Math.cos(bodyPitch)-alpha*rx+rate*rate*rz);
        const shopY=shop.movingShopCentreMm[1]/1000;
        for(const [tower,side,R] of [['port',1,port],['starboard',-1,starboard]]){
          const axial=side<0?fy:0;
          recordAnchors(sample,tower,side,-R[0],axial,-R[1],'pipe');
          const fraction=(1+side*shopY/h)/2;
          recordAnchors(sample,tower,side,-R[0]+fraction*shopFx,axial,-R[1]+fraction*shopFz,'pipeAndShopMass');
        }
      }
      registerLoad('pipeAxialN',Math.abs(fy),sample);
      registerLoad('pipeMomentY_Nm',Math.abs(my),sample);
      registerLoad('pipePortRadialN',Math.hypot(...port),sample);
      registerLoad('pipeStarboardRadialN',Math.hypot(...starboard),sample);
      registerLoad('shaftDutyNm',Math.abs(actuatorY),sample);
      if(windows){
        registerLoad('positiveShaftNm',actuatorY,sample);
        registerLoad('negativeShaftNm',-actuatorY,sample);
        previousActuatorY=actuatorY;account(dt,actuatorY);
      }
    }
    if(first===null&&normalSum>.1)first=t;
    if(occupied)lastOccupied=t;
    if(panLoad>.75*M*G&&sideLoads.every(load=>load>.03)&&norm(v)<.015&&norm(w)<.3)dwell+=dt;else dwell=0;
    if(ready===null&&dwell>.12)ready=t;
    if(t>=clock.carry&&atCarry===null)atCarry={wallS:t,ready,center:[...p],axis:[...a],dwell};
    if(elapsed>0&&elapsed<1.6){
      if(panLoad<.5*M*G||sideLoads.some(f=>f<.03))carryInterruption+=dt;else carryInterruption=0;
      maxCarryGap=Math.max(maxCarryGap,carryInterruption);
    }
    if(t>start&&openState===null)openState={wallS:t,p:[...p],a:[...a],v:[...v]};
    if(t>start){
      maxShaft=Math.max(maxShaft,Math.abs(pivotMoment));
      if(local(p[0],p[2])[0]>.340&&!occupied){
        if(!firstAir)firstAir={wallS:t,p:[...p],v:[...v],axis:[...a],omega:[...w],angularMomentum:[...L],thetaRad:theta};
        if(t-firstAir.wallS>.002){free=firstAir;break;}
      }else firstAir=null;
    }
    if(p[2]<.12||Math.abs(p[1])>.46||p[0]>.95){lost=true;break;}
    v=add(v,force,dt/M);p=add(p,v,dt);L=add(L,moment,dt);
    const newW=add(scale(L,1/J),a,dot(L,a)*(1/I-1/J));a=add(a,cross(newW,a),dt);a=scale(a,1/norm(a));
    final={wallS:t,p:[...p],axis:[...a]};
    // While genuinely seated, retain the observed state and recheck contact
    // immediately before the next external excitation. No moving interval is skipped.
    if(dwell>.3 && norm(v)<.0001 && norm(w)<.002){
      const next=t<clock.carry-.02?clock.carry-.02:t>clock.carry+1.65&&t<start-2.02?start-2.02:null;
      if(next!==null){if(windows)account(next-t,previousActuatorY);t=next;}
    }
  }
  return {scenario,id,pitch,dt,stations,yawSign:ys,offsetSign:dy,first,ready,atCarry,maxCarryGapS:maxCarryGap,openState,free,
          freeVsClockS:free?free.wallS-clock.release:null,peakQuadratureForceN:peak,maxSourcePanShaftTorqueNm:maxShaft,
          ...(reactions?{loadPeaks}:{}),...(windows?{duty}:{}),...(frameAnchors?{anchorRows,anchorSamples}:{}),guideImpact,panImpact,maxSideForceN:maxSide,maxRimDepthMm:1000*maxRim,lastOccupied,lost,final,stepCount};
}
const mode=process.argv[2]||'probe',runs={};
const cases=mode==='probe'?[['nominal','coral-A',1,1,0],['adverse','coral-A',1,1,.04]]:[];
if(mode!=='probe')for(const scenario of ['nominal','adverse'])for(const id of ['coral-A','coral-B'])
for(const [dy,ys] of scenario==='adverse'?[[1,1],[-1,1],[1,-1],[-1,-1]]:[[1,1]])for(const pitch of [-.04,.04])cases.push([scenario,id,dy,ys,pitch]);
for(const [scenario,id,dy,ys,pitch] of cases){
  for(const [dt,stations] of mode==='probe'?[[.0002,17]]:[[.0001,17],[.00005,33]]){
    const label=[scenario,id,dy,ys,pitch,dt,stations].join('/');
    const r=testCase(scenario,id,dt,dy,ys,pitch,stations);runs[label]=r;
    writeFileSync(dir+'single-pivot-spatial-'+(guarded?mode:'unguarded')+(reactions?'-installed-reactions':'')+(windows?'-duty':'')+(frameAnchors?'-frame-anchors':'')+'.json',JSON.stringify({sourceSha256:master.sha256,foundationSha256:foundation.nativeArchiveSha256,
               launchSha256:hash(sourceFile),settings,stationaryEntryGuides:guarded,scope:'Concept-level finite contact and matched feed to first exit. Shop dimensions, actual drive, post-impact path and complete score remain separate.',runs},null,2)+'\n');
    console.log(label,r.ready,r.atCarry?.dwell,r.maxCarryGapS,r.freeVsClockS,r.maxSideForceN,r.lost);
  }
}
