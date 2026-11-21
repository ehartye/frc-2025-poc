import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
const root=process.env.AGENT_FRC_HISTORICAL_FIELD_ROOT || process.env.AGENT_FRC_PLUGIN_ROOT;
if(!root)throw Error('Set AGENT_FRC_PLUGIN_ROOT to the compatible plugin directory.');
const {createFieldRehearsal}=await import(pathToFileURL(path.resolve(root,'scripts/field-rehearsal.mjs')));
const {routeStudy}=await import(pathToFileURL(path.resolve(root,'scripts/kickoff-routes.mjs')));
const profile=JSON.parse(await readFile(process.env.FRC_REHEARSAL_PROFILE || new URL('../kickoff/season-profile.json',import.meta.url),'utf8'));
const engine=createFieldRehearsal(profile);
const body={length:30,width:30,height:30,bumper:0,heading:0};
const square=(width)=>({...body,length:width,width});
const close=(a,b,tol=1e-5)=>assert.ok(Math.abs(a-b)<tol,a+' vs '+b);
const both=()=>['blue','red'].map(s=>engine.defaults(s,1));

test('release-day corner planes and polyline footprint match independent physical fixtures',()=>{
  const layout=engine.geom;
  assert.equal(layout.boundary.length,8);
  assert.match(engine.assessRoute([[45,42],[100,75]],body).obstacle,/tag 12/);
  assert.equal(engine.assessRoute([[48,45]],body).status,'clear');
  assert.match(engine.assessRoute([[48,45]],{...body,bumper:3.25}).obstacle,/tag 12/);
  const net=routeStudy(profile,'ALGAE','net','TELEOP',null,30).line;
  const finding=engine.assessRoute(net,body);
  assert.equal(finding.obstacle,'Blue reef');
  close(finding.distance,10.14407713);
  assert.equal(engine.assessRoute([net[0],[228,101],net.at(-1)],body).status,'clear');
  const processor=routeStudy(profile,'ALGAE','processor','TELEOP',null,30).line;
  assert.equal(engine.assessRoute(processor,{...body,bumper:3.25}).status,'clear');
  const unsupported=createFieldRehearsal({...profile,rehearsal:undefined});
  assert.equal(unsupported.assessRoute(net,body).status,'unknown');
  assert.equal(engine.assessRoute([[NaN,0]],body).status,'unknown');
});

test('occupied width and overhead height change the availability of a cage crossing',()=>{
  const x=profile.field.length/2,y=profile.field.width/2+(41.5+84.375)/2;
  const through=(center)=>[[x-80,center],[x+80,center]];
  assert.equal(engine.assessRoute(through(y),square(30)).status,'clear');
  assert.match(engine.assessRoute(through(y),square(36)).obstacle,/cage/);
  assert.match(engine.assessRoute(through(y),{...body,bumper:3.25}).obstacle,/cage/);
  const center=profile.field.width/2+84.375;
  assert.equal(engine.assessRoute(through(center),{...body,height:24},'shallow').status,'clear');
  assert.match(engine.assessRoute(through(center),{...body,height:24},'deep').obstacle,/cage/);
  assert.match(engine.assessRoute(through(center),body,'shallow').obstacle,/cage/);
  assert.match(engine.assessRoute(through(y),{...body,height:56}).obstacle,/overhead/);
  assert.match(engine.assessRoute(through(profile.field.width/2),body).obstacle,/support/);
});

test('continuous relative sweep detects crossing, exchanging ends, rotation and departure at contact',()=>{
  const contact=engine.geom.robotContact;
  close(contact([0,0],[100,0],square(10),[60,-60],[60,40],square(10)),0.5);
  close(contact([0,0],[100,0],square(10),[100,0],[0,0],square(10)),.45);
  assert.equal(contact([0,0],[100,0],square(10),[0,11],[100,11],square(10)),null);
  assert.equal(contact([0,0],[-10,0],body,[30,0],[30,0],body),null);
  assert.equal(contact([0,0],[0,0],{...body,heading:45},[35,0],[35,0],body),0);
  assert.equal(engine.assessRoute([[115,158.5],[252,158.5]],body).obstacle,'Blue reef');
});

test('exclusive opening queues competing arrivals and is physically unavailable to an oversized robot',()=>{
  const sim=engine.create(both());sim.paused=false;engine.advance(sim,15);
  assert.ok(sim.robots.every(r=>r.visits===1));
  assert.ok(sim.robots.some(r=>r.wait>0));
  const events=sim.log.map(e=>e.message);
  assert.ok(events.includes('B1 requested '+profile.rehearsal.resourceLabel));
  assert.ok(events.indexOf('B1 entered '+profile.rehearsal.resourceLabel)>events.indexOf('R1 cleared '+profile.rehearsal.resourceLabel));
  assert.ok(sim.hotspots.length);
  const big=engine.create([{...engine.defaults('blue',1),width:36,bumper:0}]);
  big.paused=false;engine.advance(big,15);
  assert.equal(big.robots[0].visits,0);
  assert.match(big.robots[0].blocked,/No footprint-clear path/);
});

test('clock freezes on each active deadline and retained footprint positions replay deterministically',()=>{
  const start=()=>{const s=engine.create(both());s.paused=false;return s;};
  const sim=start();engine.advance(sim,18);
  assert.equal(sim.ticks,300);assert.ok(sim.ended && sim.paused);
  const frozen=JSON.stringify(sim.robots);engine.advance(sim,3);assert.equal(JSON.stringify(sim.robots),frozen);
  const positions=sim.robots.map(r=>r.p);
  engine.nextPhase(sim);assert.equal(sim.phase,'TELEOP');assert.equal(sim.ticks,0);
  assert.deepEqual(sim.robots.map(r=>r.p),positions);
  sim.paused=false;engine.advance(sim,141);assert.equal(sim.ticks,2700);assert.ok(sim.ended);
  assert.deepEqual(sim.phaseHistory.map(h=>h.seconds),[15,135]);
  assert.throws(()=>engine.nextPhase(sim),/Finish the active period/);
  const replay=start();for(let i=0;i<150;i++)engine.advance(replay,.1);
  assert.equal(JSON.stringify(replay.robots),frozen);
  assert.throws(()=>engine.create([engine.defaults('red',1),engine.defaults('red',1)]),/unique/);
});

test('six moving robots stay separate between ticks using independent axis interval checks',()=>{
  const roster=[];
  for(const side of ['blue','red'])for(const n of [1,2,3])roster.push({...engine.defaults(side,n),speed:n===1?220:140});
  const sim=engine.create(roster);
  // An independent 1D moving interval calculation for default 0/180-degree headings.
  // Simultaneous strict overlap on both axes at any interior time would be a collision.
  const crossing=(a,b,c,d,reach)=>{
    const p=a-c,delta=(b-a)-(d-c);
    if(Math.abs(delta)<1e-9)return Math.abs(p)<reach-1e-6?[0,1]:[2,1];
    const t0=(-reach+1e-6-p)/delta,t1=(reach-1e-6-p)/delta;
    return [Math.min(t0,t1),Math.max(t0,t1)];
  };
  for(const steps of [300,2700]){
    sim.paused=false;
    for(let tick=0;tick<steps;tick++){
      const before=sim.robots.map(r=>r.p);
      engine.advance(sim,.05);
      for(let i=0;i<6;i++)for(let j=i+1;j<6;j++){
        const a=sim.robots[i],b=sim.robots[j];
        const bounds=[0,1].map(k=>crossing(before[i][k],a.p[k],before[j][k],b.p[k],
          (k?(a.width+b.width):(a.length+b.length))/2+a.bumper+b.bumper));
        assert.ok(Math.max(0,bounds[0][0],bounds[1][0])>=Math.min(1,bounds[0][1],bounds[1][1]));
      }
    }
    if(steps===300)engine.nextPhase(sim);
  }
});
