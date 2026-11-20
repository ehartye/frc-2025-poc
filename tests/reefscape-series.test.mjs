import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFile} from 'node:fs/promises';
import path from 'node:path';
import test from 'node:test';
import {pathToFileURL} from 'node:url';

const asset = name => readFile(new URL('../' + name, import.meta.url));
const plugin = process.env.AGENT_FRC_PLUGIN_ROOT;
if (!plugin) throw Error('Set AGENT_FRC_PLUGIN_ROOT to the documented rehearsal engine.');
const {createFieldRehearsal} = await import(pathToFileURL(path.resolve(plugin, 'scripts/field-rehearsal.mjs')));
const {createMatchSeries} = await import(pathToFileURL(path.resolve(plugin, 'scripts/rehearsal-series-core.mjs')));
const {createRehearsalGame} = await import(pathToFileURL(path.resolve(plugin, 'scripts/rehearsal-game.mjs')));
const bytes = await asset('kickoff/season-profile.json');
const profile = JSON.parse(bytes);
const receipt = JSON.parse(await asset('kickoff/match-series.json'));
const engine = createFieldRehearsal(profile);
const hash = data => createHash('sha256').update(data).digest('hex');

test('the kickoff and historical lab export and the reviewed public page use one self-contained file', async () => {
  const canonical = await asset('kickoff/field-lab.html');
  assert.deepEqual(canonical, await asset('experiments/six-wheel/kickoff/field-lab.html'));
  const showcase = JSON.parse(await asset('highlights.json'));
  assert.equal(hash(canonical), showcase.fieldLab.sha256);
  assert.equal(receipt.profileSha256, hash(bytes));
  const browser = JSON.parse(await asset('kickoff/series-browser.json'));
  assert.equal(browser.profileSha256,receipt.profileSha256);
  const live = JSON.parse(await asset('kickoff/live-publication.json'));
  assert.equal(live.fieldHtmlSha256,showcase.fieldLab.sha256);
  assert.equal(browser.receiptSha256,hash(await asset('kickoff/match-series.json')));
  assert.equal(browser.stationCount,profile.rehearsal.stations.length);
  for (const [i,match] of receipt.matches.entries())
    assert.ok(browser.results[i].endsWith('Blue '+match.game.score.blue+' / Red '+match.game.score.red));
});

test('six whole games reproduce their clocks, score events, traffic and finite stocks', () => {
  const series = createMatchSeries(engine, profile.rehearsal.series.matches);
  series.advance(900);
  assert.equal(series.done, true);
  const historical = JSON.parse(JSON.stringify(series.results));
  for (const match of historical) {
    for (const period of match.periods) {
      for (const robot of period.robots) {
        // The later role lab additionally records path length in inches.
        if ('travelIn' in robot) {
          assert.ok(Number.isFinite(robot.travelIn) && robot.travelIn >= 0);
          delete robot.travelIn;
        }
      }
    }
  }
  assert.deepEqual(historical, receipt.matches);
  const stations = new Map(profile.rehearsal.stations.map(s => [s.id,s]));
  for (const m of series.results) {
    assert.deepEqual(m.periods.map(p => [p.phase,p.seconds]), [['AUTO',15],['TELEOP',135]]);
    assert.equal(m.roster.length,6);
    const count = type => [...stations.values()].filter(s => s.piece === type && s.kind === 'retrieval')
      .reduce((total,s) => total + m.game.initial[s.id],0);
    assert.equal(count('CORAL'),126);
    assert.equal(count('ALGAE'),18);
    const surviving = [...Object.values(m.game.stock).flat(), ...Object.values(m.game.cargo).flat(),
      ...m.game.terminal, ...m.game.returning.map(r => r.piece)];
    assert.equal(surviving.length,144);
    assert.equal(new Set(surviving.map(p => p.id)).size,144);
    const tally = {blue:0,red:0};
    for (const e of m.game.events.filter(e => e.operation === 'attempt')) {
      const s = stations.get(e.station);
      assert.equal(e.points,e.outcome === 'credited' ? s.points[e.phase] * e.pieces.length : 0);
      tally[s.side] += e.points;
    }
    assert.deepEqual(tally,m.game.endgame.ordinaryScore);
    for(const side of ["red","blue"]) assert.equal(tally[side]+m.game.endgame.terminalScore[side],m.game.score[side]);
  }
  assert.deepEqual(receipt.matches.slice(0,3).map(m=>m.game.score), [{blue:17,red:24},{blue:24,red:47},{blue:17,red:13}]);
  const trip = receipt.matches[1].game.events.filter(e => e.phase==='TELEOP' && e.robot==='R1' && e.outcome==='credited');
  assert.equal(new Set(trip.flatMap(e=>e.pieces)).size,15);
  const focus = receipt.matches[2].game;
  assert.equal(focus.cargo.R1.length,1);
  assert.deepEqual(focus.events.filter(e=>e.robot==='R1').map(e=>[e.phase,e.outcome]), [['TELEOP','acquired']]);
});

test('moving only the partner hold changes the registered traffic and fifteen deliveries', async () => {
  const counter = JSON.parse(await asset('kickoff/traffic-control.json'));
  assert.equal(hash(bytes),counter.profileSha256);
  const variant=structuredClone(profile);
  variant.rehearsal.programs['keep-lane'].steps[0].at=counter.counterfactualHoldIn;
  const game=createMatchSeries(createFieldRehearsal(variant),[variant.rehearsal.series.matches[1]]);
  game.advance(150);
  const original=game.results[0];
  assert.deepEqual(original.game.score,counter.counterfactualScore);
  const robot=original.periods[1].robots.find(r=>r.id==='R1');
  assert.equal(robot.waitSeconds,counter.counterfactualRobot.waitSeconds);
  assert.equal(robot.blocked,counter.counterfactualRobot.blocked);
  assert.equal(robot.visits,counter.counterfactualRobot.visits);
  assert.deepEqual(receipt.matches[1].game.score,counter.issuedScore);
});

test('a robot with CORAL cannot score in the ALGAE processor and a missed trough scores zero', () => {
  const g = createRehearsalGame(profile);
  const r = {...engine.defaults('blue',1),preload:{station:'blue-chute-low',quantity:1}};
  const sim = engine.create([r]);
  const robot = sim.robots[0];
  function attempt(id,outcome) {
    const station=profile.rehearsal.stations.find(s=>s.id===id);
    robot.p=station.at;
    g.apply(sim,robot,{operation:'attempt',station:id,outcome},1);
  }
  attempt('blue-processor','credited');
  assert.equal(sim.game.events.at(-1).outcome,'no-compatible-cargo');
  assert.equal(sim.game.cargo.B1[0].id,'blue-chute-low/1');
  attempt('blue-l1','missed');
  assert.equal(sim.game.events.at(-1).points,0);
  assert.equal(sim.game.terminal[0].id,'blue-chute-low/1');
});


test('configured cage height, cutoff and support each gate endgame credit', async () => {
  const games=receipt.matches.slice(3).map(m=>m.game.endgame);
  const row=(game,id)=>game.attempts.find(r=>r.robot===id);
  assert.deepEqual(games.map(m=>m.terminalScore),[{blue:18,red:0},{blue:0,red:12},{blue:2,red:6}]);
  assert.equal(row(games[0],'R3').outcome,'failed');
  assert.equal(row(games[1],'B3').outcome,'service-incomplete');
  assert.equal(row(games[1],'R1').outcome,'did-not-arrive');
  const m=structuredClone(profile.rehearsal.series.matches[3]);
  const r=m.roster.find(row=>row.side==='blue' && row.slot===3);
  r.endgame.target='blue-2-deep';r.endgame.action='deep';
  assert.throws(()=>{const s=createMatchSeries(engine,[m]);s.advance(150);}, /setup/i);
  const barge=JSON.parse(await asset('kickoff/barge-assessment.json'));
  assert.deepEqual(barge.matches.slice(3).map(m=>m.assessmentWallS),[156,154.5,153.5]);
  assert.deepEqual(barge.matches[3].bargeQualificationRP,{blue:1,red:0});
});
