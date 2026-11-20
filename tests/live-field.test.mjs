import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFile} from 'node:fs/promises';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
import test from 'node:test';

const root = process.env.AGENT_FRC_PLUGIN_ROOT;
const load = name => import(pathToFileURL(path.resolve(root, 'scripts', name)));
const {createFieldRehearsal} = await load('field-rehearsal.mjs');
const {createMatchSeries} = await load('rehearsal-series-core.mjs');
const read = name => readFile(new URL('../' + name, import.meta.url));
const hash = data => createHash('sha256').update(data).digest('hex');
const receipt = JSON.parse(await read('kickoff/live-publication.json'));
const profile = JSON.parse(await read('kickoff/live-profile.json'));

test('all live entry points use the reviewed field, role, and renderer inputs', async () => {
  for (const [key,file] of [
    ['fieldProfileSha256','kickoff/live-profile.json'], ['fieldHtmlSha256','kickoff/field-lab.html'],
    ['fieldHtmlSha256','experiments/six-wheel/kickoff/field-lab.html'],
    ['roleProfileSha256','experiments/swerve-drive/whole-robot-trade/live-profile.json'],
    ['roleStudySha256','experiments/swerve-drive/whole-robot-trade/refinement-study.json'],
    ['roleHtmlSha256','experiments/swerve-drive/whole-robot-trade/role-lab.html'],
  ]) assert.equal(hash(await read(file)), receipt[key]);
});

test('every match conserves all 144 IDs through timed supply and reveals its opening', () => {
  const engine = createFieldRehearsal(profile);
  const series = createMatchSeries(engine, profile.rehearsal.series.matches);
  assert.equal(series.sim.robots.length, 6);
  assert.equal(series.sim.game.motion.pieces.length, 12);
  series.advance(900);
  assert.deepEqual(series.results.map(m => ({id:m.id,score:m.game.score})), receipt.matches);
  for (const {game,roster} of series.results) {
    for (const side of ['blue','red']) assert.deepEqual(roster.filter(r=>r.side===side).map(r=>r.slot).sort(),[1,2,3]);
    assert.equal(Object.keys(game.opening).length, 6);
    const all = [...Object.values(game.reserve).flat(), ...Object.values(game.stock).flat(),
      ...Object.values(game.cargo).flat(), ...game.terminal, ...game.returning.map(r=>r.piece)];
    assert.equal(all.length,144);
    assert.equal(new Set(all.map(p=>p.id)).size,144);
    for(const physical of game.motion.pieces) assert.ok(game.stock[physical.station].some(p=>p.id===physical.id));
    for (const e of game.events.filter(e => e.robot.includes('processor-attendant'))) {
      const side = e.robot.includes('/blue-') ? 'blue' : 'red';
      const opposite = side === 'blue' ? 'red' : 'blue';
      const source = game.events.find(row => row.station === opposite+'-processor' &&
        row.outcome === 'credited' && row.pieces[0] === e.pieces[0]);
      assert.ok(source);
      assert.ok(e.timeS >= source.timeS + 6 - 1e-8);
      assert.equal(e.station, side+'-net');
      assert.equal(e.outcome, 'unresolved');
      assert.equal(e.points, 0);
    }
    const lastRelease = new Map();
    const releases = game.events.filter(e => e.operation === 'release');
    assert.ok(releases.length);
    for(const e of releases) {
      assert.equal(e.phase,'TELEOP');
      assert.ok(e.timeS - (lastRelease.get(e.robot) ?? 0) >= 3-1e-8);
      lastRelease.set(e.robot,e.timeS);
    }
  }
});
