import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {join,resolve} from 'node:path';
import {pathToFileURL} from 'node:url';

if (!process.env.AGENT_FRC_PLUGIN_ROOT) throw Error('Set AGENT_FRC_PLUGIN_ROOT to the installed plugin root');
const {checkFaultMomentumTransition: check} = await import(pathToFileURL(join(resolve(process.env.AGENT_FRC_PLUGIN_ROOT),'scripts','planar-observation.mjs')));
const study = JSON.parse(readFileSync(new URL('./study.json',import.meta.url)));
const sample = study.cases.nominal['coral-A'].pickup;
const impact = sample.impact;
const mass = .816466266, radius = .05715, inertia = study.polar_inertia_kg_m2;
const make = (beforeVelocity, afterVelocity, beforeYawRate, afterYawRate, linearImpulse, angularImpulse) =>
    ({mass,inertia,beforeVelocity,afterVelocity,beforeYawRate,afterYawRate,linearImpulse,angularImpulse,linearTolerance:1e-10,angularTolerance:1e-10});
// In this planar screen the vertical drawing axis replaces planar Y; positive spin is CCW in XZ.
const outgoing=[-sample.ramp.speed*Math.cos(35*Math.PI/180),-sample.ramp.speed*Math.sin(35*Math.PI/180)];
const reset=make(outgoing,outgoing,sample.ramp.spin,0,[0,0],0);
const impulse=impact.floor_tangent_impulse_Ns;
const before=[impact.vx_before_mps,impact.vz_before_mps], after=[before[0]+impulse/mass,0];
const floor=make(before,after,impact.spin_before_rad_s,study.cases.nominal['coral-A'].phase_entry_spin_rad_s.capture[1],
                 [impulse,-mass*before[1]],radius*impulse);
const broken=check(reset), allowed=check(floor), omitted=check({...floor,angularImpulse:0});
assert.equal(broken.consistent,false);
assert.equal(allowed.consistent,true);
assert.equal(omitted.consistent,false);
console.log(JSON.stringify({instantaneousLipReset:broken,physicalShelfImpulse:allowed,omittedShelfTorque:omitted},null,2));
