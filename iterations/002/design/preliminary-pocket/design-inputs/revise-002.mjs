// Revise the layout-001 pocket design record for release layout-002 (same frame, bumper, track, height, reach and
// opening; the registered package moves to the release folder). Usage: node revise-002.mjs PLUGIN layout-001-design.json out.json
import fs from 'node:fs';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const [plugin,src,out]=process.argv.slice(2);
const {assertDesign,canonicalStringify,reviseDesign}=await import(pathToFileURL(path.join(plugin,'scripts','design-artifact.mjs')).href);
const a=assertDesign(JSON.parse(fs.readFileSync(src,'utf8')));
const b=reviseDesign(a,x=>{
 x.provenance.limitations.push("Revision 3 is release layout-002: no envelope parameter changes. The nip drive is a NEO 550 on a 36:1 planetary with a powered hold (the self-locking worm is removed), the hood gains a flared lead-in, the deployed lip stop is sprung, and the seated-pipe beam, flap-closed switch, lip-deployed switch and reef range sensor are placed. The registered package geometry is cad/preliminary-design/releases/layout-002/package.step, with the station chute, trough and first scored pipe in field-fixtures.step.");});
fs.writeFileSync(out,canonicalStringify(b),{encoding:'utf8',flag:'wx'});
console.log('revision',b.revision);
