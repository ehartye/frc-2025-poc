// Revise the CLI-created base design to the B'x pocket package parameters (same frame, bumper and track as the
// pan baseline; different tool height, reach and opening). Usage: node revise.mjs PLUGIN base.json out.json
import fs from 'node:fs';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
const [plugin,src,out]=process.argv.slice(2);
const {assertDesign,canonicalStringify,reviseDesign}=await import(pathToFileURL(path.join(plugin,'scripts','design-artifact.mjs')).href);
const {checkDesignDestination}=await import(pathToFileURL(path.join(plugin,'scripts','active-design.mjs')).href);
const a=assertDesign(JSON.parse(fs.readFileSync(src,'utf8')));
const b=reviseDesign(a,x=>{Object.assign(x.design.parameters,{frameLengthM:.74,frameWidthM:.73,trackM:.5386,bumperBottomM:.043,bumperHeightM:.138,bumperDepthM:.075,height:.8867,reach:.1,toolHeight:.6,opening:.322,phase:'start'});
 x.provenance.limitations.push("Revision 2 sets the iteration 002 frame (740 x 730 mm, 17.25 mm rear cutback from the actual frame plan hull), the 538.6 mm steer-axis track and the 75 mm bumper stack at Z 43-181 mm, carried from the pan baseline as revisable premises. The B'x pocket gives an 886.7 mm stowed height (inlet walls), a 0.60 m nip throat (tool height), a 322 mm pocket width (opening) and the deploy-once lip's 100 mm reach beyond the frame. The registered package geometry is iterations/002/design/preliminary-pocket/layout.step.");});
checkDesignDestination(out);
fs.writeFileSync(out,canonicalStringify(b),{encoding:'utf8',flag:'wx'});
console.log('revision',b.revision);
