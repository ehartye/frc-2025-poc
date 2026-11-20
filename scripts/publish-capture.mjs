// Install the checked project motion viewer after the shared showcase has verified the model.
import {createHash} from 'node:crypto';
import {readFile,writeFile,copyFile,mkdir} from 'node:fs/promises';
import path from 'node:path';
const project=path.resolve(process.argv[2]), output=path.resolve(process.argv[3]);
const source=path.join(project,'docs/robot/cad');
const pageFile=path.join(output,'index.html');
let page=await readFile(pageFile,'utf8');
const prior=page.match(/<iframe title="[^"]*"[^>]*src="(cad-[a-f0-9]{64})\/index\.html"/);
if(!prior) throw new Error('The standard showcase did not publish its CAD assembly');
const style=JSON.parse(await readFile(path.join(project,'highlights.json'))).stylesheet;
const css=await readFile(path.join(project,style.file));
const digest=data=>createHash('sha256').update(data).digest('hex');
if(digest(css)!==style.sha256) throw new Error('Changed showcase stylesheet');
const receiptFile=path.join(source,'receipt.json'), raw=await readFile(receiptFile), receipt=JSON.parse(raw);
const viewer=(await readFile(path.join(source,'index.html'),'utf8')).replace('</head>','<link rel="stylesheet" href="../evidence/'+style.sha256+'.css"></head>');
if(!viewer.includes('id="mechanism"')||!viewer.includes('id="stage-time"')) throw new Error('Missing controlled motion');
const folderName='capture-'+digest(viewer);
const dest=path.join(output,folderName);
await mkdir(dest,{recursive:true});
for(const [name,key] of [['assembly.png','imageSha256'],['assembly.stl.gz','meshSha256']]){
  if(digest(await readFile(path.join(source,name)))!==receipt[key]) throw new Error('Unregistered '+name);
  await copyFile(path.join(source,name),path.join(dest,name));
}
await writeFile(path.join(dest,'receipt.json'),raw);
await writeFile(path.join(dest,'index.html'),viewer);
page=page.replaceAll(prior[1]+'/index.html',folderName+'/index.html');
const climb=path.join(project,'cad/shallow-cage'), climbOutput=path.join(output,'climb');
const reopened=JSON.parse(await readFile(path.join(climb,'reopen.json'),'utf8'));
const archive=path.basename(reopened.source);
if (!reopened.opened || digest(await readFile(path.join(climb,archive)))!==reopened.sha256)
  throw new Error('Changed forward-jack Fusion archive');
await mkdir(climbOutput,{recursive:true});
const orbit=JSON.parse(await readFile(path.join(climb,'orbit.json'),'utf8'));
if (orbit.fusion!==reopened.sha256 || orbit.baseSha256!==receipt.meshSha256 ||
    orbit.kitSha256!==digest(await readFile(path.join(climb,'climb-kit.stl.gz'))))
  throw new Error('Changed registered climb geometry');
for(const entry of Object.values(reopened.added))
  if(orbit.sourceSteps[entry.step]!==entry.sha256 || digest(await readFile(path.join(climb,entry.step)))!==entry.sha256)
    throw new Error('Changed reopened group '+entry.step);
for(const file of [archive,'shallow-climb.webm','shallow-climb.png','orbit.json','orbit.js','climb-kit.stl.gz'])
  await copyFile(path.join(climb,file),path.join(climbOutput,file));
const climbPage=(await readFile(path.join(climb,'player.html'),'utf8')).replace('CLIMB_BASE_MESH',folderName+'/assembly.stl.gz');
await writeFile(path.join(output,'shallow-climb.html'),climbPage);
const anchor=/<a\b[^>]*href="(?:\.\/)?field-lab\.html"[^>]*>[^<]*<\/a>/;
if(!anchor.test(page)) throw new Error('Missing field lab navigation');
page=page.replace(anchor,match=>match+' <a href="shallow-climb.html">Explore the shallow-cage climb</a>');
const shop = path.join(project, 'cad/assembly-engineering/receiver-service/issue');
const installed = JSON.parse(await readFile(path.join(shop,'interfaces.json'),'utf8'));
const shopOrbit = JSON.parse(await readFile(path.join(shop,'orbit.json'),'utf8'));
const physical = JSON.parse(await readFile(path.join(shop,'engineering-reopen.json'),'utf8'));
const newArchive = installed.filename;
const collect = name => readFile(path.join(shop,name));
if (!physical.opened || physical.archiveSha256 !== digest(await collect(newArchive)) ||
    shopOrbit.issue !== digest(await collect('station-and-jack-interfaces.step')) ||
    shopOrbit.issue !== installed.outputSha256 || shopOrbit.baseSha256 !== digest(await collect('installed.stl.gz')) ||
    shopOrbit.installedBodies !== installed.robotSolids || physical.nativeBodyCount !== installed.robotSolids + 1)
  throw new Error('Changed engineering assembly or a stale public mesh');
const shopDir = path.join(output,'engineering');
await mkdir(shopDir,{recursive:true});
for (const name of ['installed.stl.gz','installed.png','assembly-drawings.pdf','orbit.json',newArchive])
  await copyFile(path.join(shop,name),path.join(shopDir,name));
await copyFile(path.join(shop,'player.html'),path.join(output,'engineering-assembly.html'));
page=page.replace(anchor,match=>match+' <a href="engineering-assembly.html">Explore the shop assembly</a>');
const roleStudy = path.join(project, 'experiments/swerve-drive/whole-robot-trade');
const rolePage = await readFile(path.join(roleStudy,'role-lab.html'));
const roleReport = JSON.parse(await readFile(path.join(roleStudy,'role-publication.json')));
const liveReport = JSON.parse(await readFile(path.join(project,'kickoff/live-publication.json')));
if (digest(rolePage) !== liveReport.roleHtmlSha256 ||
    digest(await readFile(path.join(roleStudy,'role-demand.json'))) !== roleReport.demandSha256)
  throw new Error('Role lab and selected demand must match the published study');
await writeFile(path.join(output,'role-lab.html'), rolePage);
page=page.replace(anchor,match=>match+' <a href="role-lab.html#roleLab">Compare station routes</a>');
await writeFile(pageFile,page);
console.log('Published checked capture viewer '+folderName);
