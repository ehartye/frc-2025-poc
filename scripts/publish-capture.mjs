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
await writeFile(pageFile,page);
console.log('Published checked capture viewer '+folderName);
