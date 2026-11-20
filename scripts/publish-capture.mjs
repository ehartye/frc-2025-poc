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
await writeFile(pageFile,page);
console.log('Published checked capture viewer '+folderName);
