// Finish the public site after the shared builder has run: the rough-draft showcase model is withdrawn, so
// drop the files the builder still emits for it, publish the detailed design status tree and an entry page
// that features the current preliminary design release. The independent field pages stay as built.
import {createHash} from 'node:crypto';
import {cp,mkdir,readFile,readdir,rm,writeFile} from 'node:fs/promises';
import path from 'node:path';
const project=path.resolve(process.argv[2]), output=path.resolve(process.argv[3]);
const digest=data=>createHash('sha256').update(data).digest('hex');
const json=async name=>JSON.parse(await readFile(path.join(project,name),'utf8'));
const escape=text=>String(text).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'})[c]);
async function inventory(root){
  const names=[];
  for(const item of await readdir(root,{withFileTypes:true,recursive:true}))
    if(item.isFile())names.push(path.relative(root,path.join(item.parentPath,item.name)).split(path.sep).join('/'));
  return names.sort();
}

// The committed design index records the withdrawal; site-build.json lists what the builder still emits for it.
const build=await json('docs/robot/site-build.json');
const entryPoints=build.entryPoints, declared=build.withdrawnBuilderOutput;
const tree=path.join(project,'docs/detailed-design');
const index=JSON.parse(await readFile(path.join(tree,'designs.json'),'utf8'));
const reason=index.reason?.[declared.design];
if(index.designStatus[declared.design]!=='withdrawn'||!reason||index.current!==null)
  throw new Error('The design index must show '+declared.design+' withdrawn with its reason and no current detailed design');

// Stop publishing every withdrawn file the shared builder still emits. Nothing unexpected may remain.
for(const name of declared.builderFiles)await rm(path.join(output,name),{force:true});
for(const folder of new Set(declared.builderFiles.filter(name=>name.includes('/')).map(name=>name.split('/')[0])))
  if(!(await inventory(path.join(output,folder)).catch(()=>[])).length)await rm(path.join(output,folder),{recursive:true,force:true});
const left=await inventory(output);
if(JSON.stringify(left)!==JSON.stringify(['field-lab.html']))throw new Error('Unexpected output beside the field lab: '+left.join(', '));

const roleStudy=path.join(project,'experiments/swerve-drive/whole-robot-trade');
const rolePage=await readFile(path.join(roleStudy,'role-lab.html'));
const roleReport=JSON.parse(await readFile(path.join(roleStudy,'role-publication.json')));
const liveReport=await json('kickoff/live-publication.json');
if(digest(rolePage)!==liveReport.roleHtmlSha256||
    digest(await readFile(path.join(roleStudy,'role-demand.json')))!==roleReport.demandSha256)
  throw new Error('Role lab and selected demand must match the published study');
await writeFile(path.join(output,'role-lab.html'),rolePage);

await cp(tree,path.join(output,build.detailedRoot),{recursive:true,errorOnExist:true,force:false});

// The primary view follows the current preliminary design release and its checked still.
const releases=await json('docs/preliminary-design/releases.json'), release=releases.current;
const status=releases.designStatus?.[release]??'in-progress';
const words={'in-progress':'in progress',effective:'effective'}[status];
const statusNote={'in-progress':'It is in progress and still changing.',
  effective:'It is the effective preliminary design that detailed design builds on.'}[status];
if(!releases.releases.includes(release)||!words)throw new Error('The entry page needs a current preliminary design release');
const record=await json('docs/robot/preliminary-'+release+'.json');
const still=await readFile(path.join(project,'docs/robot/preliminary-'+release+'.png'));
const served=name=>readFile(path.join(project,'docs/preliminary-design/releases',release,name));
if(record.release!==release||digest(still)!==record.imageSha256||digest(await served('receipt.json'))!==record.receiptSha256||
    digest(await served('release.json'))!==record.releaseRecordSha256||
    digest(await served('review-scene.json.gz'))!==record.sceneSha256)
  throw new Error('The preliminary still differs from the current release');
const [entry,stillPath,selector]=entryPoints;
if(entryPoints.length!==3||entry!=='index.html'||!stillPath.endsWith('/assembly.png')||selector!==path.posix.dirname(stillPath)+'/index.html')
  throw new Error('Unexpected primary entry points in docs/robot/site-build.json');
const fields={
  heading:escape('Preliminary design · '+words+' · '+release),statusNote:escape(statusNote),alt:escape(record.caption.alt),
  title:escape(record.caption.title),pose:escape(record.caption.pose),scope:escape(record.caption.scope),
  reason:escape(reason),design:escape(declared.design),still:escape(stillPath),
  preliminary:escape(build.preliminaryRoot+'/'),detailed:escape(build.detailedRoot+'/')};
const fill=async name=>(await readFile(path.join(project,'docs/robot',name),'utf8')).replace(/\{\{(\w+)\}\}/g,(_,key)=>{
  if(!Object.hasOwn(fields,key))throw new Error('Unknown page field: '+key);
  return fields[key];
});
await mkdir(path.join(output,path.dirname(stillPath)),{recursive:true});
await writeFile(path.join(output,stillPath),still);
await writeFile(path.join(output,selector),await fill('no-detailed-model.html'));
await writeFile(path.join(output,entry),await fill('entry.html'));
console.log('Published the '+release+' preliminary design entry page; '+declared.design+' is withdrawn');
