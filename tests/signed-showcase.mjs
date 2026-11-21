// Exercise the actual pre-upload command using an unrelated synthetic issue.
// Use the ordinary native teaching coupon and its independent reviewer public key.
import assert from 'node:assert/strict';
import {createHash,generateKeyPairSync,sign} from 'node:crypto';
import {spawnSync} from 'node:child_process';
import {copyFileSync,mkdirSync,mkdtempSync,readFileSync,rmSync,writeFileSync} from 'node:fs';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

const project=fileURLToPath(new URL('../',import.meta.url));
const args=process.argv.slice(2);
if(args.length!==6)throw Error('Usage: signed-showcase.mjs VERIFIER SOURCE_ROOT SOURCE_PACKET STAGED_ROOT STAGED_PACKET TRUSTED_COUPON_PUBLIC_KEY');
const [verifier,sourceRoot,sourceFile,stageRoot,stageFile,keyFile]=args.map(arg=>path.resolve(arg));
const verifierScript=path.join(verifier,'scripts/public-model-release.mjs');
const coupon='mechanical-coupon';
const verified=spawnSync(process.execPath,[verifierScript,'verify-pair',sourceRoot,sourceFile,stageRoot,stageFile,keyFile,coupon],{encoding:'utf8'});
assert.equal(verified.status,0,'Expected a genuinely issued, matching teaching coupon: '+verified.stderr);
const sourceIssue=JSON.parse(readFileSync(sourceFile)),stageIssue=JSON.parse(readFileSync(stageFile));
const pem=readFileSync(keyFile,'utf8');
const sourceNames=sourceIssue.body.files.map(r=>r.path),stageNames=stageIssue.body.files.map(r=>r.path);
assert.ok(sourceNames.length && stageNames.length);
const signedPreview=stageNames.find(n=>n.endsWith('.png'));
assert.ok(signedPreview,'The issue must include its actual CAD still');
const signed=body=>({body,signature:sign(null,Buffer.from(JSON.stringify(body)),privateKey).toString('base64')});
const bash=process.env.BASH_BINARY || (process.platform==='win32'?'C:/Program Files/Git/bin/bash.exe':'bash');
const folder=mkdtempSync(path.join(tmpdir(),'synthetic-model-packet-'));
const put=(name,bytes)=>{
  const dest=path.join(folder,name);
  mkdirSync(path.dirname(dest),{recursive:true});writeFileSync(dest,bytes);
};
const json=(name,value)=>put(name,JSON.stringify(value));
const {publicKey,privateKey}=generateKeyPairSync('ed25519');
const mismatchKey=publicKey.export({format:'pem',type:'spki'}).toString();
const git=(...args)=>{
  const r=spawnSync('git',args,{cwd:path.join(folder,'project'),encoding:'utf8'});
  if(r.status!==0)throw Error(r.stderr);
  return r.stdout.trim();
};
const source='project/.github/model-release/source.json';
const staged='project/.github/model-release/staged.json';
try{
  json('project/.github/public-model-policy.json',{schema:'agent-frc/public-model-policy@1',project:coupon,
    publicRoots:['model'],modelFiles:[...sourceNames,'model/view.html']});
  json('approved/.github/staged-model-policy.json',{schema:'agent-frc/staged-model-policy@1',project:coupon,
    mutableFieldPages:['field.html'],modelFiles:[...stageNames,'view.html']});
  copyFileSync(path.join(folder,'approved/.github/staged-model-policy.json'),path.join(folder,'project/.github/staged-model-policy.json'));
  mkdirSync(path.join(folder,'runner/field'),{recursive:true});
  put('approved/scripts/cad-readmes.py',"from pathlib import Path\nassert Path('project/model/view.html').is_file()\n");
  put('project/model/view.html','synthetic historical view');
  for(const n of sourceNames)put('project/'+n,'historical coupon fixture');
  for(const root of ['runner/approved','runner/showcase']){
    for(const n of stageNames)put(root+'/'+n,'historical coupon fixture');
    put(root+'/view.html','synthetic historical view');put(root+'/field.html','original field fixture');
  }
  git('init','-q');git('config','core.autocrlf','false');git('add','.');
  git('-c','user.name=Synthetic Test','-c','user.email=synthetic@example.invalid','commit','-qm','Synthetic baseline');
  const workflow=readFileSync(path.join(project,'.github/workflows/robot-showcase.yml'),'utf8').replaceAll('\r\n','\n');
  const block=workflow.split('      - name: Check the source inventory and every final public byte\n')[1]
    .split('      - if:')[0].split('        run: |\n')[1].trimEnd();
  const stationTemplate=readFileSync(path.join(project,'.github/showcase-station.json'));
  const station=JSON.parse(stationTemplate);
  station.approvedCommit=git('rev-parse','HEAD');
  const shared=path.join(verifier,'scripts/local-showcase-release.mjs');
  const digest=spawnSync(process.execPath,[shared,'digest',path.join(folder,'runner/approved')],{encoding:'utf8'});
  assert.equal(digest.status,0,digest.stderr);
  station.approvedSiteSha256=digest.stdout.trim();
  const fixture=Buffer.from(JSON.stringify(station));
  put('project/.github/showcase-station.json',fixture);
  const hash=bytes=>createHash('sha256').update(bytes).digest('hex');
  const pinned=hash(stationTemplate);
  assert.ok(block.includes(pinned),'Workflow must verify the actual station configuration');
  const script=block.split('\n').map(l=>l.slice(10)).join('\n').replace(pinned,hash(fixture));
  put('gate.sh',script+'\n');
  mkdirSync(path.join(folder,'verifier/scripts'),{recursive:true});
  for(const name of ['public-model-release.mjs','local-showcase-release.mjs'])
    copyFileSync(path.join(verifier,'scripts',name),path.join(folder,'verifier/scripts',name));
  function check(label,success,expected='',trustedKey=pem){
    git('add','.');
    if(git('diff','--cached','--name-only'))
      git('-c','user.name=Synthetic reviewer','-c','user.email=synthetic@example.invalid','commit','-qm',label);
    const r=spawnSync(bash,['gate.sh'],{cwd:folder,encoding:'utf8',env:{...process.env,
      REVIEW_PUBLIC_KEY:trustedKey,RUNNER_TEMP:'runner',FRC_VERIFIED_FIELD_ROOT:'runner/field'}});
    const output=r.stdout+r.stderr;
    assert.equal(r.status===0,success,label+'\n'+output);assert.ok(output.includes(expected),label+'\n'+output);
    console.log(label+' passed');
  }
  check('Historical unchanged',true);
  put('runner/showcase/field.html','regenerated field');put('runner/field/field.html','regenerated field');
  check('Independent field-only update',true);
  for(const n of sourceNames)put('project/'+n,readFileSync(path.join(sourceRoot,n)));
  for(const n of stageNames)put('runner/showcase/'+n,readFileSync(path.join(stageRoot,n)));
  check('Unsigned model change',false,'without matching approved issue');
  put(source,readFileSync(sourceFile));check('Missing staged packet',false);
  put(staged,readFileSync(stageFile));check('Ordinary signed source and staged promotion',true);
  put('runner/showcase/view.html','unreviewed selector');check('Uncovered staged selector',false,'Unreviewed changed');
  put('runner/showcase/view.html','synthetic historical view');
  put('runner/showcase/'+signedPreview,'substituted after issue');check('Substituted signed media',false,'Changed signed public asset');
  put('runner/showcase/'+signedPreview,readFileSync(path.join(stageRoot,signedPreview)));
  // Exercise the same-issue test with its own temporary key, preserving
  // passing signatures on both synthetic packets while changing one issue.
  json(source,signed(sourceIssue.body));
  const altered=stageIssue.body.stepSha256.replace(/^./,c=>c==='a'?'b':'a');
  json(staged,signed({...stageIssue.body,stepSha256:altered}));
  check('Different independently signed assembly',false,'different engineering issue',mismatchKey);
  const retired=sourceIssue.body.schema.replace(/@(\d+)$/,(_,v)=>'@'+(Number(v)-1));
  assert.notEqual(retired,sourceIssue.body.schema);
  json(source,signed({...sourceIssue.body,schema:retired}));
  json(staged,signed({...stageIssue.body,schema:retired}));
  check('Both signed packets use the retired schema',false,'Invalid model receipt',mismatchKey);
  put(staged,readFileSync(stageFile));
  json(source,{...sourceIssue,signature:Buffer.alloc(64).toString('base64')});
  check('Invalid source signature',false,'Invalid reviewer signature');
  put(source,readFileSync(sourceFile));
  put('project/model/view.html','unsigned revised source');check('Uncovered source selector',false,'without matching approved issue');
}finally{
  assert.equal(path.dirname(path.resolve(folder)),path.resolve(tmpdir()));
  assert.ok(path.basename(folder).startsWith('synthetic-model-packet-'));
  rmSync(folder,{recursive:true,force:true});
}
