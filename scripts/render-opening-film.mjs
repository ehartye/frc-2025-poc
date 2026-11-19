import {spawn} from 'node:child_process';
import {once} from 'node:events';
import {mkdir, mkdtemp, readFile, rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import path from 'node:path';
import {pathToFileURL} from 'node:url';

// Render the same released field and wheel views as the student replay, using recorded
// clock samples at 10 frames/s. Only the camera layout and labels are added here.
function filmFrame(k) {
  document.getElementById('clock').value=k;
  document.getElementById('clock').dispatchEvent(new Event('input'));
  const sheet=document.createElement('canvas');
  sheet.width=1240; sheet.height=560;
  const ctx=sheet.getContext('2d');
  ctx.fillStyle='#101820'; ctx.fillRect(0,0,1240,560);
  ctx.fillStyle='#e3eef1';ctx.font='bold 30px system-ui';
  ctx.fillText('Station approach',24,47);ctx.fillText('Module steering',940,47);
  ctx.drawImage(document.querySelector('#field'),18,67,900,423);
  ctx.drawImage(document.querySelector('#wheel'),932,95,290,275);
  ctx.font='20px system-ui';ctx.fillStyle='#b8ced3';
  ctx.fillText(document.querySelector('#event').value,24,525);
  ctx.fillText(document.querySelector('#wall').value,940,433);
  return sheet.toDataURL('image/jpeg',.91).split(',')[1];
}

const [browser,ffmpeg,output]=process.argv.slice(2);
if(!browser||!ffmpeg||!output) throw Error('Usage: node render-opening-film.mjs CHROMIUM FFMPEG OUTPUT.webm');
const source=path.resolve(import.meta.dirname,'../experiments/swerve-drive/motion/index.html');
const profile=await mkdtemp(path.join(tmpdir(),'swerve-film-'));
const chrome=spawn(browser,['--headless=new','--remote-debugging-port=0','--no-first-run',
  '--user-data-dir='+profile,'about:blank'],{windowsHide:true,stdio:'ignore'});
const closed=once(chrome,'exit');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
let ws,encoder;
try {
  let port;
  for(let i=0;!port&&i<100;i++){
    try {port=(await readFile(path.join(profile,'DevToolsActivePort'),'utf8')).split(/\r?\n/)[0];}
    catch {await delay(100);}
  }
  if(!port)throw Error('Could not reach the temporary Chromium profile.');
  const targets=await(await fetch('http://127.0.0.1:'+port+'/json')).json();
  ws=new WebSocket(targets.find(t=>t.type==='page').webSocketDebuggerUrl);
  await once(ws,'open');
  const pending=new Map();
  let seq=0;
  ws.onmessage=e=>{
    const row=JSON.parse(e.data), waiter=pending.get(row.id);
    if(waiter){pending.delete(row.id);row.error?waiter.reject(Error(row.error.message)):waiter.resolve(row.result);}
  };
  const call=(method,params)=>new Promise((resolve,reject)=>{
    const id=++seq;pending.set(id,{resolve,reject});ws.send(JSON.stringify({id,method,params}));
  });
  async function value(expression) {
    const result=await call('Runtime.evaluate',{expression,returnByValue:true});
    if(result.exceptionDetails)throw Error(result.exceptionDetails.exception?.description || result.exceptionDetails.text);
    return result.result.value;
  }
  await call('Page.navigate',{url:pathToFileURL(source).href});
  for(let i=0;i<100&&!(await value("!!window.SWERVE_REPLAY && !!document.querySelector('#wall')?.value"));i++)await delay(100);
  const times=await value("window.SWERVE_REPLAY.runs.plus.samples.map(r=>r[0])");
  if(!times?.length||times.some((t,i)=>!Number.isFinite(t)||(i&&Math.abs(t-times[i-1]-.1)>.0001)))throw Error('Expected the 10 Hz published wall clock.');
  await mkdir(path.dirname(path.resolve(output)),{recursive:true});
  encoder=spawn(ffmpeg,['-hide_banner','-loglevel','error','-n','-f','image2pipe','-r','10','-c:v','mjpeg','-i','pipe:0',
    '-c:v','libvpx-vp9','-b:v','0','-crf','28','-pix_fmt','yuv420p',output],{windowsHide:true,stdio:['pipe','ignore','inherit']});
  const finished=once(encoder,'close');
  for(let k=0;k<times.length;k++){
    const bytes=Buffer.from(await value('('+filmFrame.toString()+')('+k+')'),'base64');
    if(!encoder.stdin.write(bytes))await once(encoder.stdin,'drain');
  }
  encoder.stdin.end();
  if((await finished)[0]!==0)throw Error('Film encoding failed.');
  console.log(times.length+' frames, '+times[0]+'–'+times.at(-1)+' s.');
} finally {
  if(encoder?.exitCode===null)encoder.kill();
  ws?.close();chrome.kill();
  await closed;
  await rm(profile,{recursive:true,force:true,maxRetries:4,retryDelay:150});
}
