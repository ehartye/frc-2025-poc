const controls = document.querySelector('#mechanism');
const timeControl = document.querySelector('#stage-time');
const stageControl = document.querySelector('#stage');
const timeLabel = document.querySelector('#stage-readout');
const stageNote = document.querySelector('#stage-note');
const programStages = layout.motion.tracks;
const bodyColor = new Map(batches.map((batch,i)=>[ranges[i][2],batch.slice(1)]));
const motionBatches = layout.motion.ranges.map(([role,group,triangles])=>[triangles,...bodyColor.get(role),group]);
const pipeBaseMm = [75,0,557.1145];
const uniform = name => gl.getUniformLocation(program,name);
const shifted = uniform('partShift'), pipeAxis = uniform('pipeAxis'), pipeBase = uniform('pipeBase');
const pipeMode = uniform('isPipe');
const neutral = [0,0,...pipeBaseMm,0,1,0,0];
let active = neutral;
function selectTime() {
  const track = programStages[stageControl.value];
  const index = Math.max(0, Math.min(track.rows.length-1, Number(timeControl.value)));
  active = track.rows[index];
  timeLabel.textContent = active[0].toFixed(2)+' s · rack '+Math.round(active[1])+' mm · closure '+Math.round(active[8])+' mm';
  stageNote.textContent = track.note;
  draw();
}
function selectStage() {
  const track = programStages[stageControl.value];
  timeControl.max = track.rows.length-1;
  timeControl.value = 0;
  selectTime();
}
function moveGroup(group) {
  const [t,rack,x,y,z,ax,ay,az,close] = active;
  const pipe = group==='Demonstration CORAL';
  const m = group==='Middle slide' ? .5 : ['Outer slide','Left capture yoke','Right capture yoke'].includes(group) ? 1 : 0;
  const offset = pipe ? [x-pipeBaseMm[0],y-pipeBaseMm[1],z-pipeBaseMm[2]]
                      : [m*rack,group==='Left capture yoke' ? -close : group==='Right capture yoke' ? close : 0,0];
  gl.uniform1f(pipeMode,pipe ? 1 : 0);
  gl.uniform3fv(shifted,offset.map(v=>v/radius));
  gl.uniform3fv(pipeBase,pipeBaseMm.map((v,i)=>(v-center[i])/radius));
  gl.uniform3fv(pipeAxis,[ax,ay,az]);
}
controls.hidden = false;
stageControl.addEventListener('change',selectStage);
timeControl.addEventListener('input',selectTime);
document.querySelector('#previous-pose').onclick = ()=>{timeControl.stepDown();selectTime();};
document.querySelector('#next-pose').onclick = ()=>{timeControl.stepUp();selectTime();};
