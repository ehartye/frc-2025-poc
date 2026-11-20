const canvas = document.querySelector('#orbit');
const status = document.querySelector('#orbit-status');
const slider = document.querySelector('#pose');
const position = document.querySelector('#pose-label');
const degrees = Math.PI / 180;

async function readMesh(url, expected) {
  const response = await fetch(url);
  if (!response.ok) throw new Error('Missing mesh');
  const bytes = await response.arrayBuffer();
  const hash = await crypto.subtle.digest('SHA-256', bytes);
  const digest = Array.from(new Uint8Array(hash), n => n.toString(16).padStart(2, '0')).join('');
  if (digest !== expected) throw new Error('Mesh does not match registration');
  const expanded = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'));
  const data = new DataView(await new Response(expanded).arrayBuffer());
  const count = data.getUint32(80, true);
  if (data.byteLength !== 84 + count * 50) throw new Error('Incomplete mesh');
  const vertices = new Float32Array(count * 18);
  for (let i = 0; i < count; i++) {
    const facet = 84 + 50 * i;
    for (let j = 0; j < 3; j++) {
      for (let axis = 0; axis < 3; axis++) {
        vertices[18*i+6*j+axis] = data.getFloat32(facet+12+12*j+4*axis,true);
        vertices[18*i+6*j+3+axis] = data.getFloat32(facet+4*axis,true);
      }
    }
  }
  return vertices;
}

async function start() {
  const gl = canvas.getContext('webgl', {alpha:true,antialias:true});
  if (!gl) throw new Error('WebGL unavailable');
  const register = await (await fetch(canvas.dataset.register || 'climb/orbit.json')).json();
  function shader(kind, source) {
    const handle = gl.createShader(kind);
    gl.shaderSource(handle, source); gl.compileShader(handle);
    if (!gl.getShaderParameter(handle, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(handle));
    return handle;
  }
  const program = gl.createProgram();
  gl.attachShader(program, shader(gl.VERTEX_SHADER, `
    attribute vec3 point, normal; uniform vec3 travel, right, up, eye, origin;
    uniform vec2 scale; uniform float tilt; varying float light;
    vec3 incline(vec3 p) {
      return vec3(cos(tilt)*p.x-sin(tilt)*p.z,p.y,sin(tilt)*p.x+cos(tilt)*p.z);
    }
    void main(){
      vec3 pivot=vec3(300.,0.,1508.064);
      vec3 p=incline(point+travel-pivot)+pivot-origin;
      gl_Position=vec4(dot(p,right)*scale.x,dot(p,up)*scale.y,-dot(p,eye)/4000.,1.);
      vec3 n=incline(normal);
      light=.68+.32*max(dot(n,normalize(vec3(.3,-.5,.9))),0.);
    }`));
  gl.attachShader(program, shader(gl.FRAGMENT_SHADER, `
    precision mediump float; uniform vec3 color; varying float light;
    void main(){gl_FragColor=vec4(color*light,1.);}`));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
  gl.useProgram(program);
  const uniforms = Object.fromEntries(['travel','right','up','eye','origin','scale','tilt','color']
    .map(name=>[name,gl.getUniformLocation(program,name)]));
  const point = gl.getAttribLocation(program,'point'), normal = gl.getAttribLocation(program,'normal');
  gl.enableVertexAttribArray(point); gl.enableVertexAttribArray(normal);
  gl.enable(gl.DEPTH_TEST);
  const batches = [];
  async function add(url, sha, rows) {
    const vertices = await readMesh(url,sha);
    const gpu = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER,gpu);
    gl.bufferData(gl.ARRAY_BUFFER,vertices,gl.STATIC_DRAW);
    let start = 0;
    for (const [group,hex,count,overlay] of rows) {
      const color = hex.slice(1).match(/../g).map(v=>parseInt(v,16)/255);
      if (group !== 'Demonstration CORAL') batches.push({gpu,group,color,start,count:3*count,overlay});
      start += 3*count;
    }
    if (start !== vertices.length/6) throw new Error('Mesh group count mismatch');
  }
  await add(canvas.dataset.base,register.baseSha256,
    register.baseBatches.map(([role,color,group,count])=>[group,color,count,role==='bumper-lettering']));
  if (register.kitSha256) await add('climb/climb-kit.stl.gz',register.kitSha256,register.kitBatches);
  let yaw = -1.02, pitch = .27, zoom = 1, pointer;
  function draw() {
    const ratio = Math.min(devicePixelRatio || 1, 2);
    const w = Math.round(canvas.clientWidth*ratio), h = Math.round(canvas.clientHeight*ratio);
    if (canvas.width!==w || canvas.height!==h) {canvas.width=w;canvas.height=h;}
    gl.viewport(0,0,w,h); gl.clearColor(0,0,0,0);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
    const [approach,spread,head,clamp,lift,tilt]=register.poses[+slider.value];
    const sy=Math.sin(yaw),cy=Math.cos(yaw),sp=Math.sin(pitch),cp=Math.cos(pitch);
    gl.uniform3f(uniforms.right,-sy,cy,0);
    gl.uniform3f(uniforms.up,-cy*sp,-sy*sp,cp);
    gl.uniform3f(uniforms.eye,cy*cp,sy*cp,sp);
    gl.uniform3f(uniforms.origin,approach*.5,0,675);
    const span=Math.max(810,815*h/w)/zoom;
    gl.uniform2f(uniforms.scale,h/w/span,1/span);
    gl.uniform1f(uniforms.tilt,tilt*degrees);
    let lastBuffer;
    for(const b of batches) {
      if (lastBuffer !== b.gpu) {
        gl.bindBuffer(gl.ARRAY_BUFFER,b.gpu); gl.vertexAttribPointer(point,3,gl.FLOAT,false,24,0);
        gl.vertexAttribPointer(normal,3,gl.FLOAT,false,24,12);lastBuffer=b.gpu;
      }
      const moving=['Cage crosshead','Left cage shoe','Right cage shoe','Lower plate clamp'].includes(b.group);
      const cage=b.group==='Illustrative cage';
      gl.uniform3f(uniforms.travel,cage?0:approach,
        b.group==='Left cage shoe'?spread:b.group==='Right cage shoe'?-spread:0,
        cage?0:lift+(moving?head:0)+(b.group==='Lower plate clamp'?clamp:0));
      gl.uniform3fv(uniforms.color,b.color);
      if (b.overlay) {gl.enable(gl.POLYGON_OFFSET_FILL);gl.polygonOffset(-1,-1);}
      gl.drawArrays(gl.TRIANGLES,b.start,b.count);
      if (b.overlay) gl.disable(gl.POLYGON_OFFSET_FILL);
    }
    position.textContent=canvas.dataset.inspection ? Math.round(head)+' mm carriage travel · '+Math.round(spread)+' mm shoe travel' : (130+slider.value/8).toFixed(2)+' s · lift '+Math.round(lift)+' mm';
    canvas.dataset.pose=slider.value;
  }
  function rotate(dx,dy) {
    yaw-=dx*.011; pitch=Math.max(-1.35,Math.min(1.4,pitch+dy*.008));draw();
  }
  canvas.addEventListener('pointerdown',e=>{pointer=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
  canvas.addEventListener('pointermove',e=>{if(pointer){rotate(e.clientX-pointer[0],e.clientY-pointer[1]);pointer=[e.clientX,e.clientY];}});
  canvas.addEventListener('lostpointercapture',()=>pointer=null);
  canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.65,Math.min(2.6,zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
  canvas.addEventListener('keydown',e=>{
    const delta={ArrowLeft:[-8,0],ArrowRight:[8,0],ArrowUp:[0,8],ArrowDown:[0,-8]}[e.key];
    if(delta){e.preventDefault();rotate(...delta);}
  });
  slider.addEventListener('input',draw);
  for(const button of document.querySelectorAll('[data-pose]'))button.onclick=()=>{slider.value=button.dataset.pose;draw();};
  document.querySelector('#reset-orbit').onclick=()=>{yaw=-1.02;pitch=.27;zoom=1;draw();};
  slider.disabled=false;
  new ResizeObserver(draw).observe(canvas);
  draw();status.textContent='Drag or use arrow keys to rotate. Scroll to zoom.';
}
start().catch(error=>{status.textContent=canvas.dataset.inspection ? 'The 3D view is unavailable. Download the editable assembly below.' : 'The 3D view is unavailable. Use the climb film below.';console.error(error);});
