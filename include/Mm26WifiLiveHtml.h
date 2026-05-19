#pragma once
static const char MM26_WIFI_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>mm26 live</title>
<style>
*{box-sizing:border-box}
body{font-family:system-ui,sans-serif;background:#0d1117;color:#e6edf3;margin:0;padding:10px;font-size:12px}
h1{font-size:1.1rem;margin:0 0 4px}
.sub{color:#8b949e;margin:0 0 8px}
.layout{display:grid;grid-template-columns:1fr 340px;gap:10px}
@media(max-width:960px){.layout{grid-template-columns:1fr}}
canvas{background:#010409;border:2px solid #30363d;border-radius:8px;width:100%}
.side{display:flex;flex-direction:column;gap:8px;max-height:95vh;overflow:auto}
.box{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:8px}
.box h3{margin:0 0 6px;font-size:11px;text-transform:uppercase;color:#8b949e;letter-spacing:.04em}
.btnrow{display:flex;flex-wrap:wrap;gap:4px}
button{padding:5px 10px;border-radius:6px;border:0;background:#238636;color:#fff;cursor:pointer;font-size:11px}
button.sec{background:#21262d;border:1px solid #30363d;color:#e6edf3}
button.warn{background:#9e6a03}
pre,table{font-family:ui-monospace,monospace;font-size:10px;margin:0}
table{width:100%;border-collapse:collapse}
td{padding:2px 4px;border-bottom:1px solid #21262d}
td.k{color:#8b949e;width:42%}
#liveScript,#replayScript{max-height:140px;overflow:auto}
.seg{padding:2px 0;border-bottom:1px solid #21262d}
.seg.active{color:#58a6ff;font-weight:600}
.seg.replay.active{color:#d2a8ff}
.stat{color:#8b949e}
#copyOut{position:fixed;left:-9999px}
.legend{font-size:10px;color:#8b949e}
.legend i{display:inline-block;width:12px;height:3px;margin-right:3px;vertical-align:middle}
</style></head><body>
<h1>mm26 — live + fast-run replay</h1>
<p class="sub">Orange=live robot · Purple ghost=replay sim · White walls=sensed · Config matches firmware ticks</p>
<div class="layout">
<canvas id="cv" width="760" height="520"></canvas>
<div class="side">
<div class="box"><h3>Status</h3><div id="hdr" class="stat">…</div>
<div id="pose" style="color:#58a6ff;margin-top:4px">—</div></div>
<div class="box"><h3>Replay (exact cm / ticks)</h3>
<div class="btnrow">
<button onclick="api('replay/load')">Load NVS</button>
<button onclick="api('replay/plan')">Plan fast run</button>
<button class="warn" onclick="api('replay/play')">Play</button>
<button class="sec" onclick="api('replay/pause')">Pause</button>
<button class="sec" onclick="api('replay/reset')">Reset</button>
<button class="sec" onclick="api('replay/step')">Step</button>
</div>
<p id="replayStat" class="stat">—</p>
<div id="replayScript"></div>
<button class="sec" style="margin-top:4px" onclick="copyScript('replay')">Copy full plan</button>
</div>
<div class="box"><h3>Live script (current move)</h3>
<div id="liveScript"></div>
<button class="sec" onclick="copyScript('live')">Copy live script</button>
</div>
<div class="box"><h3>Motion debug</h3><table id="dbgTbl"></table></div>
<div class="box"><h3>Fast-run geometry</h3><table id="cfgTbl"></table></div>
</div>
</div>
</div>
<textarea id="copyOut"></textarea>
<script>
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
let D={};
async function api(path){await fetch('/api/'+path,{method:'POST'});poll();}
async function poll(){try{D=await(await fetch('/api/live')).json();draw();ui();}catch(e){document.getElementById('hdr').textContent='lost';}}
function fmtScript(list,cls,idx){
  if(!list||!list.length)return '<div class="stat">empty</div>';
  return list.map(s=>`<div class="seg ${cls} ${s.i-1===idx?'active':''}">${s.i}. ${s.ph} ${s.cm}cm / ${s.ticks}tk ${s.dir||''}</div>`).join('');
}
function copyScript(which){
  const list=which==='replay'?(D.replay&&D.replay.path):(D.liveScript);
  if(!list||!list.length){alert('nothing to copy');return;}
  const lines=list.map(s=>`${s.i}. ${s.ph} ${s.cm} cm (${s.ticks} ticks) ${s.dir||''}`);
  const t=document.getElementById('copyOut');
  t.value=lines.join('\n');
  t.select();document.execCommand('copy');
  alert('copied '+lines.length+' lines');
}
function ui(){
  document.getElementById('hdr').textContent=`${D.ip} · ${D.mode} · ${D.state}`;
  const p=D.pose;
  const lp=D.pose,rp=D.replayPose;
  let poseTxt='';
  if(lp)poseTxt+=`LIVE ${lp.xCm}, ${lp.yCm} cm hd ${['N','E','S','W'][lp.heading]} `;
  if(rp)poseTxt+=`| REPLAY ${rp.xCm}, ${rp.yCm} cm hd ${rp.hdDeg&&rp.hdDeg.toFixed?rp.hdDeg.toFixed(0):''}°`;
  poseTxt+=` cell (${(lp||rp||{}).row},${(lp||rp||{}).col})`;
  document.getElementById('pose').textContent=poseTxt||'—';
  document.getElementById('liveScript').innerHTML=fmtScript(D.liveScript,'',D.scriptIdx);
  const rp=D.replay||{};
  document.getElementById('replayStat').textContent=rp.ready?
    `plan ${rp.len} segs · fwd ${rp.totalFwdCm} cm · seg ${rp.seg+1} prog ${(rp.prog*100).toFixed(0)}% ${rp.playing?'▶':''}`:
    (rp.err||'load NVS + plan');
  document.getElementById('replayScript').innerHTML=fmtScript(rp.path,'replay',rp.seg);
  const g=D.cfg||{},db=D.dbg||{};
  document.getElementById('cfgTbl').innerHTML=`
    <tr><td class="k">ticks/cell</td><td>${g.ticksPerCell} (${g.ticksPerMm}/mm)</td></tr>
    <tr><td class="k">start offset</td><td>${g.startOffsetTicks} tk / ${g.startOffsetCm} cm</td></tr>
    <tr><td class="k">pivot approach</td><td>${g.approachCm} cm</td></tr>
    <tr><td class="k">post-pivot</td><td>${g.postPivotCm} cm</td></tr>
    <tr><td class="k">pivot trim</td><td>${g.pivotTrimCm} cm (cell−approach)</td></tr>
    <tr><td class="k">cell</td><td>${g.cellCm} cm</td></tr>`;
  document.getElementById('dbgTbl').innerHTML=db.phase?`
    <tr><td class="k">phase</td><td>${db.phase}</td></tr>
    <tr><td class="k">enc L/R</td><td>${db.encL} / ${db.encR}</td></tr>
    <tr><td class="k">tgt / avg</td><td>${db.tgt} / ${db.avg&&db.avg.toFixed?db.avg.toFixed(0):db.avg}</td></tr>
    <tr><td class="k">pos err</td><td>${db.err&&db.err.toFixed?db.err.toFixed(1):db.err} prog ${db.prog&&db.prog.toFixed?db.prog.toFixed(2):db.prog}</td></tr>
    <tr><td class="k">yaw / tgt</td><td>${db.yaw&&db.yaw.toFixed?db.yaw.toFixed(1):db.yaw}° / ${db.yawTgt&&db.yawTgt.toFixed?db.yawTgt.toFixed(1):db.yawTgt}° gz ${db.gz&&db.gz.toFixed?db.gz.toFixed(2):db.gz}</td></tr>
    <tr><td class="k">IR LF L R RF</td><td>${(db.ir||[]).join(' ')}</td></tr>
    <tr><td class="k">PWM L/R</td><td>${db.pwmL} / ${db.pwmR}</td></tr>
    <tr><td class="k">plan →</td><td>(${db.plan&&db.plan[0]},${db.plan&&db.plan[1]}) ${db.plan&&db.plan[2]} dist ${db.bestDist}</td></tr>
    <tr><td class="k">walls F/L/R</td><td>${(db.walls||[]).join('/')}</td></tr>
    <tr><td class="k">battery</td><td>${db.vbat}V ${db.bat}%</td></tr>`:'<tr><td colspan=2 class="stat">idle — start explore/fast</td></tr>';
}
function draw(){
  const rows=D.rows||6,cols=D.cols||3,cell=D.cellMm||180;
  const pad=32,gw=cols*cell,gh=rows*cell;
  const sc=Math.min((cv.width-pad*2)/gw,(cv.height-pad*2)/gh);
  const ox=pad,oy=cv.height-pad-gh*sc;
  const tx=x=>ox+x*sc, ty=y=>oy+(gh-y)*sc;
  ctx.fillStyle='#010409';ctx.fillRect(0,0,cv.width,cv.height);
  const walls=D.walls||[],vis=D.visited||[],rw=D.replayWalls||walls;
  for(let r=0;r<rows;r++)for(let c=0;c<cols;c++){
    const wx=c*cell,wy=r*cell,i=r*cols+c;
    ctx.fillStyle=vis[i]?'#132013':'#0d1117';
    ctx.fillRect(tx(wx),ty(wy+cell),cell*sc,cell*sc);
    ctx.strokeStyle='#21262d';ctx.lineWidth=1;ctx.strokeRect(tx(wx),ty(wy+cell),cell*sc,cell*sc);
  }
  ctx.lineCap='round';
  const edge=(x1,y1,x2,y2,w,col)=>{ctx.strokeStyle=col;ctx.lineWidth=w;ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();};
  for(let r=0;r<rows;r++)for(let c=0;c<cols;c++){
    const wx=c*cell,wy=r*cell,wl=rw[r*cols+c]||0;
    const bold=vis[r*cols+c]?6:3,col=vis[r*cols+c]?'#f0f6fc':'#6e7681';
    if(wl&1)edge(tx(wx),ty(wy+cell),tx(wx+cell),ty(wy+cell),bold,col);
    if(wl&2)edge(tx(wx+cell),ty(wy+cell),tx(wx+cell),ty(wy),bold,col);
    if(wl&4)edge(tx(wx),ty(wy),tx(wx+cell),ty(wy),bold,col);
    if(wl&8)edge(tx(wx),ty(wy+cell),tx(wx),ty(wy),bold,col);
  }
  edge(tx(0),ty(gh),tx(gw),ty(gh),7,'#8b949e');
  edge(tx(0),ty(0),tx(gw),ty(0),7,'#8b949e');
  edge(tx(0),ty(0),tx(0),ty(gh),7,'#8b949e');
  edge(tx(gw),ty(0),tx(gw),ty(gh),7,'#8b949e');
  const tr=D.trail||[];
  if(tr.length>1){ctx.strokeStyle='#3fb950';ctx.lineWidth=2;ctx.beginPath();
    tr.forEach((p,i)=>{const px=tx(p.x),py=ty(p.y);i?ctx.lineTo(px,py):ctx.moveTo(px,py);});ctx.stroke();}
  const drawBot=(p,col,fill)=>{
    const L=D.lenMm||100,W=D.widMm||85,rear=D.rearMm||40;
    const rad=p.hdDeg*Math.PI/180,cx=tx(p.xMm),cy=ty(p.yMm);
    ctx.save();ctx.translate(cx,cy);ctx.rotate(-rad);
    ctx.fillStyle=fill;ctx.strokeStyle=col;ctx.lineWidth=2;
    ctx.fillRect(-W/2*sc,-rear*sc,L*sc,W*sc);ctx.strokeRect(-W/2*sc,-rear*sc,L*sc,W*sc);
    ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(0,0,3,0,7);ctx.fill();ctx.restore();
  };
  if(D.replayPose)drawBot(D.replayPose,'#a371f7','#a371f788');
  if(D.pose)drawBot(D.pose,'#ffa657','#f0883e99');
  const g=D.goal||[0,2];
  ctx.fillStyle='#d2a8ff';ctx.beginPath();
  ctx.arc(tx((g[1]+0.5)*cell),ty((g[0]+0.5)*cell),6,0,7);ctx.fill();
}
setInterval(poll,120);poll();
</script></body></html>
)rawhtml";
