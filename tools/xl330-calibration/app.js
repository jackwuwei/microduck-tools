'use strict';
const i18n = window.MicroduckI18n;
const $ = id => document.getElementById(id);
const home = window.MicroduckPoseView?.HOME_JOINTS || Array(15).fill(0);
// XL330 housing instances in duck-web-mouth-v2.bin (mesh 3). Motor housings
// can be attached to either side of a joint; moving-body indices are not motor IDs.
const servoParts = {20:3,21:8,22:17,23:14,24:19,30:25,31:26,32:31,33:39,34:36,10:4,11:48,12:57,13:53,14:58};
let view,modelError=null;
try {
  if(typeof window.MicroduckPoseView!=='function')throw new Error('3D 渲染器 duck-view.js 未加载；请重启工作台后刷新页面');
  view=new window.MicroduckPoseView($('duck'),{maxFps:30});
}
catch(error){modelError=error.message;view={setState(){},setSelectedPart(){},load:async()=>{}};}
let state=null, selected=20, reference='zero', busy=false, lastSeen=0, revision=0;
const labels={drive:'Drive Mode (10)',offset:'Homing Offset (20)',minimum:'Min Position (52)',maximum:'Max Position (48)'};
function pose(){
  if(!state)return;
  let joints=Array(15).fill(0);
  const index=state.joints.findIndex(j=>j.id===selected);
  view.setSelectedPart(servoParts[selected]);
  if(reference==='home')joints=home.slice();
  if(reference==='positive'||reference==='negative')joints[index]=(reference==='positive'?1:-1)*Math.PI/18;
  if(reference==='measured')joints=state.joints.map(j=>j.telemetry?(j.telemetry.position-2048)*Math.PI/2048:0);
  view.setState(joints);
  if(modelError){$('reference-note').textContent=`3D 不可用：${modelError}；请先修复模型显示再核对实物。`;i18n.apply();return;}
  $('reference-note').textContent=reference==='measured'
    ?(state.connected?'当前编码器：使用舵机已保存的零位与方向；尚未应用采集草稿。躯干按水平显示。':'当前编码器不可用；模型暂按零位示意。')
    :'模型参考：仅改变屏幕姿态，不控制实物；采集 0° 时请使用“零位 0°”。';
  document.querySelectorAll('[data-ref]').forEach(b=>b.classList.toggle('active',b.dataset.ref===reference));
  i18n.apply();
}
function controls(){
  const connected=state?.connected===true;
  document.querySelectorAll('.joint').forEach(b=>b.disabled=busy);
  $('connect').disabled=busy||connected||!$('ready').checked;
  $('disconnect').disabled=busy||!connected;
  for(const id of ['normal','reverse','restore','restore-file'])$(id).disabled=busy||!connected;
  document.querySelectorAll('[data-capture]').forEach(b=>b.disabled=busy||!connected);
  const joint=state?.joints.find(j=>j.id===selected);
  $('preview').disabled=busy||!connected||Object.keys(joint?.marks||{}).length!==3;
  $('apply').disabled=busy||!connected||state?.preview?.motor!==selected||!$('reviewed').checked;
  $('position').disabled=busy||!connected;
}
function paint(next){
  state=next;lastSeen=performance.now();
  $('mode').textContent=state.demo?'演示模式 · 无硬件连接':`真机模式 · ${state.connection||'串口'} · EEPROM 写入`;
  $('status').textContent=state.connected?'已连接 · 全部松力':'未连接';
  $('demo').hidden=!state.demo;
  $('backup').textContent=state.backup?`连接前备份：${state.backup}`:'';
  const joint=state.joints.find(j=>j.id===selected),row=joint.telemetry;
  $('selected').textContent=`ID ${selected} · ${joint.label}`;
  $('metrics').textContent=row?`${((row.position-2048)*360/4096).toFixed(2)}°  /  ${row.position} pulse`:'等待连接';
  $('config').textContent=row?`方向 ${row.drive&1?'反向':'正向'} · 偏移 ${row.offset} · 限位 ${row.minimum}…${row.maximum} · ${row.voltage/10} V · ${row.temperature}°C · 硬件状态 0x${row.error.toString(16).padStart(2,'0')}${row.error===1?'（输入电压报警）':''}`:'';
  $('marks').textContent=Object.entries(joint.marks).map(([key,value])=>`${{zero:'零位',minimum:'负限位',maximum:'正限位'}[key]}：${value} pulse`).join('\n')||'尚未采集';
  $('saved').textContent=joint.complete?'✓ 本次已写入并回读校验':'';
  $('joints').replaceChildren(...state.joints.map(j=>{
    const b=document.createElement('button');b.className=`joint ${j.id===selected?'active':''}`;
    b.textContent=`${j.complete?'✓ ':''}${j.id} · ${j.label}`;
    const t=j.telemetry,s=document.createElement('small');
    const marks=['zero','minimum','maximum'].map(k=>`${{zero:'零',minimum:'负',maximum:'正'}[k]}${k in j.marks?'✓':'·'}`).join(' ');
    s.textContent=t?`${t.position-2048>=0?'+':''}${t.position-2048} pulse · 偏移 ${t.offset}${t.drive&1?' · 反向':''} · ${marks}${j.complete?' · 已写入':''}`:marks;
    b.append(s);
    b.disabled=busy;
    b.onclick=()=>{selected=j.id;$('reviewed').checked=false;paint(state);};return b;
  }));
  $('changes').replaceChildren();
  const preview=state.preview;
  if(preview?.motor===selected){
    const table=document.createElement('table');
    const head=table.insertRow();for(const text of ['寄存器','当前','待写入']){const th=document.createElement('th');th.textContent=text;head.append(th);}
    for(const key of Object.keys(labels)){
      const row=table.insertRow();for(const value of [labels[key],preview.before[key],preview.after[key]])row.insertCell().textContent=value;
    }
    $('changes').append(table);
  }
  controls();pose();
}
async function api(path,body){
  const response=await fetch(`/api/${path}`,body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const result=await response.json();
  if(!response.ok)throw Object.assign(new Error(result.error||'请求失败'),{disconnected:result.disconnected!==false});
  return result;
}
async function action(path,extra={}){
  if(busy)return;
  revision++;
  busy=true;controls();$('error').textContent='';
  if(path!=='apply')$('reviewed').checked=false;
  try{
    paint(await api(path,{motor:selected,...extra}));
    const files=await api('backups');
    const previous=$('backups').value;
    $('backups').replaceChildren(...files.map(file=>{const o=document.createElement('option');o.value=file;o.textContent=file;return o;}));
    if(files.includes(previous))$('backups').value=previous;
  }catch(error){
    if(!error.disconnected){$('error').textContent=`${error.message}。会话仍保持连接，已采集的数据未丢失。`;return;}
    $('error').textContent=`${error.message}。会话已停止；请检查后重新连接。`;
    if(state)paint({...state,connected:false,preview:null,joints:state.joints.map(j=>({...j,telemetry:null}))});
  }finally{busy=false;controls();i18n.apply();}
}
$('ready').onchange=controls;$('reviewed').onchange=controls;
$('connect').onclick=()=>action('connect');$('disconnect').onclick=()=>action('disconnect');
$('normal').onclick=()=>action('direction',{reverse:false});$('reverse').onclick=()=>action('direction',{reverse:true});
$('preview').onclick=()=>action('preview');$('apply').onclick=()=>action('apply',{token:state.preview.token});
$('restore').onclick=()=>action('restore');$('restore-file').onclick=()=>action('restore-file',{file:$('backups').value});
document.querySelectorAll('[data-capture]').forEach(b=>b.onclick=()=>action('capture',{kind:b.dataset.capture}));
document.querySelectorAll('[data-ref]').forEach(b=>b.onclick=()=>{reference=b.dataset.ref;pose();});
$('position').onchange=()=>action('demo-position',{position:Number($('position').value)});
async function poll(){
  if(!busy){
    const started=revision;
    try{const next=await api('state');if(started===revision&&!busy)paint(next);}
    catch(error){
      if(started===revision&&!busy){
        $('error').textContent=error.message;
        if(state)paint({...state,connected:false,preview:null,joints:state.joints.map(j=>({...j,telemetry:null}))});
      }
    }
  }
  i18n.apply();
  setTimeout(poll,1000);
}
view.load('/duck.bin').then(pose).catch(e=>{modelError=e.message;pose();});
setInterval(()=>{if(state?.connected&&performance.now()-lastSeen>3000){$('status').textContent='反馈已过期';$('metrics').textContent='读数过期';$('apply').disabled=true;i18n.apply();}},500);
$('language').onchange=event=>i18n.setLanguage(event.target.value);
i18n.apply();
poll();
