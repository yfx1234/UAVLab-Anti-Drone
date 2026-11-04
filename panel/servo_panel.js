'use strict';

const $ = id => document.getElementById(id);
const model = {connected:false, configured:false, state:null, lease:null, target:1500,
  min:1000, max:2000, degrees:180, slot:2, bank:'MAIN', channel:5,
  polling:false, actionPending:false, actionEpoch:0, queuedTarget:false, targetSource:'pulse', samples:[], events:[], lastError:null};
const preferenceKey = 'servoBench.v2';
let sliderTimer;

async function api(payload, timeout=12000) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeout);
  try {
    const response = await fetch('/api', {method:'POST', headers:{'Content-Type':'application/json'},
      body:JSON.stringify(payload), signal:controller.signal});
    const result = await response.json();
    if (!response.ok || result.ok === false) throw new Error(result.error || result.result_name || '操作未完成');
    return result;
  } catch (error) {
    if (error.name === 'AbortError') throw new Error('请求超时，请检查设备连接');
    throw error;
  } finally { clearTimeout(timer); }
}

function logEvent(text, error=false) {
  model.events.unshift({text, error, time:new Date().toLocaleTimeString('zh-CN',{hour12:false})});
  model.events = model.events.slice(0,6);
  renderLog();
}
function renderLog() {
  const list = $('activityList'); list.replaceChildren();
  if (!model.events.length) {
    const row = document.createElement('li'); row.className='empty-log';
    row.textContent='动作与异常会记录在这里。'; list.append(row); return;
  }
  model.events.forEach(event => {
    const row=document.createElement('li'), time=document.createElement('time'), label=document.createElement('span');
    time.textContent=event.time; label.textContent=event.text;
    if(event.error) row.className='error'; row.append(time,label); list.append(row);
  });
}
function message(text,error=false) {
  $('actionMessage').textContent=text;
  $('actionMessage').classList.toggle('error',error);
}
function showError(error) { message(error.message,true); logEvent(error.message,true); }
function numeric(value,min,max,label) {
  const number=Number(value);
  if(value==='' || !Number.isFinite(number) || number<min || number>max)
    throw new Error(`${label}需在 ${min}–${max} 之间`);
  return number;
}
function pulseToAngle(pulse) { return (pulse-model.min)/(model.max-model.min)*model.degrees; }
function angleToPulse(angle) { return model.min+angle/model.degrees*(model.max-model.min); }
function targetNorm() { return (model.target-(model.min+model.max)/2)/((model.max-model.min)/2); }
function behavior() { return document.querySelector('input[name="behavior"]:checked').value; }

function setTarget(pulse) {
  model.target=Math.round(Math.max(model.min,Math.min(model.max,pulse)));
  $('positionSlider').value=model.target; $('pulseInput').value=model.target;
  $('angleInput').value=pulseToAngle(model.target).toFixed(1);
  $('angleInput').style.width=($('angleInput').value.length-0.4)+'ch';
}
function updateScale() {
  $('positionSlider').min=$('pulseInput').min=model.min;
  $('positionSlider').max=$('pulseInput').max=model.max;
  $('angleInput').max=model.degrees;
  $('rangeMin').textContent=`0° / ${model.min} µs`;
  $('rangeMax').textContent=`${model.degrees}° / ${model.max} µs`;
  document.querySelectorAll('.point-angle').forEach(input=>input.max=model.degrees);
  setTarget(model.target);
}
function updateBehavior() {
  const pulse=behavior()==='pulse'; $('durationField').hidden=!pulse;
  $('behaviorHelp').textContent=pulse?`保持 ${$('holdSeconds').value || '1'} 秒后，恢复默认输出。`:
    '持续保持，直到回中或停止；页面断开时自动结束。';
  savePreferences();
}

function updateAvailability() {
  const state=model.state;
  const ready=model.connected && model.configured && state?.alive && state.armed===false;
  const sequence=state?.mode==='sequence';
  document.querySelectorAll('[data-motion]').forEach(button=>button.disabled=!ready || model.actionPending || sequence);
  $('stopBtn').disabled=!model.connected;
  $('inspectBtn').disabled=!model.connected || (state?.mode && state.mode!=='idle');
  $('outputBank').disabled=$('outputChannel').disabled=!!(state?.mode && state.mode!=='idle');
  $('liveFollow').disabled=!ready || sequence;
  document.querySelectorAll('#sequenceBody input,#sequenceBody button,#addPointBtn,#clearSequenceBtn,#smallSweepBtn,#importBtn,#repeatCount,#sequenceAfter')
    .forEach(control=>control.disabled=sequence);
  const firstUp=$('sequenceBody').querySelector('.move-up');if(firstUp)firstUp.disabled=true;
}
function setConnected(connected) {
  model.connected=connected;
  $('headerStatus').className='connection-status'+(connected?' connected':'');
  $('headerStatus').replaceChildren();
  const dot=document.createElement('span'); dot.className='status-dot';
  $('headerStatus').append(dot,connected?'设备已连接':'连接设备');
  $('deviceState').textContent=connected?'已连接':'未连接';
  $('deviceState').classList.toggle('online',connected);
  $('connectBtn').textContent=connected?'断开连接':'连接设备';
  $('boardHost').disabled=$('boardUser').disabled=connected;
  if(!connected) {
    model.lease=null; model.configured=false; model.state=null;
    model.samples=[];
    $('actualChartLine').setAttribute('d','');$('targetChartLine').setAttribute('d','');
    $('chartMax').textContent=$('chartMin').textContent='';$('chartEmpty').hidden=false;
    $('actualOutput').textContent='—'; $('armedState').textContent='尚未读取';
    $('compactOutput').textContent='—';$('compactFreshness').textContent='等待反馈';
    $('telemetryState').textContent='等待数据'; $('telemetryState').classList.remove('online');
    $('motionStatus').textContent='待机'; $('motionStatus').classList.remove('active');
    $('footerStatus').textContent='未连接 · 不发送动作';
  }
  updateAvailability();
}

async function inspectOutput() {
  model.configured=false; updateAvailability();
  const bank=$('outputBank').value, channel=Number($('outputChannel').value);
  const state=await api({op:'inspect',bank,channel});
  const prefix=`PWM_${bank}_`, func=state.params[`${prefix}FUNC${channel}`];
  const min=state.params[`${prefix}MIN${channel}`], max=state.params[`${prefix}MAX${channel}`];
  if(!Number.isFinite(func) || func<301 || func>306 || !Number.isFinite(min) || !Number.isFinite(max) || max<=min) {
    $('slotMapping').textContent=Number.isFinite(func)?`功能 ${func} · 暂不可控`:'未读到输出配置';
    throw new Error('该物理通道尚未映射为舵机控制槽位，请在飞控中核对输出配置');
  }
  model.bank=bank; model.channel=channel; model.slot=func-300; model.min=min; model.max=max;
  model.configured=true;
  $('physicalOutput').textContent=`${bank==='MAIN'?'I/O':'FMU'} · ${channel} 号口`;
  $('slotMapping').textContent=`槽位 ${model.slot} · 功能 ${func}`;
  $('configMin').value=min; $('configMax').value=max;
  updateScale(); renderState(state); savePreferences();
}

async function connect(event) {
  event?.preventDefault();
  const button=$('connectBtn'); button.disabled=true; button.classList.add('is-busy');
  try {
    if(model.connected) {
      await api({op:'disconnect'}); setConnected(false); logEvent('设备已断开，测试已结束');
      $('connectMessage').textContent='已断开连接。'; return;
    }
    button.textContent='连接中…'; $('connectMessage').textContent='正在连接设备并读取输出配置…';
    await api({op:'connect',host:$('boardHost').value.trim(),user:$('boardUser').value.trim(),pass:$('boardPassword').value},20000);
    $('boardPassword').value=''; setConnected(true); savePreferences();
    try { await inspectOutput(); $('connectMessage').textContent='连接正常，输出配置已核对。'; }
    catch(error) { $('connectMessage').textContent=error.message; showError(error); }
    logEvent('设备连接成功');message('设备已就绪，选择目标与动作方式后应用位置。'); await pollState();
  } catch(error) { setConnected(false); $('connectMessage').textContent=error.message; showError(error); }
  finally {button.disabled=false; button.classList.remove('is-busy'); if(!model.connected)button.textContent='连接设备';}
}

function renderState(state) {
  model.state=state;
  $('armedState').textContent=state.armed===false?'未解锁':state.armed===true?'已解锁 · 台架测试不可用':'尚未读取';
  $('armedState').classList.toggle('error-text',state.armed===true);
  const label={idle:'待机',pulse:`短时测试 · ${state.remaining_s??0} 秒`,hold:'保持位置',sequence:`序列 · ${state.seq_index}/${state.seq_total}`};
  $('motionStatus').textContent=label[state.mode] || '待机';
  $('motionStatus').classList.toggle('active',state.mode && state.mode!=='idle');
  if(state.mode==='idle') model.lease=null;
  const outputs=state.outputs || {}, port=model.bank==='MAIN'?'0':'1';
  const value=outputs[port]?.[String(model.channel)];
  const fresh=state.alive && Number.isFinite(state.output_age_s) && state.output_age_s<3 && Number.isFinite(value) && value>0;
  $('actualOutput').textContent=fresh?Math.round(value):'—';
  $('compactOutput').textContent=fresh?Math.round(value):'—';
  $('compactFreshness').textContent=fresh?'实时更新':'等待反馈';
  $('telemetryState').textContent=fresh?'实时更新':state.alive?'暂无该口反馈':'遥测已中断';
  $('telemetryState').classList.toggle('online',fresh);
  if(fresh) {
    const now=Date.now(); model.samples.push({t:now,value,target:model.target});
    model.samples=model.samples.filter(sample=>sample.t>now-30000).slice(-80); renderChart();
  }
  const progress=state.mode==='sequence' || state.seq_total>0;
  $('sequenceProgress').hidden=!progress;
  $('sequenceProgressText').textContent=state.mode==='sequence'?'正在执行':state.mode==='hold'?'序列已完成，保持末点':'序列已结束';
  $('sequenceProgressCount').textContent=`${state.seq_index || 0} / ${state.seq_total || 0}`;
  $('sequenceProgressBar').max=state.seq_total || 1; $('sequenceProgressBar').value=state.seq_index || 0;
  [...$('sequenceBody').rows].forEach((row,index)=>row.classList.toggle('current',state.mode==='sequence' && index===(state.seq_index-1)%Math.max(1,$('sequenceBody').rows.length)));
  $('footerStatus').textContent=state.alive?`${model.bank}${model.channel} · ${state.armed===false?'未解锁':'已解锁'} · ${state.mode==='idle'?'待机':'动作中'}`:'已连接板子 · 等待飞控遥测';
  if(state.motion_error && state.motion_error!==model.lastError) {
    model.lastError=state.motion_error; message(state.motion_error,true); logEvent(state.motion_error,true);
  }
  updateAvailability();
}
function renderChart() {
  const samples=model.samples;
  if(!samples.length)return;
  $('chartEmpty').hidden=true;
  const values=samples.flatMap(sample=>[sample.value,sample.target]);
  const low=Math.min(...values)-25, high=Math.max(...values)+25;
  const first=Math.max(samples[0].t,samples.at(-1).t-30000), span=Math.max(1000,samples.at(-1).t-first);
  const path=key=>samples.map((sample,index)=>`${index?'L':'M'}${((sample.t-first)/span*320).toFixed(1)},${(100-(sample[key]-low)/(high-low)*84).toFixed(1)}`).join(' ');
  $('actualChartLine').setAttribute('d',path('value')); $('targetChartLine').setAttribute('d',path('target'));
  $('chartMax').textContent=Math.round(high);$('chartMin').textContent=Math.round(low);
  $('chartPeriod').textContent=`最近 ${Math.max(1,Math.round(span/1000))} 秒`;
}
async function pollState() {
  if(!model.connected || model.polling)return;
  model.polling=true;
  const lease=model.lease;
  try { const state=await api({op:'state',lease_id:lease || undefined},5000);if(lease===model.lease)renderState(state); }
  catch(error) {setConnected(false); $('connectMessage').textContent='连接中断，测试将自动结束。请重新连接。'; showError(error);}
  finally {model.polling=false;}
}

async function applyPosition() {
  if(model.actionPending){model.queuedTarget=true;return;}
  const epoch=++model.actionEpoch;
  try {
    if(model.targetSource==='angle')setTarget(angleToPulse(numeric($('angleInput').value,0,model.degrees,'目标角度')));
    else setTarget(numeric($('pulseInput').value,model.min,model.max,'目标脉宽'));
    if(!model.connected || !model.configured)throw new Error('请先连接设备并核对输出配置');
    const sentMode=behavior(), sentPulse=model.target;
    const duration=sentMode==='pulse'?numeric($('holdSeconds').value,0.2,30,'保持时长'):1;
    model.actionPending=true; updateAvailability();
    const result=await api({op:'move',slot:model.slot,norm:targetNorm(),mode:sentMode,hold_s:duration});
    if(epoch!==model.actionEpoch)return;
    model.lease=result.lease_id; model.lastError=null;
    const suffix=sentMode==='hold'?'持续保持':`${duration} 秒后恢复默认输出`;
    message(`上次应用：${sentPulse} µs，${suffix}。`);
    logEvent(`目标 ${sentPulse} µs · ${sentMode==='hold'?'保持':duration+' 秒'}`);
    await pollState();
  } catch(error) {showError(error);}
  finally {model.actionPending=false; updateAvailability();if(model.queuedTarget){model.queuedTarget=false;queueMicrotask(applyPosition);}}
}
async function stopMotion() {
  clearTimeout(sliderTimer);
  model.queuedTarget=false;++model.actionEpoch;
  if(!model.connected)return;
  try {
    model.lease=null; await api({op:'stop'}); message('已停止，恢复默认输出。');
    logEvent('停止动作，恢复默认输出'); await pollState();
  } catch(error) {showError(error);}
}

function pointValues() {
  return [...$('sequenceBody').rows].map(row=>({
    angle:numeric(row.querySelector('.point-angle').value,0,model.degrees,'动作点角度'),
    duration_ms:numeric(row.querySelector('.point-duration').value,0,30000,'转动用时'),
    hold_ms:numeric(row.querySelector('.point-hold').value,0,30000,'停留时间')}));
}
function addPoint(point={angle:model.degrees/2,duration_ms:600,hold_ms:500}) {
  if($('sequenceBody').rows.length>=200)throw new Error('最多添加 200 个动作点');
  const fragment=$('sequenceRow').content.cloneNode(true), row=fragment.querySelector('tr');
  row.querySelector('.point-angle').value=point.angle; row.querySelector('.point-angle').max=model.degrees;
  row.querySelector('.point-duration').value=point.duration_ms; row.querySelector('.point-hold').value=point.hold_ms;
  row.querySelector('.remove-point').onclick=()=>{row.remove(); updateSequence(); savePreferences();};
  row.querySelector('.move-up').onclick=()=>{if(row.previousElementSibling)row.before(row.previousElementSibling); updateSequence(); savePreferences();};
  row.onchange=()=>{updateSequence();savePreferences();}; $('sequenceBody').append(fragment); updateSequence();
}
function setPoints(points) { $('sequenceBody').replaceChildren();points.forEach(point=>addPoint(point));updateSequence(); }
function updateSequence() {
  const rows=[...$('sequenceBody').rows];
  rows.forEach((row,index)=>{row.cells[0].textContent=index+1;row.querySelector('.move-up').disabled=index===0;});
  $('sequenceSummary').textContent=`${rows.length} 个动作点`;
  try {
    const points=pointValues(), repeat=numeric($('repeatCount').value,1,20,'重复次数');
    const seconds=points.reduce((sum,point)=>sum+point.duration_ms+point.hold_ms,0)*repeat/1000;
    $('sequenceTime').textContent=points.length?`预计 ${seconds.toFixed(1)} 秒`:'添加动作点以开始';
  } catch { $('sequenceTime').textContent='请补全动作点'; }
}
async function runSequence() {
  try {
    if(!model.connected || !model.configured)throw new Error('请先连接设备并核对输出配置');
    const points=pointValues(), repeat=numeric($('repeatCount').value,1,20,'重复次数');
    if(!Number.isInteger(repeat) || !points.length || points.length*repeat>200)throw new Error('重复后的序列需包含 1–200 个动作点');
    const command=Array.from({length:repeat},()=>points).flat().map(point=>({slot:model.slot,
      norm:(angleToPulse(point.angle)-(model.min+model.max)/2)/((model.max-model.min)/2),
      interp_ms:point.duration_ms,hold_ms:point.hold_ms}));
    model.actionPending=true;updateAvailability();
    const result=await api({op:'seq',points:command,after:$('sequenceAfter').value});
    model.lease=result.lease_id;model.lastError=null;
    message(`序列已开始，共 ${command.length} 个动作点。`);logEvent(`开始序列 · ${points.length} 点 × ${repeat} 次`);
    await pollState();
  } catch(error) {showError(error);}
  finally {model.actionPending=false;updateAvailability();}
}
function exportSequence() {
  try {
    const data={version:1,points:pointValues(),repeat:Number($('repeatCount').value),after:$('sequenceAfter').value};
    const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));
    const link=document.createElement('a');link.href=url;link.download='servo-sequence.json';link.click();
    setTimeout(()=>URL.revokeObjectURL(url),1000);logEvent('已导出动作序列');
  } catch(error) {showError(error);}
}
async function importSequence(file) {
  try {
    if(!file)return;
    if(file.size>131072)throw new Error('序列文件过大');
    const data=JSON.parse(await file.text());
    if(data.version!==1 || !Array.isArray(data.points) || data.points.length>200)throw new Error('不是支持的动作序列文件');
    if(!data.points.every(point=>point && typeof point==='object' &&
      [point.angle,point.duration_ms,point.hold_ms].every(value=>typeof value==='number' && Number.isFinite(value))))
      throw new Error('动作点字段必须是有效数字');
    const points=data.points.map(point=>({angle:numeric(point.angle,0,model.degrees,'动作点角度'),
      duration_ms:numeric(point.duration_ms,0,30000,'转动用时'),hold_ms:numeric(point.hold_ms,0,30000,'停留时间')}));
    const repeat=numeric(data.repeat??1,1,20,'重复次数');
    if(!Number.isInteger(repeat) || repeat*points.length>200 || !['center','hold'].includes(data.after))throw new Error('重复次数或结束方式无效');
    setPoints(points);$('repeatCount').value=repeat;$('sequenceAfter').value=data.after;updateSequence();savePreferences();
    logEvent(`已导入 ${points.length} 个动作点`);
  } catch(error) {showError(error);}
  finally {$('importFile').value='';}
}

function savePreferences() {
  try {
    localStorage.setItem(preferenceKey,JSON.stringify({host:$('boardHost').value,user:$('boardUser').value,
      bank:$('outputBank').value,channel:$('outputChannel').value,degrees:model.degrees,behavior:behavior(),
      hold_s:Number($('holdSeconds').value)||1,repeat:Number($('repeatCount').value)||1,
      after:$('sequenceAfter').value,points:pointValues()}));
  } catch { /* Invalid in-progress edits are not persisted. */ }
}
function restorePreferences() {
  let prefs={};try {prefs=JSON.parse(localStorage.getItem(preferenceKey)||'{}');}catch{}
  if(typeof prefs.host==='string')$('boardHost').value=prefs.host;
  if(typeof prefs.user==='string')$('boardUser').value=prefs.user;
  if(['MAIN','AUX'].includes(prefs.bank))$('outputBank').value=prefs.bank;
  if(Number(prefs.channel)>=1 && Number(prefs.channel)<=8)$('outputChannel').value=prefs.channel;
  if(Number.isFinite(prefs.degrees) && prefs.degrees>=1 && prefs.degrees<=360)model.degrees=prefs.degrees;
  $('angleRange').value=model.degrees;
  if(['pulse','hold'].includes(prefs.behavior))document.querySelector(`input[name="behavior"][value="${prefs.behavior}"]`).checked=true;
  if(Number.isFinite(prefs.hold_s) && prefs.hold_s>=0.2 && prefs.hold_s<=30)$('holdSeconds').value=prefs.hold_s;
  if(Number.isInteger(prefs.repeat) && prefs.repeat>=1 && prefs.repeat<=20)$('repeatCount').value=prefs.repeat;
  if(['center','hold'].includes(prefs.after))$('sequenceAfter').value=prefs.after;
  const valid=Array.isArray(prefs.points) && prefs.points.length<=200 && prefs.points.every(point=>Number.isFinite(point.angle) && point.angle>=0 && point.angle<=model.degrees && Number.isFinite(point.duration_ms) && point.duration_ms>=0 && point.duration_ms<=30000 && Number.isFinite(point.hold_ms) && point.hold_ms>=0 && point.hold_ms<=30000);
  const middle=model.degrees/2;
  setPoints(valid?prefs.points:[{angle:Math.max(0,middle-5),duration_ms:600,hold_ms:500},
    {angle:Math.min(model.degrees,middle+5),duration_ms:600,hold_ms:500},{angle:middle,duration_ms:600,hold_ms:1000}]);
  updateScale();updateBehavior();
}

$('connectForm').onsubmit=connect;
$('headerStatus').onclick=()=>document.querySelector('.device-panel').scrollIntoView({behavior:matchMedia('(prefers-reduced-motion: reduce)').matches?'auto':'smooth',block:'start'});
$('applyBtn').onclick=applyPosition;$('stopBtn').onclick=stopMotion;
$('centerBtn').onclick=()=>{model.targetSource='pulse';setTarget((model.min+model.max)/2);applyPosition();};
document.querySelectorAll('[data-nudge]').forEach(button=>button.onclick=()=>{model.targetSource='pulse';setTarget(angleToPulse(Math.max(0,Math.min(model.degrees,pulseToAngle(model.target)+Number(button.dataset.nudge)))));applyPosition();});
$('positionSlider').oninput=()=>{model.targetSource='pulse';setTarget(Number($('positionSlider').value));clearTimeout(sliderTimer);if($('liveFollow').checked)sliderTimer=setTimeout(applyPosition,180);};
$('pulseInput').oninput=()=>{model.targetSource='pulse';};
$('angleInput').oninput=()=>{model.targetSource='angle';};
$('pulseInput').onchange=()=>{try{setTarget(numeric($('pulseInput').value,model.min,model.max,'目标脉宽'));}catch(error){showError(error);}};
$('angleInput').onchange=()=>{try{setTarget(angleToPulse(numeric($('angleInput').value,0,model.degrees,'目标角度')));}catch(error){showError(error);}};
document.querySelectorAll('input[name="behavior"]').forEach(input=>input.onchange=updateBehavior);
$('holdSeconds').onchange=updateBehavior;
$('holdSeconds').oninput=updateBehavior;
$('angleRange').onchange=()=>{try{model.degrees=numeric($('angleRange').value,1,360,'角度行程');updateScale();updateSequence();savePreferences();}catch(error){showError(error);}};
['outputBank','outputChannel'].forEach(id=>$(id).onchange=async()=>{model.configured=false;updateAvailability();savePreferences();if(model.connected)try{await inspectOutput();}catch(error){showError(error);}});
$('inspectBtn').onclick=async()=>{try{await inspectOutput();logEvent('已重新读取输出配置');}catch(error){showError(error);}};
$('addPointBtn').onclick=()=>{try{addPoint();savePreferences();}catch(error){showError(error);}};
$('smallSweepBtn').onclick=()=>{const middle=model.degrees/2;setPoints([{angle:Math.max(0,middle-5),duration_ms:600,hold_ms:500},{angle:Math.min(model.degrees,middle+5),duration_ms:600,hold_ms:500},{angle:middle,duration_ms:600,hold_ms:500}]);savePreferences();};
$('clearSequenceBtn').onclick=()=>{setPoints([]);savePreferences();};
['repeatCount','sequenceAfter'].forEach(id=>$(id).onchange=()=>{updateSequence();savePreferences();});
$('runSequenceBtn').onclick=runSequence;$('exportBtn').onclick=exportSequence;
$('importBtn').onclick=()=>$('importFile').click();$('importFile').onchange=()=>importSequence($('importFile').files[0]);
$('clearLogBtn').onclick=()=>{model.events=[];renderLog();};
document.addEventListener('keydown',event=>{if(event.key==='Escape'){event.preventDefault();stopMotion();}});
window.addEventListener('pagehide',()=>{if(model.lease){navigator.sendBeacon('/api',new Blob([JSON.stringify({op:'stop'})],{type:'application/json'}));}});

restorePreferences();updateAvailability();
setInterval(pollState,700);
(async()=>{try{const connection=await api({op:'connection'});if(connection.connected){setConnected(true);await inspectOutput();$('connectMessage').textContent='已连接设备，输出配置已核对。';message('设备已就绪，选择目标与动作方式后应用位置。');await pollState();}}catch(error){$('connectMessage').textContent=error.message;}})();
