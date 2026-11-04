// Test the real parsing/command functions with inert DOM and network seams.
// This runs no browser, opens no socket, and cannot move physical hardware.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../panel/servo_panel.js'),'utf8');
const functions=source.slice(0,source.indexOf("$('connectForm').onsubmit=connect;"));

function harness(points=[{angle:85,duration_ms:600,hold_ms:500}]) {
  const elements={};
  const element=id=>elements[id] ||= {value:'',style:{},textContent:'',classList:{toggle(){}}};
  element('sequenceBody').rows=points.map(point=>({querySelector:selector=>({value:String(point[
    {'.point-angle':'angle','.point-duration':'duration_ms','.point-hold':'hold_ms'}[selector]])})}));
  element('repeatCount').value='2';element('sequenceAfter').value='hold';
  element('pulseInput').value='1500';element('angleInput').value='95';element('holdSeconds').value='0.5';
  const context={console,Blob,AbortController,setTimeout:()=>1,clearTimeout(){},queueMicrotask(){},
    localStorage:{setItem(){}}, document:{getElementById:element,
      querySelector:()=>({value:'pulse'}),createElement:()=>({click(){},append(){}})},
    URL:{createObjectURL(blob){context.exportedBlob=blob;return 'blob:test';},revokeObjectURL(){}}};
  vm.createContext(context);vm.runInContext(functions,context);
  vm.runInContext(`logEvent=()=>{};showError=error=>{globalThis.lastError=error.message;};
    setPoints=points=>{globalThis.imported=points;};updateSequence=()=>{};savePreferences=()=>{};
    updateAvailability=()=>{};message=()=>{};pollState=async()=>{};
    api=async payload=>{globalThis.command=payload;return {lease_id:'test-session'};};`,context);
  return {context,elements,element};
}

test('export contains actual rows, repetitions and end behavior',async()=>{
  const {context}=harness();vm.runInContext('exportSequence()',context);
  const result=JSON.parse(await context.exportedBlob.text());
  assert.deepEqual(result,{version:1,points:[{angle:85,duration_ms:600,hold_ms:500}],repeat:2,after:'hold'});
});

test('exported documents import with the same values',async()=>{
  const {context,elements}=harness();vm.runInContext('exportSequence()',context);
  const text=await context.exportedBlob.text();context.file={size:text.length,text:async()=>text};
  await vm.runInContext('importSequence(file)',context);
  assert.deepEqual(JSON.parse(JSON.stringify(context.imported)),[{angle:85,duration_ms:600,hold_ms:500}]);
  assert.equal(elements.repeatCount.value,2);assert.equal(elements.sequenceAfter.value,'hold');
});

test('invalid import never replaces the existing sequence',async()=>{
  for(const point of [{angle:null,duration_ms:1,hold_ms:1},{angle:'0',duration_ms:1,hold_ms:1},
      {angle:181,duration_ms:1,hold_ms:1},{angle:90,duration_ms:30001,hold_ms:1}]){
    const {context}=harness();const text=JSON.stringify({version:1,points:[point],repeat:1,after:'center'});
    context.file={size:text.length,text:async()=>text};await vm.runInContext('importSequence(file)',context);
    assert.equal(context.imported,undefined);assert.ok(context.lastError);
  }
});

test('Apply uses edited pulse input even without a blur/change event',async()=>{
  const {context,element}=harness();element('pulseInput').value='1530';
  vm.runInContext("model.connected=true;model.configured=true;model.targetSource='pulse';",context);
  await vm.runInContext('applyPosition()',context);
  assert.equal(context.command.norm,0.06);assert.equal(context.command.hold_s,0.5);
});

test('Apply uses edited angle input rather than an older pulse value',async()=>{
  const {context}=harness();
  vm.runInContext("model.connected=true;model.configured=true;model.targetSource='angle';",context);
  await vm.runInContext('applyPosition()',context);
  assert.ok(context.command.norm>0.05 && context.command.norm<0.06);
});
