'use strict';
const $ = id => document.getElementById(id);
const W=188, H=120, N=W*H;
let settings, baseline, draft, footage=null, busy=false, lastMapKey='', map=null;
const controls = new Map();
const val = key => draft['TRACK_'+key];
function showMessage(s){ $('msg').textContent=s; }
function buttons(on){busy=on;for(const key of ['save','deploy','reset']) $(key).disabled=on; $('groups').inert=on;}
async function loadState(restore=true){
    const response=await fetch('/api/state');settings=await response.json();
    baseline=settings.values;
    draft={...(restore && settings.draft?.revision===settings.revision ? settings.draft.values : baseline)};
    renderFields(); buttons(settings.busy);refresh();
    showMessage(settings.busy ? settings.status : (restore && settings.draft?.revision===settings.revision?'已载入上次保存的参数':'已读取本地工程参数'));
}
function renderFields(){
    const holder=$('groups');holder.replaceChildren();controls.clear();const parts={};
    for(const [key,f] of Object.entries(settings.fields)){
        if(!parts[f.group]){
            const part=document.createElement('details'),title=document.createElement('summary');
            part.open=Object.keys(parts).length===0;title.textContent=f.group;part.append(title);holder.append(part);parts[f.group]=part;
        }
        const row=document.createElement('div');row.className='field';
        const label=document.createElement('label');label.htmlFor=key;label.textContent=f.label;
        const unit=document.createElement('small');unit.textContent=f.unit;label.append(unit);row.title=key;row.append(label);
        const slider=document.createElement('input'),box=document.createElement('input');
        slider.type='range';box.type='number';box.id=key;
        for(const el of [slider,box]){
            el.min=f.min;el.max=f.max;el.step=f.step;el.value=draft[key];el.setAttribute('aria-label',f.label);
            el.addEventListener('input',()=>{
                if(!el.validity.valid || el.value==='')return;
                draft[key]=Number(el.value);(el===box?slider:box).value=el.value;
                showMessage('正在编辑 · 保存后可供烧录');refresh();
            });
            row.append(el);
        }
        box.onblur=()=>{box.value=draft[key];};parts[f.group].append(row);controls.set(key,row);
    }
    $('count').textContent=controls.size+' 项';
}
function buildMap(){
    const height=val('A16_CAMERA_HEIGHT_M'),pitch=val('A16_CAMERA_PITCH_DEG')*Math.PI/180,key=height+'/'+pitch;
    if(lastMapKey===key)return; lastMapKey=key;map=new Int32Array(N).fill(-1);
    const cp=Math.cos(pitch),sp=Math.sin(pitch);
    for(let x=0;x<W;x++){
        const u=(x-93.5)/122.5,c=1/Math.sqrt(1+u*u);
        for(let y=0;y<H;y++){
            const v=(59.5-y)/122.5,down=c*.258819-v*.965926,forward=.23*(c*.965926+v*.258819);
            const lateral=.23*u*c,h=height*down,depth=forward*cp+h*sp,up=forward*sp-h*cp;
            const sx=Math.floor(94+122.5*lateral/depth),sy=Math.floor(60-122.5*up/Math.hypot(depth,lateral));
            if(down>0&&sx>=0&&sx<W&&sy>=0&&sy<H)map[y*W+x]=sy*W+sx;
        }
    }
}
function cutoff(frame){
    const hist=new Uint16Array(32);let n=0;
    for(let y=32;y<=116;y+=4)for(let x=4;x<184;x+=4){hist[frame[y*W+x]>>3]++;n++;}
    let low=0,high=0,sum=0,t=val('MODE16_GRAY_THRESHOLD');
    for(let i=0;i<32;i++){sum+=hist[i];if(sum*10<n)low=i+1;if(sum*10>=9*n){high=i;break;}}
    if(high>=low+10 && high*8<t+24)t=(low+3*high)*2;
    if(high>=low+4 && (high<low+10 || high>=30)){
        const offset=high<low+10?4:0,mid=(low+high)*4+offset;
        t=mid+Math.floor(((low+3*high)*2+offset-mid)*100/val('A16_CONTRAST_GAIN_PERCENT'));
    }
    return t;
}
function draw(id,gray){
    const ctx=$(id).getContext('2d'),pixels=ctx.createImageData(W,H);
    for(let i=0;i<N;i++){pixels.data.fill(gray[i],4*i,4*i+3);pixels.data[4*i+3]=255;}
    ctx.putImageData(pixels,0,0);return ctx;
}
function preview(){
    if(!footage)return;
    const i=Number($('frame').value)-1,frame=footage.subarray(i*N,(i+1)*N),t=cutoff(frame);buildMap();
    const binary=new Uint8Array(N);
    for(let y=2;y<H;y++)for(let x=2;x<W-2;x++){const k=y*W+x;binary[k]=map[k]>=0 && frame[map[k]]>t?255:0;}
    const clean=binary.slice();
    for(let y=1;y<H-1;y++)for(let x=1;x<W-1;x++){
        const k=y*W+x;let count=0;
        for(let j=-1;j<=1;j++)for(let a=-1;a<=1;a++)if((a||j)&&binary[k+j*W+a])count++;
        if(binary[k]===0 && count>=5)clean[k]=255;
        if(binary[k] && count<=2)clean[k]=0;
    }
    draw('raw',frame);const ctx=draw('bin',clean);
    for(const [row,color] of [[val('A16_LOOKAHEAD_ROW'),'#008ef2'],[val('A16_NEAR_ROW'),'#e57c00']]){
        ctx.fillStyle=color;for(let x=0;x<W;x+=6)ctx.fillRect(x,row,4,1);
    }
    $('threshold').textContent='· 阈值 '+t;$('frameNo').textContent=(i+1)+' / '+footage.length/N;
}
function refresh(){
    const changes=[];
    for(const [key,row] of controls){
        const diff=draft[key]!==baseline[key];row.classList.toggle('changed',diff);
        if(diff)changes.push(settings.fields[key].label+'　'+baseline[key]+' → '+draft[key]+' '+settings.fields[key].unit);
    }
    $('changes').textContent=changes.join('\n')||'与工程配置一致';
    const center=val('SERVO_CENTER_DEG'),limit=val('A16_MAX_STEER_DELTA_DEG');
    const cap=val('WHEEL_PWM_MAX_PERCENT'),floor=Math.min(cap,val('MOTOR_MIN_MOVING_PERCENT'));
    const dutyAt=pct=>[1,2].map(m=>Math.min(100,pct*val('MOTOR'+m+'_OUTPUT_SCALE_PERCENT')/100).toFixed(1)+'%').join(' / ');
    const px=Number($('err').value);
    const dev=Math.trunc(px*(val('STEER_GAIN_CENTIDEG_PER_PIXEL')+3)/100);
    const servo=Math.min(val('SERVO_LEFT_DEG'),center+limit,Math.max(val('SERVO_RIGHT_DEG'),center-limit,center-dev));
    $('response').textContent=px+' px → '+servo+'°';
    const estimate=val('SPEED_PID_ENABLED')?'闭环调节':dutyAt(Math.max(floor,cap*(1-val('CURVE_SLOWDOWN_PERCENT')/100)));
    $('metrics').replaceChildren();
    for(const [name,value] of [['普通路舵机范围',Math.max(center-limit,val('SERVO_RIGHT_DEG'))+'°–'+Math.min(center+limit,val('SERVO_LEFT_DEG'))+'°'],['直道 M1 / M2',val('SPEED_PID_ENABLED')?'闭环调节':dutyAt(cap)],['机械满舵 M1 / M2',estimate]]){
        const el=document.createElement('div'),title=document.createElement('small'),num=document.createElement('strong');title.textContent=name;num.textContent=value;el.append(title,num);$('metrics').append(el);
    }
    preview();
}
function setFootage(bytes,name){
    if(!bytes.length || bytes.length%N)throw Error('灰度文件大小须为 188 × 120 字节的整数倍');
    footage=bytes;$('frame').max=bytes.length/N;$('frame').value=Math.min(8,bytes.length/N);
    $('frameName').textContent=name;$('empty').textContent='';preview();
}
$('file').onchange=async e=>{
    const f=e.target.files[0];if(!f)return;
    try{
        if(f.type.startsWith('image/')){
            const img=await createImageBitmap(f);
            if(img.width*120!==img.height*188)throw Error('请导入 188:120 比例的车载原始画面');
            const c=document.createElement('canvas');c.width=W;c.height=H;const g=c.getContext('2d');g.drawImage(img,0,0,W,H);img.close();
            const rgb=g.getImageData(0,0,W,H).data;
            setFootage(Uint8Array.from({length:N},(_,i)=>Math.round(.299*rgb[i*4]+.587*rgb[i*4+1]+.114*rgb[i*4+2])),f.name);
        }else setFootage(new Uint8Array(await f.arrayBuffer()),f.name);
    }catch(e){showMessage(e.message);}
};
$('frame').oninput=preview;
$('err').oninput=refresh;
$('reset').onclick=()=>{draft={...baseline};renderFields();refresh();showMessage('已恢复当前工程参数');};
async function submit(action){
    buttons(true);
    try{
        const r=await fetch('/api/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-Panel-Token':settings.token},body:JSON.stringify({revision:settings.revision,values:draft})});
        const s=await r.json();if(!r.ok)throw Error(s.error);
        buttons(s.busy);showMessage(s.status);if(s.busy)watch();
    }catch(e){buttons(false);showMessage(e.message);}
}
$('save').onclick=()=>submit('save');
$('deploy').onclick=()=>submit('deploy');
async function watch(){
    try{
        const r=await fetch('/api/status');const s=await r.json();showMessage(s.status);$('log').hidden=false;
        if(s.busy){setTimeout(watch,1200);return;}
        await loadState(false);showMessage(s.status);
    }catch(e){showMessage('暂时无法读取烧录状态，正在重连');setTimeout(watch,2500);}
}
(async()=>{
    try{
        await loadState();
        const r=await fetch('/camera.gray');
        if(r.ok)setFootage(new Uint8Array(await r.arrayBuffer()),'历史车载实拍');
        else $('empty').textContent='导入车载灰度帧或图片即可预览';
        if(settings.busy)watch();
    }catch(e){showMessage(e.message);}
})();
