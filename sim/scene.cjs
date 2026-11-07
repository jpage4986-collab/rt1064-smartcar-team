// Grayscale camera and bicycle model. All dimensions except map pixels use metres.
const readline = require('node:readline');
const W = 188, H = 120, MW = 590, MH = 341, ppm = 82, dt = 1/30;
const height = Number(process.argv[2]);
const pitch = Number(process.argv[3])*Math.PI/180;
const cp = Math.cos(pitch), sp = Math.sin(pitch);
const fov = 75*Math.PI/180, th = Math.tan(fov/2), tv = Math.tan(Math.atan(th*H/W));
const forward = new Float64Array(W*H), lateral = new Float64Array(W*H);
for (let x=0;x<W;x++) {
    const a=Math.atan(((x+.5)/W*2-1)*th), ca=Math.fround(Math.cos(a)), sa=Math.fround(Math.sin(a));
    for(let y=0;y<H;y++){
        const b=Math.atan((1-(y+.5)/H*2)*tv), cb=Math.cos(b), sb=Math.sin(b);
        const down=cb*ca*sp-sb*cp, k=y*W+x;
        forward[k]=down>.006?(cb*ca*cp+sb*sp)/down*height:NaN;
        lateral[k]=cb*sa/down*height;
    }
}
let road,edge,x,y,heading,speed,time,distance;
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
function sample(src,u,v){
    if(u<0||v<0||u>=MW-1||v>=MH-1)return 0;
    const ix=Math.floor(u),iy=Math.floor(v),fx=u-ix,fy=v-iy,k=iy*MW+ix;
    return src[k]*(1-fx)*(1-fy)+src[k+1]*fx*(1-fy)+src[k+MW]*(1-fx)*fy+src[k+MW+1]*fx*fy;
}
function capture(){
    const pixels=Buffer.alloc(W*H), ch=Math.cos(heading), sh=Math.sin(heading), frameNo=Math.floor(time*25);
    for(let j=0;j<H;j++)for(let i=0;i<W;i++){
        const k=j*W+i, f=forward[k], s=lateral[k];
        let cov=0,border=0,mark=false,brick=-1;
        if(Number.isFinite(f)){
            const gx=x+ppm*(f*ch-s*sh),gy=y+ppm*(f*sh+s*ch);
            cov=sample(road,gx,gy);border=sample(edge,gx,gy);
            const sy=(gy-20)/ppm+.15;
            mark=cov>.7&&Math.abs(gx-228)<.145*ppm&&sy>=0&&sy<.30&&Math.floor(sy/(.30/13))%2===0;
            const vx=gx-x,vy=gy-y;
            let lo=1-.05/height,hi=1;
            if(Math.abs(vx)<1e-8){if(x<192||x>211.68)hi=-1;}
            else{const a=(192-x)/vx,b=(211.68-x)/vx;lo=Math.max(lo,Math.min(a,b));hi=Math.min(hi,Math.max(a,b));}
            if(Math.abs(vy)<1e-8){if(y<310||y>321.5)hi=-1;}
            else{const a=(310-y)/vy,b=(321.5-y)/vy;lo=Math.max(lo,Math.min(a,b));hi=Math.min(hi,Math.max(a,b));}
            if(lo<=hi && lo>=0)brick=85+(Math.abs(lo-(1-.05/height))<1e-6?8:0);
        }
        const z=Math.sin(i*127.1+j*311.7+frameNo*74.7)*43758.5453;
        const texture=4*Math.sin((x+i*1.7+j*.4)*.31+(y+j*1.6-i*.25)*.19)+(2*(z-Math.floor(z))-1)*10;
        const ambient=5*Math.sin(j*.09+time*.7)+3*Math.sin(i*.07-j*.045);
        pixels[k]=brick>=0?clamp(brick+texture,0,255):mark?22:clamp(43+cov*(150+ambient)+texture-border*58,0,255);
    }
    return pixels.toString('base64');
}
function update(p){
    let off=false;
    if(p.map){
        road=Buffer.from(p.map,'base64');if(road.length!==MW*MH)throw Error('map size');
        edge=Buffer.alloc(MW*MH);
        for(let j=1;j<MH-1;j++)for(let i=1;i<MW-1;i++){
            const k=j*MW+i;if(road[k]&&(!road[k-1]||!road[k+1]||!road[k-MW]||!road[k+MW]))edge[k]=1;
        }
        [x,y,heading]=p.pose;speed=distance=time=0;
    }else{
        for(let i=0;i<3;i++){
            speed+=clamp(.5*p.duty/15000-speed,-4*dt,4*dt);
            time+=dt;
            const wheel=clamp((88-p.servo)*2.5/45,-1,1)*45*Math.PI/180;
            heading+=speed/.18*Math.tan(wheel)*dt;
            x+=Math.cos(heading)*speed*ppm*dt;y+=Math.sin(heading)*speed*ppm*dt;distance+=speed*dt;
            if(!road[Math.round(y)*MW+Math.round(x)]){off=true;break;}
        }
    }
    return {x,y,heading,speed,time,distance,off,gray:capture()};
}
readline.createInterface({input:process.stdin}).on('line',line=>console.log(JSON.stringify(update(JSON.parse(line)))));
