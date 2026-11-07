"""Camera-only closed loop. Course coordinates are used by the scorer and renderer."""
import argparse
import base64
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
from PIL import Image
from firmware import Firmware

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent

def excursion(rows,center):
    """Largest continuous angular excursion within the island's approach region."""
    previous=None
    total=peak=0.
    for r in rows:
        dx,dy=r['x']-center[0],r['y']-center[1]
        if math.hypot(dx,dy)>=70:
            previous=None;total=0
            continue
        angle=math.atan2(dy,dx)
        if previous is not None:
            total+=math.remainder(angle-previous,math.tau)
        peak=max(peak,abs(total));previous=angle
    return round(math.degrees(peak),1)

def run(args,direction):
    mask=Image.open(HERE/'home-track.png').convert('L').point(lambda x:1 if x>128 else 0)
    cpu=Firmware(args.elf)
    child=subprocess.Popen(['node',str(HERE/'scene.cjs'), str(args.height_cm/100), str(args.pitch_deg)],text=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
    def rpc(message):
        child.stdin.write(json.dumps(message)+'\n');child.stdin.flush()
        return json.loads(child.stdout.readline())
    frames=[]
    try:
        heading = 0 if direction == 'cw' else math.pi
        offset_px = args.start_offset_cm * .82
        state=rpc(dict(map=base64.b64encode(mask.tobytes()).decode(),
                       pose=[228, 20 + math.cos(heading)*offset_px,
                             heading + math.radians(args.start_yaw_deg)]))
        stationary=0
        for i in range(1201):
            control=cpu.step(base64.b64decode(state.pop('gray')))
            frames.append(dict(**state,**control))
            stationary=stationary+1 if control['duty']==0 and state['speed']==0 else 0
            if state['off']:reason='offroad';break
            if control['lap']==2 and state['speed']==0:
                reason='finish' if state['distance']>15 and math.hypot(state['x']-228,state['y']-20)<85 else 'early_finish'
                break
            if stationary>=20:reason='stopped';break
            if i==1200:reason='time_limit';break
            state=rpc(dict(servo=control['servo'],duty=control['duty']))
    finally:
        child.stdin.close();child.wait(timeout=10)
    args.output.mkdir(parents=True,exist_ok=True)
    with (args.output/(direction+'.csv')).open('w',newline='',encoding='utf8') as out:
        writer=csv.DictWriter(out,fieldnames=frames[0]);writer.writeheader();writer.writerows(frames)
    result=dict(direction=direction,height_cm=args.height_cm,pitch_deg=args.pitch_deg,
                start_offset_cm=args.start_offset_cm,start_yaw_deg=args.start_yaw_deg,reason=reason,elapsed_s=round(state['time'],2),
                distance_m=round(state['distance'],3),right_ring_deg=excursion(frames,(512,142)),
                central_ring_deg=excursion(frames,(326,183)),obstacle_seen=any(r['obstacle'] and r['y']>278 for r in frames),
                elf_sha256=hashlib.sha256(args.elf.read_bytes()).hexdigest())
    (args.output/(direction+'.json')).write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
    return reason=='finish' and result['right_ring_deg']>=300 and result['central_ring_deg']>=300 and result['obstacle_seen']

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--elf',type=Path,default=ROOT/'Project/GCC/wireless-app/tutorial.elf')
    parser.add_argument('--output',type=Path,default=ROOT/'reports')
    parser.add_argument('--direction',choices=('cw','ccw','both'),default='both')
    parser.add_argument('--height-cm',type=float,default=31.5)
    parser.add_argument('--pitch-deg',type=float,default=30.)
    parser.add_argument('--start-offset-cm', type=float, default=0, help='starting displacement to the right')
    parser.add_argument('--start-yaw-deg', type=float, default=0, help='clockwise heading offset')
    args=parser.parse_args()
    if not (0 < args.height_cm <= 100 and 0 < args.pitch_deg < 90):
        parser.error('height must be in (0,100] cm and downward pitch in (0,90) degrees')
    results=[run(args,d) for d in (('cw','ccw') if args.direction=='both' else (args.direction,))]
    raise SystemExit(0 if all(results) else 1)
