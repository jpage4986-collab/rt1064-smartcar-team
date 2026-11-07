"""Local A16 tuning panel. Run with Python 3 on the workstation that owns the DAP."""
import argparse
import hashlib
import json
import math
import os
import re
import secrets
import subprocess
import tempfile
import threading
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
HEADER = ROOT / "Project/USER/inc/tune_config.h"
OUT = ROOT / "reports/tuning"
DRAFT = OUT / "pending.json"
TOKEN = secrets.token_urlsafe(32)
FIELDS = {}
# Name, Chinese label, unit, minimum, maximum, step.
def group(title, spec):
    for line in spec.strip().splitlines():
        name, label, unit, low, high, step = line.split()
        FIELDS["TRACK_" + name] = dict(group=title, label=label, unit="" if unit=="-" else unit,
                                       min=float(low), max=float(high), step=float(step))

group("视觉与前视", """
MODE16_GRAY_THRESHOLD 基础灰度阈值 灰阶 1 254 1
A16_CONTRAST_GAIN_PERCENT 二值化对比度 % 100 200 1
A16_ENTRY_CONFIRM_FRAMES 环岛入口确认帧数 帧 2 6 1
A16_LOOKAHEAD_ROW 前视目标行 行 73 115 1
A16_NEAR_ROW 近处锚点行 行 74 118 1
A16_NEAR_GAIN_PERCENT 近处偏差补偿 % 0 100 1
A16_CAMERA_HEIGHT_M 摄像头高度 m 0.1 0.65 0.001
A16_CAMERA_PITCH_DEG 向下俯角 ° 5 60 0.1
""")
group("转向", """
STEER_GAIN_CENTIDEG_PER_PIXEL 转向增益 0.01°/px 1 100 1
A16_MAX_STEER_DELTA_DEG 普通路最大舵角偏移 ° 1 32 1
A16_BEND_STEP_DEG 明显弯道打舵步长 °/帧 1 30 1
A16_SUSTAINED_BEND_STEP_DEG 持续弯道打舵步长 °/帧 1 30 1
A16_ROAD_RETURN_STEP_DEG 普通路快速回正步长 °/帧 1 30 1
SERVO_CENTER_DEG 舵机中位 ° 74 104 1
SERVO_LEFT_DEG 左机械限位 ° 74 105 1
SERVO_RIGHT_DEG 右机械限位 ° 73 104 1
STEER_REFERENCE_X 环岛转向参考列 px 1 186 1
""")
group("电机与减速", """
WHEEL_PWM_MAX_PERCENT 基础占空比上限 % 0 100 1
CURVE_SLOWDOWN_PERCENT 满舵减速 % 0 100 1
MOTOR_MIN_MOVING_PERCENT 行驶占空比下限 % 0 100 1
MOTOR1_OUTPUT_SCALE_PERCENT 左右轮倍率·M1 % 0 150 1
MOTOR2_OUTPUT_SCALE_PERCENT 左右轮倍率·M2 % 0 150 1
MOTOR_RAMP_STEP_PERCENT_PER_FRAME 每帧加速步长 % 1 100 1
MOTOR_PWM_HZ 电机PWM频率 Hz 1000 25000 1
MOTOR1_FORWARD_LEVEL M1前进电平 - 0 1 1
MOTOR2_FORWARD_LEVEL M2前进电平 - 0 1 1
""")
group("丢线与停车", """
A16_SEARCH_PWM_PERCENT 找路占空比 % 0 100 1
A16_SEARCH_TIME_MS 找路时限 ms 1 5000 1
CAMERA_LOST_TIMEOUT_MS 断帧停车 ms 30 1000 1
DRIVE_MIN_ANCHOR_CONFIDENCE 锚点最低置信度 - 1 100 1
DRIVE_MIN_TARGET_CONFIDENCE 目标最低置信度 - 1 100 1
DRIVE_MIN_VALID_ROWS 最少有效行 行 1 116 1
""")
group("编码器与速度 PID", """
SPEED_PID_ENABLED 速度PID开关 - 0 1 1
ENCODER1_REVERSE M1编码器反向 - 0 1 1
ENCODER2_REVERSE M2编码器反向 - 0 1 1
SPEED_TARGET_COUNTS_PER_10MS 每10ms目标计数 - 0 500 1
SPEED_PID_KP 比例增益 - 0 1000 1
SPEED_PID_KI 积分增益 - 0 1000 1
SPEED_PID_KD 微分增益 - 0 1000 1
SPEED_PID_IMAX 积分限幅 - 0 20000 1
SPEED_PID_OUTMAX 输出限幅 - 0 50000 1
""")
group("道路扫描", """
OTSUSCAN_SCAN_TOP_ROW 扫描顶行 行 16 72 1
OTSUSCAN_ANCHOR_ROW 基础扫描锚点 行 49 118 1
OTSUSCAN_TARGET_ROW 基础扫描目标行 行 17 117 1
""")
define = re.compile(r"^(#define (TRACK_\w+) )([\d.]+)([uUfF]?)(?=\s|$)", re.M)
lock = threading.RLock()
busy = False
status = "就绪"
def source():
    data = HEADER.read_bytes()
    return data.decode(), hashlib.sha256(data).hexdigest()

def values(text):
    return {name: float(value) for _,name,value,_ in define.findall(text) if name in FIELDS}

def validate(v):
    if not isinstance(v,dict) or set(v)!=set(FIELDS):
        raise ValueError("参数不完整，请刷新面板")
    for key,field in FIELDS.items():
        x=v[key]
        if type(x) not in (float,int) or not math.isfinite(x) or not field['min']<=x<=field['max'] or abs(x/field['step']-round(x/field['step']))>1e-6:
            raise ValueError(field['label']+" 超出范围或精度")
    if v['TRACK_A16_LOOKAHEAD_ROW']>=v['TRACK_A16_NEAR_ROW']:raise ValueError("前视行必须小于近视行")
    if not v['TRACK_SERVO_RIGHT_DEG']<v['TRACK_SERVO_CENTER_DEG']<v['TRACK_SERVO_LEFT_DEG']:raise ValueError("舵机中位须在左右限位之间")
    if not v['TRACK_OTSUSCAN_SCAN_TOP_ROW']<v['TRACK_OTSUSCAN_TARGET_ROW']<v['TRACK_OTSUSCAN_ANCHOR_ROW']:raise ValueError("基础扫描行顺序有误")
    if v['TRACK_A16_SEARCH_TIME_MS']==0:raise ValueError("找路时限需大于零")

def atomic(path,data):
    with tempfile.NamedTemporaryFile(dir=path.parent,delete=False) as temp:
        temp.write(data)
    os.replace(temp.name,path)

def render(text,v):
    def replace(m):
        prefix,name,old,suffix=m.groups()
        if name not in v or float(old)==v[name]:return m[0]
        literal=str(float(v[name])) if suffix=='f' else str(int(v[name]))
        return prefix+literal+suffix
    return define.sub(replace,text).encode()

def deploy(payload):
    global busy,status
    try:
        text,rev=source()
        if rev!=payload['revision']:raise ValueError("源码已变化，请刷新")
        validate(payload['values'])
        atomic(HEADER,render(text,payload['values']))
        with (OUT/'last-flash.log').open('w',encoding='utf-8') as log:
            for script,stage in [('build.ps1','正在编译'),('flash.ps1','正在烧录和回读')]:
                status=stage
                subprocess.run(['pwsh','-NoProfile','-File',str(ROOT/script)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
        report=json.loads((ROOT/'reports/flash/latest-wired-update.json').read_text(encoding='utf8'))
        status="烧录并回读成功 · CRC32 "+report['crc32'].upper()
    except Exception as e:
        status="未完成 · "+str(e)+" · 见烧录日志"
    finally:
        with lock:busy=False

class Handler(BaseHTTPRequestHandler):
    def reply(self,code,data,mime='application/json'):
        if mime=='application/json':data=json.dumps(data,ensure_ascii=False).encode()
        self.send_response(code);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store')
        self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    def do_GET(self):
        if self.path in ('/','/panel.js','/style.css'):
            name,typ={'/':('index.html','text/html; charset=utf-8'),'/panel.js':('panel.js','text/javascript; charset=utf-8'),'/style.css':('style.css','text/css; charset=utf-8')}[self.path]
            return self.reply(200,(HERE/name).read_bytes(),typ)
        if self.path=='/api/state':
            with lock:
                text,rev=source()
                draft=json.loads(DRAFT.read_bytes()) if DRAFT.exists() else None
                return self.reply(200,dict(root=str(ROOT),values=values(text),fields=FIELDS,revision=rev,token=TOKEN,draft=draft,busy=busy,status=status))
        if self.path=='/api/status': return self.reply(200,dict(busy=busy,status=status))
        if self.path=='/camera.gray' and (OUT/'camera.gray').is_file():
            return self.reply(200,(OUT/'camera.gray').read_bytes(),'application/octet-stream')
        if self.path=='/flash.log' and (OUT/'last-flash.log').exists():
            return self.reply(200,(OUT/'last-flash.log').read_bytes(),'text/plain; charset=utf-8')
        self.reply(404,{"error":"无文件"})
    def do_POST(self):
        global busy,status
        if self.path not in ('/api/save','/api/deploy') or self.headers.get('X-Panel-Token')!=TOKEN or self.headers.get('Origin')!=f'http://127.0.0.1:{self.server.server_port}':
            return self.reply(403,{"error":"无效请求来源"})
        try:
            length=int(self.headers.get('Content-Length',0))
            if not 0<length<=16384:raise ValueError("无效参数长度")
            body=json.loads(self.rfile.read(length))
            with lock:
                if busy:return self.reply(409,{"error":"烧录中"})
                _,rev=source()
                if body.get('revision')!=rev:return self.reply(409,{"error":"源码参数已更新，请刷新页面"})
                validate(body.get('values'))
                body=dict(values=body['values'],revision=rev,saved=datetime.now().isoformat(timespec='seconds'))
                atomic(DRAFT,json.dumps(body,ensure_ascii=False,indent=2).encode())
                if self.path=='/api/deploy':
                    busy=True;status='准备编译';threading.Thread(target=deploy,args=(body,),daemon=True).start()
                else:status="待烧录参数已保存 · "+body['saved'][11:]
            self.reply(200,dict(status=status,busy=busy))
        except (ValueError,TypeError,KeyError) as e:self.reply(400,{"error":str(e)})

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=10640);args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    text,_=source()
    if set(values(text))!=set(FIELDS):raise SystemExit("tune_config.h 缺少参数")
    ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever()
