"""Start the local panel and open it in the default browser."""
from pathlib import Path
import subprocess
import sys
import time
import webbrowser
import json
from urllib.request import urlopen

root = Path(__file__).resolve().parent
address = 'http://127.0.0.1:10640'

def online():
    try:
        with urlopen(address + '/api/state', timeout=1) as response:
            return json.load(response).get('root') == str(root)
    except (OSError, ValueError):
        return False

if not online():
    logs = root/'reports/tuning'
    logs.mkdir(parents=True, exist_ok=True)
    with (logs/'server.log').open('a', encoding='utf8') as log:
        process = subprocess.Popen([sys.executable, '-X', 'utf8', str(root/'tools/tuning/panel.py')],
                                   cwd=root, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                   creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    for _ in range(40):
        if online():break
        if process.poll() is not None:break
        time.sleep(.15)
    if not online():raise SystemExit(f'Panel startup failed. See {logs / "server.log"}')
print(address)
webbrowser.open(address)
