"""Record backups on the current Windows user's actual Desktop."""
from datetime import datetime
from pathlib import Path
import ctypes


def record_backup(source, destination, contents, purpose):
    buffer = ctypes.create_unicode_buffer(32768)
    if ctypes.windll.shell32.SHGetFolderPathW(None, 0x10, None, 0, buffer) != 0:
        raise RuntimeError('Cannot locate Desktop for backup path logging')
    log = Path(buffer.value) / '备份地址记录.txt'
    with log.open('a', encoding='utf-8') as f:
        f.write(f'\n[{datetime.now().astimezone().isoformat()}] 来源：{source}；备份：{destination}；内容：{contents}；目的：{purpose}。\n')
